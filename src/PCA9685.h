#pragma once

#include <algorithm>
#include <cstdint>
#include <span>
#include <cmath>
#include <cstring>
#include <array>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_timer.h>

#include <driver/i2c_master.h>

namespace YOBA {
	enum class PCA9685Error : uint8_t {
		none,
		I2C,
		invalidArgument
	};

	enum class PCA9685OutputChangeMode : uint8_t {
		// Channel outputs will be simultaneously updated on the end of I2C transaction
		stop,
		// Channel outputs will be updated subsequentially on I2C transaction ACK frame, i.e. on every channel in sequence - should be used only with setAutoIncrement(true)
		acknowledgment
	};

	enum class PCA9685OutputDriverMode : uint8_t {
		// Uses internal pull-up resistors (default mode)
		totemPole,
		// Requires external pull-up resistors and allows to attach heavy load with custom voltage range (see datasheet for details)
		openDrain
	};

	// Defines channel behavior when OE pin is being in HIGH state
	enum class PCA9685OutputDisabledMode : uint8_t {
		// When OE pin is HIGH, channel output will be LOW (default)
		low,
		// When OE pin is HIGH, channel output will be HIGH (only for PCA9685OutputDriverMode = totemPole. In PCA9685OutputDisabledMode = openDrain behavior will be the same as floating)
		high,
		// When OE pin is HIGH, channel output will be in floating (or high-impedance) state, can be used to operate heavy load P-channel MOSFETs
		floating
	};

	class PCA9685 {
		public:
			// Use soldering iron to change address of your module. Then you can use custom address via PCA9685::I2CBaseAddress | 0b00000001
			constexpr static uint8_t I2CBaseAddress = 0x40;
			constexpr static uint32_t I2CDefaultFrequency = 400'000;
			constexpr static uint8_t channelCount = 16;

			// Manual setup for DS/Nioh players
			PCA9685Error setup(
				const i2c_master_bus_handle_t& I2CBus,
				const uint8_t I2CAddress = I2CBaseAddress,
				const uint32_t I2CFrequencyHz = I2CDefaultFrequency
			) {
				return internalSetup(
					I2CBus,
					I2CAddress,
					I2CFrequencyHz
				);
			}

			// Easy setup with pre-defined stuff in correct order for pussyboys
			PCA9685Error setup(
				const i2c_master_bus_handle_t& I2CBus,
				const uint8_t I2CAddress = I2CBaseAddress,
				const uint32_t I2CFrequencyHz = I2CDefaultFrequency,

				const uint32_t PWMFrequencyHz = 50,
				const PCA9685OutputDriverMode outputDriverMode = PCA9685OutputDriverMode::totemPole,
				const PCA9685OutputChangeMode outputChangeMode = PCA9685OutputChangeMode::stop,
				const PCA9685OutputDisabledMode outputDisabledMode = PCA9685OutputDisabledMode::low,
				const bool outputInverted = false,
				const bool autoIncrement = true
			) {
				auto error = internalSetup(
					I2CBus,
					I2CAddress,
					I2CFrequencyHz
				);

				if (error != PCA9685Error::none)
					return error;

				// -------------------------------- Initialization --------------------------------

				error = setOutputDriverMode(outputDriverMode);
				if (error != PCA9685Error::none)
					return error;

				error = setOutputChangeMode(outputChangeMode);
				if (error != PCA9685Error::none)
					return error;

				error = setOutputDisabledMode(outputDisabledMode);
				if (error != PCA9685Error::none)
					return error;

				error = setOutputInverted(outputInverted);
				if (error != PCA9685Error::none)
					return error;

				error = setAutoIncrement(autoIncrement);
				if (error != PCA9685Error::none)
					return error;

				error = setFrequency(PWMFrequencyHz);
				if (error != PCA9685Error::none)
					return error;

				return PCA9685Error::none;
			}

			PCA9685Error setFrequency(const uint32_t frequencyHz) const {
				if (frequencyHz < 24 || frequencyHz > 1526) {
					ESP_LOGE(LOG_TAG, "frequency %d is out of range [24; 1526] Hz", frequencyHz);
					return PCA9685Error::invalidArgument;
				}

				// prescale = round(osc_clock / (4096 * frequencyHz)) - 1
				// osc_clock = 25 MHz
				const auto prescale =  std::clamp<uint8_t>(
					static_cast<uint8_t>(25'000'000 / (4096 * frequencyHz)),
					3,
					255
				);

				// Reading current mode1 reg value
				uint8_t mode1 = 0;

				auto error = readRegister(REG_MODE1, mode1);

				if (error != PCA9685Error::none)
					return error;

				delayUs(500);

				// Prescale can be updated only in sleep mode, so...
				error = writeRegister(REG_MODE1, (mode1 & ~REG_MODE1_RESTART) | REG_MODE1_SLEEP);

				if (error != PCA9685Error::none)
					return error;

				// Updating prescale
				error = writeRegister(REG_PRESCALE, prescale);

				if (error != PCA9685Error::none)
					return error;

				// Leaving sleep mode & moving to restart mode
				error = writeRegister(REG_MODE1, (mode1 & ~REG_MODE1_SLEEP) | REG_MODE1_RESTART);

				if (error != PCA9685Error::none)
					return error;

				// Datasheet says that when leaving sleep mode, we should give oscillator at least of 500 us to gain some pussy juice
				delayUs(500);

				// readRegister(REG_MODE1, mode1);
				// ESP_LOGI(_logTag, "Mode1 after freq change: %d", mode1);

				return PCA9685Error::none;
			}

			PCA9685Error setOutputInverted(const bool state) const {
				return updateRegisterBit(REG_MODE2, REG_MODE2_INVRT, state);
			}

			PCA9685Error setAutoIncrement(const bool state) const {
				return updateRegisterBit(REG_MODE1, REG_MODE1_AI, state);
			}

			PCA9685Error setOutputChangeMode(const PCA9685OutputChangeMode mode) const {
				return updateRegisterBit(REG_MODE2, REG_MODE2_OCH, mode == PCA9685OutputChangeMode::acknowledgment);
			}

			PCA9685Error setOutputDriverMode(const PCA9685OutputDriverMode mode) const {
				return updateRegisterBit(REG_MODE2, REG_MODE2_OUTDRV, mode == PCA9685OutputDriverMode::totemPole);
			}

			PCA9685Error setOutputDisabledMode(const PCA9685OutputDisabledMode mode) const {
				uint8_t value = 0;

				const auto error = readRegister(REG_MODE2, value);

				if (error != PCA9685Error::none)
					return error;

				uint8_t bits;

				switch (mode) {
					case PCA9685OutputDisabledMode::low:
						bits = 0b00;
						break;
					case PCA9685OutputDisabledMode::high:
						bits = 0b01;
						break;
					// Floating
					default:
						bits = 0b11;
						break;
				}

				return writeRegister(REG_MODE2, (value & 0b1111'1100) | bits);
			}

			PCA9685Error setDuty(const uint8_t channel, const uint16_t onDuty, const uint16_t offDuty) const {
				if (channel > 15) {
					ESP_LOGE(LOG_TAG, "channel %d is out of range [0; 15]", channel);
					return PCA9685Error::invalidArgument;
				}
				else if (!checkChannelValue(onDuty) || !checkChannelValue(offDuty)) {
					return PCA9685Error::invalidArgument;
				}

				// uint8_t mode1 = 0;
				// uint8_t mode2 = 0;
				// readRegister(REG_MODE1, mode1);
				// readRegister(REG_MODE2, mode2);
				// ESP_LOGI(_logTag, "mode1: %d, mode2: %d", mode1, mode2);

				constexpr static uint8_t bufferSize = 1 + 4;

				uint8_t buffer[bufferSize] {
					static_cast<uint8_t>(REG_LED0 + channel * 4)
				};

				*reinterpret_cast<uint16_t*>(buffer + 1) = onDuty;
				*reinterpret_cast<uint16_t*>(buffer + 3) = offDuty;

				return write({ buffer, bufferSize });
			}

			PCA9685Error setDuty(const uint8_t channel, const uint16_t duty) const {
				return setDuty(channel, 0, duty);
			}

			// Warning: can be used only with PCA9685OutputChangeMode::stop
			// Otherwise visual blinking may occur
			PCA9685Error setDuties(const uint16_t duty) const {
				constexpr static uint8_t bufferSize = 1 + 4;

				uint8_t buffer[bufferSize] {
					REG_ALL_LED
				};

				*reinterpret_cast<uint32_t*>(buffer + 1) = duty << 16;

				return write({ buffer, bufferSize });
			}

			template<uint8_t fromChannel, uint8_t channelCount>
			PCA9685Error setDuties(const std::array<uint16_t, channelCount>& duties) {
				if (channelCount == 0) {
					ESP_LOGE(LOG_TAG, "channelCount should be > 0");
					return PCA9685Error::invalidArgument;
				}
				else if (fromChannel + channelCount > 16) {
					ESP_LOGE(LOG_TAG, "fromChannel + channelCount should be <= 16");
					return PCA9685Error::invalidArgument;
				}

				// Reg + 4 bytes per value * channel count
				constexpr static uint8_t bufferSize = 1 + 4 * channelCount;

				uint8_t buffer[bufferSize] {
					static_cast<uint8_t>(REG_LED0 + fromChannel * 0x04),
				};

				const auto channelPtr = reinterpret_cast<uint32_t*>(buffer + 1);

				for (uint8_t i = 0; i < channelCount; ++i) {
					if (!checkChannelValue(duties[i]))
						return PCA9685Error::invalidArgument;

					channelPtr[i] = duties[i] << 16;
				}

				return write({ buffer, bufferSize });
			}

			PCA9685Error setDuties(const std::array<uint16_t, channelCount>& duties) {
				return setDuties<0, channelCount>(duties);
			}

			static void errorToString(const PCA9685Error error, const std::span<char> str) {
				switch (error) {
					case PCA9685Error::none:
						std::strncpy(str.data(), "none", str.size());
						break;

					case PCA9685Error::I2C:
						std::strncpy(str.data(), "I2C or wiring failure", str.size());
						break;

					default:
						std::strncpy(str.data(), "invalid argument", str.size());
						break;
				}
			}

		private:
			// -------------------------------- Subtypes --------------------------------

			constexpr static auto LOG_TAG = "PCA9685";

			constexpr static uint8_t REG_MODE1 = 0x00;
			constexpr static uint8_t REG_MODE1_RESTART = 1 << 7;
			constexpr static uint8_t REG_MODE1_AI = 1 << 5;
			constexpr static uint8_t REG_MODE1_SLEEP = 1 << 4;

			constexpr static uint8_t REG_MODE2 = 0x01;
			constexpr static uint8_t REG_MODE2_INVRT = 1 << 4;
			constexpr static uint8_t REG_MODE2_OCH = 1 << 3;
			constexpr static uint8_t REG_MODE2_OUTDRV = 1 << 2;

			constexpr static uint8_t REG_LED0 = 0x06;                 // Start of LED registers, 4 bytes per each, LE
			constexpr static uint8_t REG_ALL_LED = 0xFA;              // Start of all LEDs registers, 4 bytes in total, LE
			constexpr static uint8_t REG_PRESCALE = 0xFE;

			// -------------------------------- Vars --------------------------------

			i2c_master_dev_handle_t _device {};

			PCA9685Error write(const std::span<const uint8_t> data) const {
				const auto ESPError = i2c_master_transmit(
					_device,
					data.data(),
					data.size(),
					500
				);

				if (ESPError != ESP_OK) {
					ESP_ERROR_CHECK_WITHOUT_ABORT(ESPError);

					return PCA9685Error::I2C;
				}

				return PCA9685Error::none;
			}

			PCA9685Error writeRegister(const uint8_t reg, const uint8_t value) const {
				const uint8_t data[2] {
					reg,
					value
				};

				return write({ data, 2 });
			}

			PCA9685Error readRegister(const uint8_t reg, uint8_t& value) const {
				const auto ESPError = i2c_master_transmit_receive(
					_device,
					&reg,
					1,
					&value,
					1,
					500
				);

				if (ESPError != ESP_OK) {
					ESP_ERROR_CHECK_WITHOUT_ABORT(ESPError);

					return PCA9685Error::I2C;
				}

				return PCA9685Error::none;
			}

			PCA9685Error updateRegisterBit(const uint8_t reg, const uint8_t bit, const bool state) const {
				uint8_t value = 0;

				const auto error = readRegister(reg, value);

				if (error != PCA9685Error::none)
					return error;

				return writeRegister(reg, (value & ~bit) | (state ? bit : 0));
			}

			static bool checkChannelValue(const uint16_t value) {
				if (value > 4095) {
					ESP_LOGE(LOG_TAG, "value %d is out of range [0; 4095]", value);

					return false;
				}

				return true;
			}

			static void delayUs(const uint32_t us) {
				// esp_rom_delay_us() would be better, but nah
				vTaskDelay(pdMS_TO_TICKS(std::max<uint32_t>(us / 1'000, portTICK_PERIOD_MS)));
			}

			PCA9685Error internalSetup(
				const i2c_master_bus_handle_t& I2CBus,
				const uint8_t I2CAddress,
				const uint32_t I2CFrequencyHz
			) {
				i2c_device_config_t deviceConfig {};
				deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
				deviceConfig.device_address = I2CAddress;
				deviceConfig.scl_speed_hz = I2CFrequencyHz;

				const auto error = i2c_master_bus_add_device(I2CBus, &deviceConfig, &_device);

				if (error != ESP_OK) {
					ESP_ERROR_CHECK_WITHOUT_ABORT(error);
					return PCA9685Error::I2C;
				}

				return PCA9685Error::none;
			}
	};
}