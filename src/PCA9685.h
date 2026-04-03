#pragma once

#include <algorithm>
#include <cstdint>
#include <utility>
#include <span>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <array>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gpio.h>
#include <driver/i2c_master.h>

#include <esp_timer.h>

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
		openDrain,
		totemPole
	};

	class PCA9685 {
		public:
			constexpr static uint8_t baseI2CAddress = 0x40;
			constexpr static uint8_t channelCount = 16;

			PCA9685Error setup(
				const i2c_master_bus_handle_t& bus,
				const uint8_t address = baseI2CAddress,
				const uint32_t clockSpeedHz = 400'000
			) {
				// -------------------------------- Device --------------------------------

				i2c_device_config_t deviceConfig {};
				deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
				deviceConfig.device_address = address;
				deviceConfig.scl_speed_hz = clockSpeedHz;

				const auto state = i2c_master_bus_add_device(bus, &deviceConfig, &_device);

				if (state != ESP_OK) {
					ESP_ERROR_CHECK_WITHOUT_ABORT(state);
					return PCA9685Error::I2C;
				}

				return PCA9685Error::none;
			}


			PCA9685Error restart() const {
				const auto error = writeRegister(REG_MODE1, REG_MODE1_RESTART);

				if (error != PCA9685Error::none)
					return error;

				delayUs(500);

				return PCA9685Error::none;
			}

			PCA9685Error setFrequency(const uint32_t frequencyHz) const {
				if (frequencyHz < 24 || frequencyHz > 1526) {
					ESP_LOGE(_logTag, "frequency %d is out of range [24; 1526] Hz", frequencyHz);
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

				// Prescale should be updated only in sleep mode, so...
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

			PCA9685Error setAutoIncrement(const bool state) const {
				return updateRegisterBit(REG_MODE1, REG_MODE1_AI, state);
			}

			PCA9685Error setOutputChangeMode(const PCA9685OutputChangeMode mode) const {
				return updateRegisterBit(REG_MODE2, REG_MODE2_OCH, mode == PCA9685OutputChangeMode::acknowledgment);
			}

			PCA9685Error setOutputDriverMode(const PCA9685OutputDriverMode mode) const {
				return updateRegisterBit(REG_MODE2, REG_MODE2_OUTDRV, mode == PCA9685OutputDriverMode::totemPole);
			}

			PCA9685Error setChannelValue(const uint8_t channel, const uint16_t on, const uint16_t off) const {
				if (channel > 15) {
					ESP_LOGE(_logTag, "channel %d is out of range [0; 15]", channel);
					return PCA9685Error::invalidArgument;
				}
				else if (!checkChannelValue(on)) {
					return PCA9685Error::invalidArgument;
				}
				else if (!checkChannelValue(off)) {
					return PCA9685Error::invalidArgument;
				}

				// uint8_t mode1 = 0;
				// uint8_t mode2 = 0;
				// readRegister(REG_MODE1, mode1);
				// readRegister(REG_MODE2, mode2);
				// ESP_LOGI(_logTag, "mode1: %d, mode2: %d", mode1, mode2);

				uint8_t data[5] {
					// Reg
					static_cast<uint8_t>(REG_LED0 + channel * 4),

					// On low/high
					static_cast<uint8_t>(on & 0xFF),
					static_cast<uint8_t>(on >> 8),

					// Off low/high
					static_cast<uint8_t>(off & 0xFF),
					static_cast<uint8_t>(off >> 8),
				};

				return write({ data, 5 });
			}

			PCA9685Error setChannelDuty(const uint8_t channel, const uint16_t duty) const {
				return setChannelValue(channel, 0, duty);
			}

			// Warning: should be used only with PCA9685OutputChangeMode::stop
			// Otherwise visual blinking may occur
			PCA9685Error setChannelDuties(const uint16_t duty) const {
				constexpr static uint8_t bufferSize = 1 + 4;

				uint8_t buffer[bufferSize] {
					REG_ALL_LED
				};

				*reinterpret_cast<uint32_t*>(buffer + 1) = duty << 16;

				return write({ buffer, bufferSize });
			}

			template<uint8_t fromChannel, uint8_t channelCount>
			PCA9685Error setChannelDuties(const std::array<uint16_t, channelCount>& duties) {
				if (channelCount == 0) {
					ESP_LOGE(_logTag, "channelCount should be > 0");
					return PCA9685Error::invalidArgument;
				}
				else if (fromChannel + channelCount > 15) {
					ESP_LOGE(_logTag, "fromChannel + channelCount should be <= 15");
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

			PCA9685Error setChannelDuties(const std::array<uint16_t, channelCount>& duties) {
				return setChannelDuties<0, channelCount>(duties);
			}

			static void errorToString(const PCA9685Error error, const std::span<char> str) {
				switch (error) {
					case PCA9685Error::none:
						std::strncpy(str.data(), "none", str.size());
						break;

					case PCA9685Error::I2C:
						std::strncpy(str.data(), "I2C", str.size());
						break;

					default:
						std::strncpy(str.data(), "invalid argument", str.size());
						break;
				}
			}

		private:
			// -------------------------------- Subtypes --------------------------------

			constexpr static auto _logTag = "PCA9685";

			constexpr static uint8_t REG_MODE1 = 0x00;
			constexpr static uint8_t REG_MODE1_RESTART = 1 << 7;
			constexpr static uint8_t REG_MODE1_EXTCLK = 1 << 6;
			constexpr static uint8_t REG_MODE1_AI = 1 << 5;
			constexpr static uint8_t REG_MODE1_SLEEP = 1 << 4;

			constexpr static uint8_t REG_MODE2 = 0x01;
			constexpr static uint8_t REG_MODE2_INVRT = 1 << 4;
			constexpr static uint8_t REG_MODE2_OCH = 1 << 3;
			constexpr static uint8_t REG_MODE2_OUTDRV = 1 << 2;

			constexpr static uint8_t REG_SUBADR1 = 0x02;
			constexpr static uint8_t REG_SUBADR2 = 0x03;
			constexpr static uint8_t REG_SUBADR3 = 0x04;
			constexpr static uint8_t REG_ALLCALLADR = 0x05;
			constexpr static uint8_t REG_LED0 = 0x06;                 // Start of LED registers, 4 bytes per each, LE
			constexpr static uint8_t REG_ALL_LED = 0xFA;              // Start of all LEDs registers, 4 bytes in total, LE
			constexpr static uint8_t REG_PRESCALE = 0xFE;

			// -------------------------------- Vars --------------------------------

			i2c_master_dev_handle_t _device {};

			PCA9685Error write(const std::span<const uint8_t> data) const {
				const auto state = i2c_master_transmit(_device, data.data(), data.size(), 1'000);
				ESP_ERROR_CHECK_WITHOUT_ABORT(state);

				return state == ESP_OK ? PCA9685Error::none : PCA9685Error::I2C;
			}

			PCA9685Error writeRegister(const uint8_t reg, const uint8_t value) const {
				const uint8_t data[2] {
					reg,
					value
				};

				return write({ data, 2 });
			}

			PCA9685Error updateRegisterBit(const uint8_t reg, const uint8_t bit, const bool state) const {
				// Reading current value
				uint8_t value = 0;

				const auto error = readRegister(reg, value);

				if (error != PCA9685Error::none)
					return error;

				// Prescale should be updated only in sleep mode, so...
				return writeRegister(reg, (value & ~bit) | (state ? bit : 0));
			}

			PCA9685Error readRegister(const uint8_t reg, uint8_t& value) const {
				const auto state = i2c_master_transmit_receive(
					_device,

					&reg,
					1,

					&value,
					1,

					1'000
				);

				ESP_ERROR_CHECK_WITHOUT_ABORT(state);

				return state == ESP_OK ? PCA9685Error::none : PCA9685Error::I2C;
			}

			static bool checkChannelValue(const uint16_t value) {
				if (value > 4095) {
					ESP_LOGE(_logTag, "value %d is out of range [0; 4095]", value);
					return false;
				}

				return true;
			}

			static void delayUs(const uint32_t us) {
				// esp_rom_delay_us() would be better, but nah
				vTaskDelay(pdMS_TO_TICKS(std::max<uint32_t>(us / 1'000, portTICK_PERIOD_MS)));
			}
	};
}