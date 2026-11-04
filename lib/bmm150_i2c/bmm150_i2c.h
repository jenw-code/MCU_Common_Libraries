#pragma once

#include <Arduino.h>
#include "i2c_master.h"

/// @brief Class for the Bosch BMM150 geomagnetic sensor
class BMM150_I2C final : public I2C_Master
{
public:
	enum class I2CAddress : uint8_t
	{
		Addr_0x10 = 0b10000, // CSB = 0, SDO = 0
		Addr_0x11 = 0b10001, // CSB = 0, SDO = 1
		Addr_0x12 = 0b10010, // CSB = 1, SDO = 0
		Addr_0x13 = 0b10011	 // CSB = 1, SDO = 1
	};

	enum class PowerMode : uint8_t
	{
		Off = 0,
		Suspend = 1,
		Sleep = 2,
		Active = 3 // Periodic measurements at the output data rate (ODR)
	};

private:
	static constexpr uint8_t IC_ID_VALUE = 0x32; // Value returned from the BMM150 when reading its chip ID

	static constexpr uint8_t REG_CHIP_ID = 0x40;	   // Chip ID register (reads 0x00 while in suspend mode)
	static constexpr uint8_t REG_POWER_CONTROL = 0x4B; // Power control and soft reset register (only register accessible in suspend mode)
	static constexpr uint8_t REG_OP_MODE = 0x4C;	   // Operation mode, data rate and self test register

	// Power control bits
	static constexpr uint8_t POWER_CONTROL_BIT_POWER_CONTROL = (1 << 0); // Power Control: 1 = Sleep mode, 0 = Suspend mode
	static constexpr uint8_t POWER_CONTROL_BIT_SOFT_RESET_1 = (1 << 1);	 // Soft Reset '1': 1 = Soft reset (must be set with bit 7), auto-cleared on completion
	static constexpr uint8_t POWER_CONTROL_BIT_SPI3EN = (1 << 2);		 // SPI3en: 1 = SPI 3-wire mode, 0 = SPI 4-wire mode
	static constexpr uint8_t POWER_CONTROL_BIT_FIXED_3 = (1 << 3);		 // Fixed to 0
	static constexpr uint8_t POWER_CONTROL_BIT_FIXED_4 = (1 << 4);		 // Fixed to 0
	static constexpr uint8_t POWER_CONTROL_BIT_FIXED_5 = (1 << 5);		 // Fixed to 0
	static constexpr uint8_t POWER_CONTROL_BIT_FIXED_6 = (1 << 6);		 // Fixed to 0
	static constexpr uint8_t POWER_CONTROL_BIT_SOFT_RESET_7 = (1 << 7);	 // Soft Reset '1': 1 = Soft reset (must be set with bit 1), auto-cleared on completion

	// Operation mode bits
	static constexpr uint8_t OP_MODE_BIT_SELF_TEST = (1 << 0);	 // Self Test: 1 = Normal self-test enabled, 0 = Disabled
	static constexpr uint8_t OP_MODE_BIT_OPMODE_0 = (1 << 1);	 // Opmode <0>: Operation mode control bit 0
	static constexpr uint8_t OP_MODE_BIT_OPMODE_1 = (1 << 2);	 // Opmode <1>: Operation mode control bit 1
	static constexpr uint8_t OP_MODE_BIT_DATA_RATE_0 = (1 << 3); // Data rate <0>: Data rate control bit 0
	static constexpr uint8_t OP_MODE_BIT_DATA_RATE_1 = (1 << 4); // Data rate <1>: Data rate control bit 1
	static constexpr uint8_t OP_MODE_BIT_DATA_RATE_2 = (1 << 5); // Data rate <2>: Data rate control bit 2
	static constexpr uint8_t OP_MODE_BIT_ADV_ST_0 = (1 << 6);	 // Adv. ST <0>: Advanced self-test control bit 0
	static constexpr uint8_t OP_MODE_BIT_ADV_ST_1 = (1 << 7);	 // Adv. ST <1>: Advanced self-test control bit 1

	// Opmode values
	static constexpr uint8_t OP_MODE_OPMODE_BITS_MASK = OP_MODE_BIT_OPMODE_1 | OP_MODE_BIT_OPMODE_0;
	static constexpr uint8_t OP_MODE_OPMODE_NORMAL = 0;											 // 00: Normal mode
	static constexpr uint8_t OP_MODE_OPMODE_FORCED = OP_MODE_BIT_OPMODE_0;						 // 01: Forced mode
																								 // 10: Reserved, do not use
	static constexpr uint8_t OP_MODE_OPMODE_SLEEP = OP_MODE_BIT_OPMODE_1 | OP_MODE_BIT_OPMODE_0; // 11: Sleep mode

	static constexpr uint8_t START_UP_TIME_MILLIS = 3; // Time from suspend to sleep mode after setting power control bit

	/// @brief Return 8 bit chip ID
	uint8_t getChipID() const
	{
		return readReg(REG_CHIP_ID);
	}

protected:
	/// @brief Initialise BMM150_I2C
	/// @return True if successful
	bool startWire() override;

public:
	explicit BMM150_I2C(TwoWire &i2cWire = Wire, const I2CAddress i2cAddr = I2CAddress::Addr_0x10);

	/// @brief Set power mode
	/// @return True if successful
	template <PowerMode MODE>
	bool setPowerMode()
	{
		static_assert(MODE != PowerMode::Off, "Power off mode can only be entered by removing VDD / VDDIO");

		uint8_t value = 0;
		if (!readReg(REG_POWER_CONTROL, &value, 1))
			return false;

		if constexpr (MODE == PowerMode::Suspend)
		{
			value &= ~POWER_CONTROL_BIT_POWER_CONTROL; // Power Control Bit - set to 0 (suspend mode)
			return writeReg(REG_POWER_CONTROL, &value);
		}
		else
		{
			// Opmode register is only accessible once out of suspend mode
			if ((value & POWER_CONTROL_BIT_POWER_CONTROL) == 0)
			{
				value |= POWER_CONTROL_BIT_POWER_CONTROL; // Power Control Bit - set to 1 (suspend -> sleep mode)
				if (!writeReg(REG_POWER_CONTROL, &value))
					return false;

				delay(START_UP_TIME_MILLIS);
			}

			if (!readReg(REG_OP_MODE, &value, 1))
				return false;

			value &= ~OP_MODE_OPMODE_BITS_MASK; // Opmode - clear bits

			if constexpr (MODE == PowerMode::Sleep)
				value |= OP_MODE_OPMODE_SLEEP;
			else if constexpr (MODE == PowerMode::Active)
				value |= OP_MODE_OPMODE_NORMAL;

			return writeReg(REG_OP_MODE, &value);
		}
	}

	/// @brief Trigger a single measurement (forced mode). The device returns to sleep mode once complete
	/// @return True if triggered. False if suspended, in normal mode, or a measurement is still in progress
	bool forceMeasurement();
};
