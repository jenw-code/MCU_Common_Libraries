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

	enum class DataOutputRate : uint8_t
	{
		Rate_2 = 2,
		Rate_6 = 6,
		Rate_8 = 8,
		Rate_10 = 10, // Default
		Rate_15 = 15,
		Rate_20 = 20,
		Rate_25 = 25,
		Rate_30 = 30
	};

	enum class MeasurementPreset : uint8_t
	{
		LowPower = 0,		 // Low power preset - nXY = 3, nZ = 3, ODR = 10 Hz
		Regular = 1,		 // Regular preset - nXY = 9, nZ = 15, ODR = 10 Hz
		EnhancedRegular = 2, // Enhanced regular preset - nXY = 15, nZ = 27, ODR = 10 Hz
		HighAccuracy = 3	 // High accuracy preset - nXY = 47, nZ = 83, ODR = 20 Hz
	};

	enum class InterruptAxes : uint8_t
	{
		None = 0, // Interrupt disabled
		X = 1,
		Y = 2,
		Z = 3,
		XY = 4,
		XZ = 5,
		YZ = 6,
		XYZ = 7
	};

	struct InterruptStatus final
	{
		bool lowX = false;		  // Low Int X: X-axis below Low-Threshold
		bool lowY = false;		  // Low Int Y: Y-axis below Low-Threshold
		bool lowZ = false;		  // Low Int Z: Z-axis below Low-Threshold
		bool highX = false;		  // High Int X: X-axis above High-Threshold
		bool highY = false;		  // High Int Y: Y-axis above High-Threshold
		bool highZ = false;		  // High Int Z: Z-axis above High-Threshold
		bool overflow = false;	  // Overflow: Measurement overflow
		bool dataOverrun = false; // Data Overrun: Data registers updated before previous data read
	};

	static constexpr int16_t DATA_XY_OVERFLOW = -4096; // X/Y data register value when the axis overflowed
	static constexpr int16_t DATA_Z_OVERFLOW = -16384; // Z data register value when the axis overflowed

private:
	static constexpr uint8_t IC_ID_VALUE = 0x32; // Value returned from the BMM150 when reading its chip ID

	static constexpr uint8_t REG_CHIP_ID = 0x40;			// Chip ID register (reads 0x00 while in suspend mode)
	static constexpr uint8_t REG_DATA_X_LSB = 0x42;			// DATA X [4:0] LSB and X-Self-Test
	static constexpr uint8_t REG_DATA_X_MSB = 0x43;			// DATA X [12:5] MSB
	static constexpr uint8_t REG_DATA_Y_LSB = 0x44;			// DATA Y [4:0] LSB and Y-Self-Test
	static constexpr uint8_t REG_DATA_Y_MSB = 0x45;			// DATA Y [12:5] MSB
	static constexpr uint8_t REG_DATA_Z_LSB = 0x46;			// DATA Z [6:0] LSB and Z-Self-Test
	static constexpr uint8_t REG_DATA_Z_MSB = 0x47;			// DATA Z [14:7] MSB
	static constexpr uint8_t REG_RHALL_LSB = 0x48;			// RHALL [5:0] LSB and Data Ready Status
	static constexpr uint8_t REG_RHALL_MSB = 0x49;			// RHALL [13:6] MSB
	static constexpr uint8_t REG_INTERRUPT_STATUS = 0x4A;	// Interrupt status register (cleared when read, for latched interrupts)
	static constexpr uint8_t REG_POWER_CONTROL = 0x4B;		// Power control and soft reset register (only register accessible in suspend mode)
	static constexpr uint8_t REG_OP_MODE = 0x4C;			// Operation mode, data rate and self test register
	static constexpr uint8_t REG_INTERRUPT_SETTINGS = 0x4D; // Interrupt settings register
	static constexpr uint8_t REG_INTERRUPT_AXES = 0x4E;		// Interrupt settings and axes enable register
	static constexpr uint8_t REG_LOW_THRESHOLD = 0x4F;		// Low-Threshold interrupt threshold setting
	static constexpr uint8_t REG_HIGH_THRESHOLD = 0x50;		// High-Threshold interrupt threshold setting
	static constexpr uint8_t REG_REPXY = 0x51;				// Number of repetitions for x/y-axis (nXY = 1 + 2 x REPXY)
	static constexpr uint8_t REG_REPZ = 0x52;				// Number of repetitions for z-axis (nZ = 1 + REPZ)

	// Data X LSB bits
	static constexpr uint8_t DATA_X_LSB_DATA_X_SHIFT = 3;									// DATA X [4:0] starts at bit 3
	static constexpr uint8_t DATA_X_LSB_BIT_X_SELF_TEST = (1 << 0);							// X-Self-Test: Normal self-test result for x-axis
	static constexpr uint8_t DATA_X_LSB_BIT_FIXED_1 = (1 << 1);								// Fixed to 0
	static constexpr uint8_t DATA_X_LSB_BIT_FIXED_2 = (1 << 2);								// Fixed to 0
	static constexpr uint8_t DATA_X_LSB_BITS_DATA_X = (0b11111 << DATA_X_LSB_DATA_X_SHIFT); // DATA X [4:0]

	// Data Y LSB bits
	static constexpr uint8_t DATA_Y_LSB_DATA_Y_SHIFT = 3;									// DATA Y [4:0] starts at bit 3
	static constexpr uint8_t DATA_Y_LSB_BIT_Y_SELF_TEST = (1 << 0);							// Y-Self-Test: Normal self-test result for y-axis
	static constexpr uint8_t DATA_Y_LSB_BIT_FIXED_1 = (1 << 1);								// Fixed to 0
	static constexpr uint8_t DATA_Y_LSB_BIT_FIXED_2 = (1 << 2);								// Fixed to 0
	static constexpr uint8_t DATA_Y_LSB_BITS_DATA_Y = (0b11111 << DATA_Y_LSB_DATA_Y_SHIFT); // DATA Y [4:0]

	// Data Z LSB bits
	static constexpr uint8_t DATA_Z_LSB_DATA_Z_SHIFT = 1;									  // DATA Z [6:0] starts at bit 1
	static constexpr uint8_t DATA_Z_LSB_BIT_Z_SELF_TEST = (1 << 0);							  // Z-Self-Test: Normal self-test result for z-axis
	static constexpr uint8_t DATA_Z_LSB_BITS_DATA_Z = (0b1111111 << DATA_Z_LSB_DATA_Z_SHIFT); // DATA Z [6:0]

	// RHALL LSB bits
	static constexpr uint8_t RHALL_LSB_RHALL_SHIFT = 2;									 // RHALL [5:0] starts at bit 2
	static constexpr uint8_t RHALL_LSB_BIT_DATA_READY_STATUS = (1 << 0);				 // Data Ready Status: 1 = Data registers updated and not yet read, 0 = Read (cleared on data register read)
	static constexpr uint8_t RHALL_LSB_BIT_FIXED_1 = (1 << 1);							 // Fixed to 0
	static constexpr uint8_t RHALL_LSB_BITS_RHALL = (0b111111 << RHALL_LSB_RHALL_SHIFT); // RHALL [5:0]

	// Interrupt status bits
	static constexpr uint8_t INTERRUPT_STATUS_BIT_LOW_INT_X = (1 << 0);	   // Low Int X: 1 = X-axis below Low-Threshold
	static constexpr uint8_t INTERRUPT_STATUS_BIT_LOW_INT_Y = (1 << 1);	   // Low Int Y: 1 = Y-axis below Low-Threshold
	static constexpr uint8_t INTERRUPT_STATUS_BIT_LOW_INT_Z = (1 << 2);	   // Low Int Z: 1 = Z-axis below Low-Threshold
	static constexpr uint8_t INTERRUPT_STATUS_BIT_HIGH_INT_X = (1 << 3);   // High Int X: 1 = X-axis above High-Threshold
	static constexpr uint8_t INTERRUPT_STATUS_BIT_HIGH_INT_Y = (1 << 4);   // High Int Y: 1 = Y-axis above High-Threshold
	static constexpr uint8_t INTERRUPT_STATUS_BIT_HIGH_INT_Z = (1 << 5);   // High Int Z: 1 = Z-axis above High-Threshold
	static constexpr uint8_t INTERRUPT_STATUS_BIT_OVERFLOW = (1 << 6);	   // Overflow: 1 = Measurement overflow
	static constexpr uint8_t INTERRUPT_STATUS_BIT_DATA_OVERRUN = (1 << 7); // Data Overrun: 1 = Data registers updated before previous data read

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

	// Data rate values
	static constexpr uint8_t OP_MODE_DATA_RATE_BITS_MASK = OP_MODE_BIT_DATA_RATE_2 | OP_MODE_BIT_DATA_RATE_1 | OP_MODE_BIT_DATA_RATE_0;
	static constexpr uint8_t OP_MODE_DATA_RATE_10 = 0;																			 // 000: 10 Hz (default)
	static constexpr uint8_t OP_MODE_DATA_RATE_2 = OP_MODE_BIT_DATA_RATE_0;														 // 001: 2 Hz
	static constexpr uint8_t OP_MODE_DATA_RATE_6 = OP_MODE_BIT_DATA_RATE_1;														 // 010: 6 Hz
	static constexpr uint8_t OP_MODE_DATA_RATE_8 = OP_MODE_BIT_DATA_RATE_1 | OP_MODE_BIT_DATA_RATE_0;							 // 011: 8 Hz
	static constexpr uint8_t OP_MODE_DATA_RATE_15 = OP_MODE_BIT_DATA_RATE_2;													 // 100: 15 Hz
	static constexpr uint8_t OP_MODE_DATA_RATE_20 = OP_MODE_BIT_DATA_RATE_2 | OP_MODE_BIT_DATA_RATE_0;							 // 101: 20 Hz
	static constexpr uint8_t OP_MODE_DATA_RATE_25 = OP_MODE_BIT_DATA_RATE_2 | OP_MODE_BIT_DATA_RATE_1;							 // 110: 25 Hz
	static constexpr uint8_t OP_MODE_DATA_RATE_30 = OP_MODE_BIT_DATA_RATE_2 | OP_MODE_BIT_DATA_RATE_1 | OP_MODE_BIT_DATA_RATE_0; // 111: 30 Hz

	// Interrupt settings bits
	static constexpr uint8_t INTERRUPT_SETTINGS_BIT_LOW_INT_X_EN = (1 << 0);	// Low Int X En: 0 = Enabled, 1 = Disabled (default)
	static constexpr uint8_t INTERRUPT_SETTINGS_BIT_LOW_INT_Y_EN = (1 << 1);	// Low Int Y En: 0 = Enabled, 1 = Disabled (default)
	static constexpr uint8_t INTERRUPT_SETTINGS_BIT_LOW_INT_Z_EN = (1 << 2);	// Low Int Z En: 0 = Enabled, 1 = Disabled (default)
	static constexpr uint8_t INTERRUPT_SETTINGS_BIT_HIGH_INT_X_EN = (1 << 3);	// High Int X En: 0 = Enabled, 1 = Disabled (default)
	static constexpr uint8_t INTERRUPT_SETTINGS_BIT_HIGH_INT_Y_EN = (1 << 4);	// High Int Y En: 0 = Enabled, 1 = Disabled (default)
	static constexpr uint8_t INTERRUPT_SETTINGS_BIT_HIGH_INT_Z_EN = (1 << 5);	// High Int Z En: 0 = Enabled, 1 = Disabled (default)
	static constexpr uint8_t INTERRUPT_SETTINGS_BIT_OVERFLOW_INT_EN = (1 << 6); // Overflow Int En: 1 = Overflow mapped to INT pin, 0 = Disabled (default)
	static constexpr uint8_t INTERRUPT_SETTINGS_BIT_DATA_OVERRUN_EN = (1 << 7); // Data Overrun En: 1 = Data overrun indication enabled, 0 = Disabled (default)

	// Interrupt and axes bits
	static constexpr uint8_t INTERRUPT_AXES_BIT_INTERRUPT_POLARITY = (1 << 0); // Interrupt Polarity: 1 = INT active high (default), 0 = Active low
	static constexpr uint8_t INTERRUPT_AXES_BIT_INTERRUPT_LATCH = (1 << 1);	   // Interrupt Latch: 1 = Latched until 0x4A read (default), 0 = Non-latched
	static constexpr uint8_t INTERRUPT_AXES_BIT_DR_POLARITY = (1 << 2);		   // DR Polarity: 1 = DRDY active high (default), 0 = Active low
	static constexpr uint8_t INTERRUPT_AXES_BIT_CHANNEL_X = (1 << 3);		   // Channel X: 0 = X-axis enabled (default), 1 = Disabled
	static constexpr uint8_t INTERRUPT_AXES_BIT_CHANNEL_Y = (1 << 4);		   // Channel Y: 0 = Y-axis enabled (default), 1 = Disabled
	static constexpr uint8_t INTERRUPT_AXES_BIT_CHANNEL_Z = (1 << 5);		   // Channel Z: 0 = Z-axis and resistance enabled (default), 1 = Disabled
	static constexpr uint8_t INTERRUPT_AXES_BIT_INTERRUPT_PIN_EN = (1 << 6);   // Interrupt Pin En: 1 = Interrupt status mapped to INT pin, 0 = Disabled (default)
	static constexpr uint8_t INTERRUPT_AXES_BIT_DATA_READY_PIN_EN = (1 << 7);  // Data Ready Pin En: 1 = Data ready mapped to DRDY pin, 0 = Disabled (default)

	static constexpr uint8_t START_UP_TIME_MILLIS = 3; // Time from suspend to sleep mode after setting power control bit

	/// @brief Return 8 bit chip ID
	uint8_t getChipID() const
	{
		return readReg(REG_CHIP_ID);
	}

	/// @brief Return true if the interrupt axes include the x-axis
	static constexpr bool interruptAxesIncludeX(const InterruptAxes axes)
	{
		return (axes == InterruptAxes::X) || (axes == InterruptAxes::XY) || (axes == InterruptAxes::XZ) || (axes == InterruptAxes::XYZ);
	}

	/// @brief Return true if the interrupt axes include the y-axis
	static constexpr bool interruptAxesIncludeY(const InterruptAxes axes)
	{
		return (axes == InterruptAxes::Y) || (axes == InterruptAxes::XY) || (axes == InterruptAxes::YZ) || (axes == InterruptAxes::XYZ);
	}

	/// @brief Return true if the interrupt axes include the z-axis
	static constexpr bool interruptAxesIncludeZ(const InterruptAxes axes)
	{
		return (axes == InterruptAxes::Z) || (axes == InterruptAxes::XZ) || (axes == InterruptAxes::YZ) || (axes == InterruptAxes::XYZ);
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

	/// @brief Set output data rate (used in normal mode)
	/// @return True if successful
	template <DataOutputRate RATE>
	bool setDataRate()
	{
		uint8_t value = 0;
		if (!readReg(REG_OP_MODE, &value, 1))
			return false;

		value &= ~OP_MODE_DATA_RATE_BITS_MASK; // Data rate - clear bits

		if constexpr (RATE == DataOutputRate::Rate_2)
			value |= OP_MODE_DATA_RATE_2;
		else if constexpr (RATE == DataOutputRate::Rate_6)
			value |= OP_MODE_DATA_RATE_6;
		else if constexpr (RATE == DataOutputRate::Rate_8)
			value |= OP_MODE_DATA_RATE_8;
		else if constexpr (RATE == DataOutputRate::Rate_10)
			value |= OP_MODE_DATA_RATE_10;
		else if constexpr (RATE == DataOutputRate::Rate_15)
			value |= OP_MODE_DATA_RATE_15;
		else if constexpr (RATE == DataOutputRate::Rate_20)
			value |= OP_MODE_DATA_RATE_20;
		else if constexpr (RATE == DataOutputRate::Rate_25)
			value |= OP_MODE_DATA_RATE_25;
		else if constexpr (RATE == DataOutputRate::Rate_30)
			value |= OP_MODE_DATA_RATE_30;

		return writeReg(REG_OP_MODE, &value);
	}

	/// @brief Set number of repetitions
	/// @tparam N_XY Number of repetitions for x/y-axis (odd, 1 to 511)
	/// @tparam N_Z Number of repetitions for z-axis (1 to 256)
	/// @return True if successful
	template <uint16_t N_XY, uint16_t N_Z>
	bool setRepetitions()
	{
		static_assert((N_XY >= 1) && (N_XY <= 511), "N_XY must be in the range of 1 to 511");
		static_assert((N_XY % 2) == 1, "N_XY must be odd (nXY = 1 + 2 x REPXY)");
		static_assert((N_Z >= 1) && (N_Z <= 256), "N_Z must be in the range of 1 to 256");

		uint8_t repXY = static_cast<uint8_t>((N_XY - 1) / 2);
		uint8_t repZ = static_cast<uint8_t>(N_Z - 1);

		return writeReg(REG_REPXY, &repXY) &&
			   writeReg(REG_REPZ, &repZ);
	}

	/// @brief Set recommended preset for repetitions and output data rate
	/// @return True if successful
	template <MeasurementPreset PRESET>
	bool setMeasurementPreset()
	{
		if constexpr (PRESET == MeasurementPreset::LowPower)
			return setRepetitions<3, 3>() && setDataRate<DataOutputRate::Rate_10>();
		else if constexpr (PRESET == MeasurementPreset::Regular)
			return setRepetitions<9, 15>() && setDataRate<DataOutputRate::Rate_10>();
		else if constexpr (PRESET == MeasurementPreset::EnhancedRegular)
			return setRepetitions<15, 27>() && setDataRate<DataOutputRate::Rate_10>();
		else if constexpr (PRESET == MeasurementPreset::HighAccuracy)
			return setRepetitions<47, 83>() && setDataRate<DataOutputRate::Rate_20>();
		else
			return false;
	}

	/// @brief Set Low-Threshold interrupt (mapped to INT pin)
	/// @brief Triggered when an enabled axis' raw data is below THRESHOLD x 16 (roughly 6 µT per LSB, not temperature compensated)
	/// @tparam AXES The axes to enable for detection. None disables the interrupt
	/// @param threshold Low Threshold register value
	/// @return True if successful
	template <InterruptAxes AXES>
	bool setLowThresholdInterrupt(const uint8_t threshold)
	{
		if constexpr (AXES != InterruptAxes::None)
		{
			if (!writeReg(REG_LOW_THRESHOLD, &threshold))
				return false;
		}

		uint8_t value = 0;
		if (!readReg(REG_INTERRUPT_SETTINGS, &value, 1))
			return false;

		// Low Int X/Y/Z En - set to 1 (disabled), then set selected axes to 0 (enabled)
		value |= (INTERRUPT_SETTINGS_BIT_LOW_INT_X_EN | INTERRUPT_SETTINGS_BIT_LOW_INT_Y_EN | INTERRUPT_SETTINGS_BIT_LOW_INT_Z_EN);

		if constexpr (interruptAxesIncludeX(AXES))
			value &= ~INTERRUPT_SETTINGS_BIT_LOW_INT_X_EN;
		if constexpr (interruptAxesIncludeY(AXES))
			value &= ~INTERRUPT_SETTINGS_BIT_LOW_INT_Y_EN;
		if constexpr (interruptAxesIncludeZ(AXES))
			value &= ~INTERRUPT_SETTINGS_BIT_LOW_INT_Z_EN;

		return writeReg(REG_INTERRUPT_SETTINGS, &value);
	}

	/// @brief Set High-Threshold interrupt (mapped to INT pin)
	/// @brief Triggered when an enabled axis' raw data is above THRESHOLD x 16 (roughly 6 µT per LSB, not temperature compensated)
	/// @tparam AXES The axes to enable for detection. None disables the interrupt
	/// @param threshold High Threshold register value
	/// @return True if successful
	template <InterruptAxes AXES>
	bool setHighThresholdInterrupt(const uint8_t threshold)
	{
		if constexpr (AXES != InterruptAxes::None)
		{
			if (!writeReg(REG_HIGH_THRESHOLD, &threshold))
				return false;
		}

		uint8_t value = 0;
		if (!readReg(REG_INTERRUPT_SETTINGS, &value, 1))
			return false;

		// High Int X/Y/Z En - set to 1 (disabled), then set selected axes to 0 (enabled)
		value |= (INTERRUPT_SETTINGS_BIT_HIGH_INT_X_EN |
				  INTERRUPT_SETTINGS_BIT_HIGH_INT_Y_EN |
				  INTERRUPT_SETTINGS_BIT_HIGH_INT_Z_EN);

		if constexpr (interruptAxesIncludeX(AXES))
			value &= ~INTERRUPT_SETTINGS_BIT_HIGH_INT_X_EN;
		if constexpr (interruptAxesIncludeY(AXES))
			value &= ~INTERRUPT_SETTINGS_BIT_HIGH_INT_Y_EN;
		if constexpr (interruptAxesIncludeZ(AXES))
			value &= ~INTERRUPT_SETTINGS_BIT_HIGH_INT_Z_EN;

		return writeReg(REG_INTERRUPT_SETTINGS, &value);
	}

	/// @brief Enable / disable the Overflow interrupt (mapped to INT pin, shared for x, y and z axes)
	/// @return True if successful
	bool setOverflowInterrupt(const bool enable);

	/// @brief Enable / disable the Data Overrun indication in the interrupt status register (not mapped to a pin)
	/// @return True if successful
	bool setDataOverrun(const bool enable);

	/// @brief Configure the INT pin (Low-Threshold, High-Threshold and Overflow interrupts)
	/// @param enable True maps interrupt status to the INT pin, false leaves the pin in high-Z
	/// @param latched True = pin stays on until interrupt status is read, false = cleared when condition no longer valid
	/// @param activeHigh True = active high, false = active low
	/// @return True if successful
	bool setInterruptPin(const bool enable, const bool latched = true, const bool activeHigh = true);

	/// @brief Configure the DRDY pin (Data Ready interrupt, always non-latched)
	/// @param enable True maps data ready to the DRDY pin, false leaves the pin in high-Z
	/// @param activeHigh True = active high, false = active low
	/// @return True if successful
	bool setDataReadyPin(const bool enable, const bool activeHigh = true);

	/// @brief Read the interrupt status. Reading clears latched interrupts and the INT pin
	/// @param status Populated with the interrupt status
	/// @return True if successful
	bool getInterruptStatus(InterruptStatus &status) const;

	/// @brief Check if new data is available (for polling when the DRDY pin is not used)
	/// @brief Note: Reading clears the Data Ready Status, so read the data straight after
	/// @return True if the data registers have been updated and not yet read
	bool isDataReady() const
	{
		uint8_t value = 0;
		return readReg(REG_RHALL_LSB, &value, 1) && ((value & RHALL_LSB_BIT_DATA_READY_STATUS) != 0);
	}

	/// @brief Read raw (uncompensated) data from a single burst read, so all values are from the same measurement
	/// @param x X-axis raw data (13-bit signed). DATA_XY_OVERFLOW on overflow
	/// @param y Y-axis raw data (13-bit signed). DATA_XY_OVERFLOW on overflow
	/// @param z Z-axis raw data (15-bit signed). DATA_Z_OVERFLOW on overflow
	/// @param rhall Hall resistance (14-bit unsigned), used for temperature compensation
	/// @return True if successful
	bool readRawData(int16_t &x, int16_t &y, int16_t &z, uint16_t &rhall) const;
};
