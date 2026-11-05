#pragma once

#include <Arduino.h>
#include "i2c_master.h"

/// @brief Class for the Bosch BMM150 geomagnetic sensor
/// @brief Note: The protocol is selected in hardware. The PS pin must be connected to VDDIO for I2C
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

	static constexpr int16_t MAGNETIC_FIELD_OVERFLOW = -32768; // Compensated value when the axis overflowed or the data is invalid

	// A gravity sensor axis, with its sign, used to align the gravity sensor's orientation with the BMM150's
	enum class AxisOrientation : uint8_t
	{
		PlusX = 0,
		MinusX = 1,
		PlusY = 2,
		MinusY = 3,
		PlusZ = 4,
		MinusZ = 5
	};

private:
	static constexpr uint8_t IC_ID_VALUE = 0x32; // Value returned from the BMM150 when reading its chip ID

	static constexpr uint8_t REG_CHIP_ID = 0x40;			// Chip ID register (reads 0x00 while in suspend mode)
	static constexpr uint8_t REG_DATA_X_LSB = 0x42;			// DATA X low byte and X-Self-Test
	static constexpr uint8_t REG_DATA_X_MSB = 0x43;			// DATA X high byte
	static constexpr uint8_t REG_DATA_Y_LSB = 0x44;			// DATA Y low byte and Y-Self-Test
	static constexpr uint8_t REG_DATA_Y_MSB = 0x45;			// DATA Y high byte
	static constexpr uint8_t REG_DATA_Z_LSB = 0x46;			// DATA Z low byte and Z-Self-Test
	static constexpr uint8_t REG_DATA_Z_MSB = 0x47;			// DATA Z high byte
	static constexpr uint8_t REG_RHALL_LSB = 0x48;			// RHALL low byte and Data Ready Status
	static constexpr uint8_t REG_RHALL_MSB = 0x49;			// RHALL high byte
	static constexpr uint8_t REG_INTERRUPT_STATUS = 0x4A;	// Interrupt status register (cleared when read, for latched interrupts)
	static constexpr uint8_t REG_POWER_CONTROL = 0x4B;		// Power control and soft reset register (only register accessible in suspend mode)
	static constexpr uint8_t REG_OP_MODE = 0x4C;			// Operation mode, data rate and self test register
	static constexpr uint8_t REG_INTERRUPT_SETTINGS = 0x4D; // Interrupt settings register
	static constexpr uint8_t REG_INTERRUPT_AXES = 0x4E;		// Interrupt settings and axes enable register
	static constexpr uint8_t REG_LOW_THRESHOLD = 0x4F;		// Low-Threshold interrupt threshold setting
	static constexpr uint8_t REG_HIGH_THRESHOLD = 0x50;		// High-Threshold interrupt threshold setting
	static constexpr uint8_t REG_REPXY = 0x51;				// Number of repetitions for x/y-axis (nXY = 1 + 2 x REPXY)
	static constexpr uint8_t REG_REPZ = 0x52;				// Number of repetitions for z-axis (nZ = 1 + REPZ)

	// Trim registers (not documented in the datasheet, layout from the Bosch Sensortec BMM150 API)
	static constexpr uint8_t REG_DIG_X1 = 0x5D;		  // dig_x1 (int8)
	static constexpr uint8_t REG_DIG_Y1 = 0x5E;		  // dig_y1 (int8)
	static constexpr uint8_t REG_DIG_Z4_LSB = 0x62;	  // dig_z4 low byte (int16)
	static constexpr uint8_t REG_DIG_Z4_MSB = 0x63;	  // dig_z4 high byte
	static constexpr uint8_t REG_DIG_X2 = 0x64;		  // dig_x2 (int8)
	static constexpr uint8_t REG_DIG_Y2 = 0x65;		  // dig_y2 (int8)
	static constexpr uint8_t REG_DIG_Z2_LSB = 0x68;	  // dig_z2 low byte (int16)
	static constexpr uint8_t REG_DIG_Z2_MSB = 0x69;	  // dig_z2 high byte
	static constexpr uint8_t REG_DIG_Z1_LSB = 0x6A;	  // dig_z1 low byte (uint16)
	static constexpr uint8_t REG_DIG_Z1_MSB = 0x6B;	  // dig_z1 high byte
	static constexpr uint8_t REG_DIG_XYZ1_LSB = 0x6C; // dig_xyz1 low byte (uint16)
	static constexpr uint8_t REG_DIG_XYZ1_MSB = 0x6D; // dig_xyz1 high byte (bit 7 not part of value)
	static constexpr uint8_t REG_DIG_Z3_LSB = 0x6E;	  // dig_z3 low byte (int16)
	static constexpr uint8_t REG_DIG_Z3_MSB = 0x6F;	  // dig_z3 high byte
	static constexpr uint8_t REG_DIG_XY2 = 0x70;	  // dig_xy2 (int8)
	static constexpr uint8_t REG_DIG_XY1 = 0x71;	  // dig_xy1 (uint8)

	// Data X low byte bits
	static constexpr uint8_t DATA_X_LSB_BIT_X_SELF_TEST = (1 << 0); // X-Self-Test: Normal self-test result for x-axis
	static constexpr uint8_t DATA_X_LSB_BIT_FIXED_1 = (1 << 1);		// Fixed to 0
	static constexpr uint8_t DATA_X_LSB_BIT_FIXED_2 = (1 << 2);		// Fixed to 0
	static constexpr uint8_t DATA_X_LSB_BITS_DATA_X = 0b11111000;	// DATA X

	// Data Y low byte bits
	static constexpr uint8_t DATA_Y_LSB_BIT_Y_SELF_TEST = (1 << 0); // Y-Self-Test: Normal self-test result for y-axis
	static constexpr uint8_t DATA_Y_LSB_BIT_FIXED_1 = (1 << 1);		// Fixed to 0
	static constexpr uint8_t DATA_Y_LSB_BIT_FIXED_2 = (1 << 2);		// Fixed to 0
	static constexpr uint8_t DATA_Y_LSB_BITS_DATA_Y = 0b11111000;	// DATA Y

	// Data Z low byte bits
	static constexpr uint8_t DATA_Z_LSB_BIT_Z_SELF_TEST = (1 << 0); // Z-Self-Test: Normal self-test result for z-axis
	static constexpr uint8_t DATA_Z_LSB_BITS_DATA_Z = 0b11111110;	// DATA Z

	// RHALL low byte bits
	static constexpr uint8_t RHALL_LSB_BIT_DATA_READY_STATUS = (1 << 0); // Data Ready Status: 1 = Data registers updated and not yet read, 0 = Read (cleared on data register read)
	static constexpr uint8_t RHALL_LSB_BIT_FIXED_1 = (1 << 1);			 // Fixed to 0
	static constexpr uint8_t RHALL_LSB_BITS_RHALL = 0b11111100;			 // RHALL

	// Data field positions within the low byte registers (used to combine the high and low bytes)
	static constexpr uint8_t DATA_X_LSB_DATA_X_SHIFT = 3; // DATA X starts at bit 3
	static constexpr uint8_t DATA_Y_LSB_DATA_Y_SHIFT = 3; // DATA Y starts at bit 3
	static constexpr uint8_t DATA_Z_LSB_DATA_Z_SHIFT = 1; // DATA Z starts at bit 1
	static constexpr uint8_t RHALL_LSB_RHALL_SHIFT = 2;	  // RHALL starts at bit 2

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

	// dig_xyz1 high byte bits
	static constexpr uint8_t DIG_XYZ1_MSB_BITS_DIG_XYZ1 = 0b1111111; // dig_xyz1 (bit 7 is not used)

	static constexpr uint8_t START_UP_TIME_MILLIS = 3;			   // Time from suspend to sleep mode after setting power control bit
	static constexpr uint8_t SOFT_RESET_TIME_MILLIS = 1;		   // Time for soft reset to complete
	static constexpr uint8_t NORMAL_SELF_TEST_TIMEOUT_MILLIS = 10; // Time allowed for the normal self-test to complete

	// Compensation constants (Bosch Sensortec fixed point compensation)
	// Note: Other literal values in the compensation are undocumented fixed point scaling steps
	static constexpr uint16_t COMPENSATION_RESISTANCE_SCALE = 16384;			  // Resistance ratio scale (2^14)
	static constexpr uint32_t COMPENSATION_XY_OFFSET = 256 * 4096;				  // X/Y offset (256 at 4096 scale)
	static constexpr uint8_t COMPENSATION_XY2_GAIN_OFFSET = 160;				  // dig_x2 / dig_y2 gain offset
	static constexpr uint8_t COMPENSATION_XY1_SCALE = 8;						  // dig_x1 / dig_y1 scale
	static constexpr uint32_t COMPENSATION_Z_SCALE = 65536;						  // Z dig_z1 term scale (2^16)
	static constexpr uint16_t COMPENSATION_Z_ROUNDING = COMPENSATION_Z_SCALE / 2; // Z dig_z1 term rounding to nearest

	struct TrimData final
	{
		int8_t digX1 = 0;
		int8_t digY1 = 0;
		int8_t digX2 = 0;
		int8_t digY2 = 0;
		uint16_t digZ1 = 0;
		int16_t digZ2 = 0;
		int16_t digZ3 = 0;
		int16_t digZ4 = 0;
		uint8_t digXY1 = 0;
		int8_t digXY2 = 0;
		uint16_t digXYZ1 = 0;
	};

	TrimData _trimData;

	bool _suspended = true; // Set from the device by initialiseSuspended, kept up to date by setPowerMode

	/// @brief Read the chip ID
	/// @return Chip ID (IC_ID_VALUE for a BMM150), or 0 on failure or in suspend mode
	uint8_t readChipID() const
	{
		return readReg(REG_CHIP_ID);
	}

	/// @brief Set _suspended from the device, which may already be out of suspend mode (e.g. after an MCU-only reset)
	/// @return True if successful
	bool initialiseSuspended();

	/// @brief Read the factory trim data used for compensation into _trimData
	/// @return True if successful
	bool populateTrimData();

	/// @brief Parse raw (uncompensated) data from the data register bytes
	/// @param data Bytes read from DATA_REGISTER onwards (DATA_LENGTH bytes)
	/// @param x X-axis raw data (13-bit signed). DATA_XY_OVERFLOW on overflow
	/// @param y Y-axis raw data (13-bit signed). DATA_XY_OVERFLOW on overflow
	/// @param z Z-axis raw data (15-bit signed). DATA_Z_OVERFLOW on overflow
	/// @param rhall Hall resistance (14-bit unsigned)
	void parseRawData(const uint8_t *const data, int16_t &x, int16_t &y, int16_t &z, uint16_t &rhall) const;

	/// @brief Compensate raw x-axis data (Bosch Sensortec fixed point compensation)
	/// @param rawX X-axis raw data (13-bit signed)
	/// @param rhall Hall resistance (14-bit unsigned). 0 = not measured, so the trim reference is used
	/// @return Compensated x-axis magnetic field (µT), or MAGNETIC_FIELD_OVERFLOW
	int16_t compensateX(const int16_t rawX, const uint16_t rhall) const;

	/// @brief Compensate raw y-axis data (Bosch Sensortec fixed point compensation)
	/// @param rawY Y-axis raw data (13-bit signed)
	/// @param rhall Hall resistance (14-bit unsigned). 0 = not measured, so the trim reference is used
	/// @return Compensated y-axis magnetic field (µT), or MAGNETIC_FIELD_OVERFLOW
	int16_t compensateY(const int16_t rawY, const uint16_t rhall) const;

	/// @brief Compensate raw z-axis data (Bosch Sensortec fixed point compensation)
	/// @param rawZ Z-axis raw data (15-bit signed)
	/// @param rhall Hall resistance (14-bit unsigned). 0 = not measured, so z can't be compensated
	/// @return Compensated z-axis magnetic field (µT), or MAGNETIC_FIELD_OVERFLOW
	int16_t compensateZ(const int16_t rawZ, const uint16_t rhall) const;

	/// @brief Calculate a heading from two horizontal components
	/// @param x Component along the x-axis (any scale, as long as it matches y)
	/// @param y Component along the y-axis
	/// @param degrees Heading in whole degrees, rounded down (0 to 359): 0 along the y-axis, increasing towards the x-axis
	/// @return True if successful. False if both components are 0 (heading undefined)
	static bool calculateHeading(const float x, const float y, uint16_t &degrees);

	/// @brief Check whether an orientation uses the gravity sensor's x-axis, ignoring its sign
	/// @tparam ORIENTATION The axis orientation
	/// @return True if it uses the x-axis
	template <AxisOrientation ORIENTATION>
	static constexpr bool xAxis()
	{
		return (ORIENTATION == AxisOrientation::PlusX) ||
			   (ORIENTATION == AxisOrientation::MinusX);
	}

	/// @brief Check whether an orientation uses the gravity sensor's y-axis, ignoring its sign
	/// @tparam ORIENTATION The axis orientation
	/// @return True if it uses the y-axis
	template <AxisOrientation ORIENTATION>
	static constexpr bool yAxis()
	{
		return (ORIENTATION == AxisOrientation::PlusY) ||
			   (ORIENTATION == AxisOrientation::MinusY);
	}

	/// @brief Check whether an orientation uses the gravity sensor's z-axis, ignoring its sign
	/// @tparam ORIENTATION The axis orientation
	/// @return True if it uses the z-axis
	template <AxisOrientation ORIENTATION>
	static constexpr bool zAxis()
	{
		return (ORIENTATION == AxisOrientation::PlusZ) ||
			   (ORIENTATION == AxisOrientation::MinusZ);
	}

	/// @brief Get gravity along one of the BMM150's axes
	/// @tparam ORIENTATION The gravity sensor axis (and sign) that lines up with the BMM150 axis
	/// @param gravityX Gravity sensor x-axis reading
	/// @param gravityY Gravity sensor y-axis reading
	/// @param gravityZ Gravity sensor z-axis reading
	/// @return Gravity along the BMM150 axis
	template <AxisOrientation ORIENTATION>
	static constexpr float mapGravityAxis(const int16_t gravityX, const int16_t gravityY, const int16_t gravityZ)
	{
		float gravity = 0.0f;
		if constexpr (xAxis<ORIENTATION>())
			gravity = static_cast<float>(gravityX);
		else if constexpr (yAxis<ORIENTATION>())
			gravity = static_cast<float>(gravityY);
		else
			gravity = static_cast<float>(gravityZ);

		if constexpr ((ORIENTATION == AxisOrientation::MinusX) ||
					  (ORIENTATION == AxisOrientation::MinusY) ||
					  (ORIENTATION == AxisOrientation::MinusZ))
			gravity = -gravity;

		return gravity;
	}

	/// @brief Calculate a tilt-compensated heading, with gravity already in the BMM150's axes
	/// @param degrees Heading in degrees (0 to 359): 0 along the y-axis, increasing towards the x-axis
	/// @param gravityX Gravity along the BMM150's x-axis (any units). Positive along an axis pointing up
	/// @param gravityY Gravity along the BMM150's y-axis
	/// @param gravityZ Gravity along the BMM150's z-axis
	/// @return True if successful. False if any axis overflowed, there's no gravity, or the heading is undefined
	bool readTiltCompensatedHeading(uint16_t &degrees, const float gravityX, const float gravityY, const float gravityZ) const;

	/// @brief Check whether the interrupt axes include the x-axis
	/// @param axes The interrupt axes
	/// @return True if the x-axis is included
	static constexpr bool interruptAxesIncludeX(const InterruptAxes axes)
	{
		return (axes == InterruptAxes::X) ||
			   (axes == InterruptAxes::XY) ||
			   (axes == InterruptAxes::XZ) ||
			   (axes == InterruptAxes::XYZ);
	}

	/// @brief Check whether the interrupt axes include the y-axis
	/// @param axes The interrupt axes
	/// @return True if the y-axis is included
	static constexpr bool interruptAxesIncludeY(const InterruptAxes axes)
	{
		return (axes == InterruptAxes::Y) ||
			   (axes == InterruptAxes::XY) ||
			   (axes == InterruptAxes::YZ) ||
			   (axes == InterruptAxes::XYZ);
	}

	/// @brief Check whether the interrupt axes include the z-axis
	/// @param axes The interrupt axes
	/// @return True if the z-axis is included
	static constexpr bool interruptAxesIncludeZ(const InterruptAxes axes)
	{
		return (axes == InterruptAxes::Z) ||
			   (axes == InterruptAxes::XZ) ||
			   (axes == InterruptAxes::YZ) ||
			   (axes == InterruptAxes::XYZ);
	}

protected:
	/// @brief Initialise BMM150_I2C
	/// @return True if successful
	bool startWire() override;

public:
	// Data registers, for a host that reads them on the BMM150's behalf (e.g. the MPU-6500's auxiliary I2C master)
	static constexpr uint8_t DATA_REGISTER = REG_DATA_X_LSB;					 // First data register (DATA X low byte)
	static constexpr uint8_t DATA_LENGTH = (REG_RHALL_MSB - REG_DATA_X_LSB) + 1; // Number of data registers (DATA X low byte to RHALL high byte)

	/// @param i2cWire The I2C bus the device is connected to
	/// @param i2cAddr The device's I2C address, set by the CSB and SDO pins
	explicit BMM150_I2C(TwoWire &i2cWire = Wire, const I2CAddress i2cAddr = I2CAddress::Addr_0x10);

	/// @brief Set power mode
	/// @tparam MODE Suspend, Sleep or Active (measuring continuously at the output data rate). Off can't be set
	/// @return True if successful
	template <PowerMode MODE>
	bool setPowerMode()
	{
		static_assert(MODE != PowerMode::Off, "Power off mode can only be entered by removing VDD / VDDIO");

		if constexpr (MODE == PowerMode::Suspend)
		{
			uint8_t powerControl = 0;
			if (!readReg(REG_POWER_CONTROL, &powerControl, 1))
				return false;

			powerControl &= ~POWER_CONTROL_BIT_POWER_CONTROL; // Power Control Bit - set to 0 (suspend mode)
			if (!writeReg(REG_POWER_CONTROL, &powerControl))
				return false;

			_suspended = true;
			return true;
		}
		else
		{
			// Opmode register is only accessible once out of suspend mode
			if (_suspended)
			{
				uint8_t powerControl = 0;
				if (!readReg(REG_POWER_CONTROL, &powerControl, 1))
					return false;

				powerControl |= POWER_CONTROL_BIT_POWER_CONTROL; // Power Control Bit - set to 1 (suspend -> sleep mode)
				if (!writeReg(REG_POWER_CONTROL, &powerControl))
					return false;

				delay(START_UP_TIME_MILLIS);
				_suspended = false;
			}

			uint8_t opMode = 0;
			if (!readReg(REG_OP_MODE, &opMode, 1))
				return false;

			opMode &= ~OP_MODE_OPMODE_BITS_MASK; // Opmode - clear bits

			if constexpr (MODE == PowerMode::Sleep)
				opMode |= OP_MODE_OPMODE_SLEEP;
			else if constexpr (MODE == PowerMode::Active)
				opMode |= OP_MODE_OPMODE_NORMAL;
			else
				return false;

			return writeReg(REG_OP_MODE, &opMode);
		}
	}

	/// @brief Trigger a single measurement (forced mode). The device returns to sleep mode once complete
	/// @return True if triggered. False if suspended, in normal mode, or a measurement is still in progress
	bool forceMeasurement();

	/// @brief Soft reset. All registers except the trim and power control registers are reset to default, and the device is left in sleep mode
	/// @brief Note: Fails in suspend mode, where the device ignores a soft reset
	/// @return True if successful
	bool softReset();

	/// @brief Run the normal self-test (checks x, y and z signal paths and connectivity)
	/// @brief Note: The self-test puts the device in sleep mode, and the previous power mode is not restored.
	/// @brief It's therefore recommended to configure the device after the self-test
	/// @return True if the test completed and all axes passed
	bool normalSelfTest();

	/// @brief Enable / disable measurement channels. A disabled channel keeps its last measured value in the data registers
	/// @brief Note: Disabling z also disables the hall resistance measurement, so z is not compensated and x / y use the trim reference instead
	/// @param channelX True = x-axis measured, false = disabled
	/// @param channelY True = y-axis measured, false = disabled
	/// @param channelZ True = z-axis and hall resistance measured, false = disabled
	/// @return True if successful
	bool enableChannels(const bool channelX, const bool channelY, const bool channelZ);

	/// @brief Set output data rate (used in normal mode)
	/// @tparam RATE The output data rate (2 to 30 Hz)
	/// @return True if successful
	template <DataOutputRate RATE>
	bool setDataRate()
	{
		// Registers other than power control are not accessible in suspend mode
		if (_suspended)
			return false;

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
		else
			return false;

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

		// Registers other than power control are not accessible in suspend mode
		if (_suspended)
			return false;

		uint8_t repXY = static_cast<uint8_t>((N_XY - 1) / 2);
		uint8_t repZ = static_cast<uint8_t>(N_Z - 1);

		return writeReg(REG_REPXY, &repXY) &&
			   writeReg(REG_REPZ, &repZ);
	}

	/// @brief Set recommended preset for repetitions and output data rate
	/// @tparam PRESET The preset (see MeasurementPreset for each one's repetitions and rate)
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
	/// @brief Triggered when an enabled axis' raw data is below threshold x 16 (roughly 6 µT per LSB, not temperature compensated)
	/// @tparam AXES The axes to enable for detection. None disables the interrupt
	/// @param threshold Low Threshold register value
	/// @return True if successful
	template <InterruptAxes AXES>
	bool setLowThresholdInterrupt(const uint8_t threshold)
	{
		// Registers other than power control are not accessible in suspend mode
		if (_suspended)
			return false;

		if constexpr (AXES != InterruptAxes::None)
		{
			if (!writeReg(REG_LOW_THRESHOLD, &threshold))
				return false;
		}

		uint8_t value = 0;
		if (!readReg(REG_INTERRUPT_SETTINGS, &value, 1))
			return false;

		// Low Int X/Y/Z En - set to 1 (disabled), then set selected axes to 0 (enabled)
		value |= (INTERRUPT_SETTINGS_BIT_LOW_INT_X_EN |
				  INTERRUPT_SETTINGS_BIT_LOW_INT_Y_EN |
				  INTERRUPT_SETTINGS_BIT_LOW_INT_Z_EN);

		if constexpr (interruptAxesIncludeX(AXES))
			value &= ~INTERRUPT_SETTINGS_BIT_LOW_INT_X_EN;
		if constexpr (interruptAxesIncludeY(AXES))
			value &= ~INTERRUPT_SETTINGS_BIT_LOW_INT_Y_EN;
		if constexpr (interruptAxesIncludeZ(AXES))
			value &= ~INTERRUPT_SETTINGS_BIT_LOW_INT_Z_EN;

		return writeReg(REG_INTERRUPT_SETTINGS, &value);
	}

	/// @brief Set High-Threshold interrupt (mapped to INT pin)
	/// @brief Triggered when an enabled axis' raw data is above threshold x 16 (roughly 6 µT per LSB, not temperature compensated)
	/// @tparam AXES The axes to enable for detection. None disables the interrupt
	/// @param threshold High Threshold register value
	/// @return True if successful
	template <InterruptAxes AXES>
	bool setHighThresholdInterrupt(const uint8_t threshold)
	{
		// Registers other than power control are not accessible in suspend mode
		if (_suspended)
			return false;

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
	/// @param enable True = enabled, false = disabled
	/// @return True if successful
	bool setOverflowInterrupt(const bool enable);

	/// @brief Enable / disable the Data Overrun indication in the interrupt status register (not mapped to a pin)
	/// @param enable True = enabled, false = disabled
	/// @return True if successful
	bool enableDataOverrun(const bool enable);

	/// @brief Enable the configured Low-Threshold, High-Threshold and Overflow interrupts on the INT pin
	/// @param enable True maps interrupt status to the INT pin, false leaves the pin in high-Z
	/// @param latched True = pin stays on until interrupt status is read, false = cleared when condition no longer valid
	/// @param activeHigh True = active high, false = active low
	/// @return True if successful
	bool enableInterrupts(const bool enable, const bool latched = true, const bool activeHigh = true);

	/// @brief Enable the Data Ready interrupt on the DRDY pin (always non-latched)
	/// @param enable True maps data ready to the DRDY pin, false leaves the pin in high-Z
	/// @param activeHigh True = active high, false = active low
	/// @return True if successful
	bool enableDataReadyInterrupt(const bool enable, const bool activeHigh = true);

	/// @brief Read the interrupt status. Reading clears latched interrupts and the INT pin
	/// @param status The interrupt status
	/// @return True if successful
	bool getInterruptStatus(InterruptStatus &status) const;

	/// @brief Check if new data is available (for polling when the DRDY pin is not used)
	/// @brief Note: Reading clears the Data Ready Status, so read the data straight after
	/// @return True if the data registers have been updated and not yet read
	bool dataReady() const
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

	/// @brief Parse compensated magnetic field data from data register bytes already read (e.g. by the MPU-6500's auxiliary I2C master)
	/// @param data Bytes read from DATA_REGISTER onwards
	/// @param length Number of bytes in data (must be DATA_LENGTH)
	/// @param x X-axis magnetic field (µT). MAGNETIC_FIELD_OVERFLOW on overflow
	/// @param y Y-axis magnetic field (µT). MAGNETIC_FIELD_OVERFLOW on overflow
	/// @param z Z-axis magnetic field (µT). MAGNETIC_FIELD_OVERFLOW on overflow
	/// @return True if successful
	bool parseData(const uint8_t *const data, const uint8_t length, int16_t &x, int16_t &y, int16_t &z) const;

	/// @brief Read compensated magnetic field data (temperature and sensitivity compensated)
	/// @param x X-axis magnetic field (µT). MAGNETIC_FIELD_OVERFLOW on overflow
	/// @param y Y-axis magnetic field (µT). MAGNETIC_FIELD_OVERFLOW on overflow
	/// @param z Z-axis magnetic field (µT). MAGNETIC_FIELD_OVERFLOW on overflow
	/// @return True if successful
	bool readData(int16_t &x, int16_t &y, int16_t &z) const;

	/// @brief Calculate compass heading from the x and y magnetic field (0 degrees along the y-axis, increasing towards the x-axis)
	/// @param degrees Heading in degrees (0 to 359)
	/// @return True if successful. False if x or y overflowed, or there is no horizontal field
	bool readHeading(uint16_t &degrees) const;

	/// @brief Calculate a tilt-compensated compass heading, using gravity from another sensor (e.g. the MPU-6500's accelerometer)
	/// @brief Uses the same direction as readHeading: 0 degrees along the y-axis, increasing towards the x-axis
	/// @tparam X_ORIENTATION Gravity sensor axis (and sign) that lines up with the BMM150's x-axis
	/// @tparam Y_ORIENTATION Gravity sensor axis (and sign) that lines up with the BMM150's y-axis
	/// @tparam Z_ORIENTATION Gravity sensor axis (and sign) that lines up with the BMM150's z-axis
	/// @param degrees Heading in degrees (0 to 359)
	/// @param gravityX Gravity sensor x-axis reading, as read (any units, e.g. mg). Reads positive along an axis pointing up
	/// @param gravityY Gravity sensor y-axis reading, as read
	/// @param gravityZ Gravity sensor z-axis reading, as read
	/// @return True if successful. False if any axis overflowed, there's no gravity, or the heading is undefined
	template <AxisOrientation X_ORIENTATION, AxisOrientation Y_ORIENTATION, AxisOrientation Z_ORIENTATION>
	bool readHeading(uint16_t &degrees, const int16_t gravityX, const int16_t gravityY, const int16_t gravityZ) const
	{
		// The tilt maths needs each gravity sensor axis used exactly once. With three orientations,
		// that means every axis must be used
		static_assert(xAxis<X_ORIENTATION>() || xAxis<Y_ORIENTATION>() || xAxis<Z_ORIENTATION>(), "The gravity sensor's x-axis must be used");
		static_assert(yAxis<X_ORIENTATION>() || yAxis<Y_ORIENTATION>() || yAxis<Z_ORIENTATION>(), "The gravity sensor's y-axis must be used");
		static_assert(zAxis<X_ORIENTATION>() || zAxis<Y_ORIENTATION>() || zAxis<Z_ORIENTATION>(), "The gravity sensor's z-axis must be used");

		return readTiltCompensatedHeading(degrees,
										  mapGravityAxis<X_ORIENTATION>(gravityX, gravityY, gravityZ),
										  mapGravityAxis<Y_ORIENTATION>(gravityX, gravityY, gravityZ),
										  mapGravityAxis<Z_ORIENTATION>(gravityX, gravityY, gravityZ));
	}

	/// @brief Set up the device in one call. The defaults are the datasheet's general purpose (Regular) preset, measuring continuously
	/// @tparam POWER_MODE Power mode (default Active, measuring continuously)
	/// @tparam PRESET Measurement preset (default Regular)
	/// @return True if successful
	template <PowerMode POWER_MODE = PowerMode::Active,
			  MeasurementPreset PRESET = MeasurementPreset::Regular>
	bool quickSetup()
	{
		// The power mode is set first, as the preset registers can't be written in suspend mode
		return setPowerMode<POWER_MODE>() &&
			   setMeasurementPreset<PRESET>();
	}
};
