#include "bmm150_i2c.h"

BMM150_I2C::BMM150_I2C(TwoWire &i2cWire, const I2CAddress i2cAddr) : I2C_Master(static_cast<uint8_t>(i2cAddr), i2cWire) {}

bool BMM150_I2C::startWire()
{
	return I2C_Master::startWire() &&
		   initialiseSuspended() &&
		   setPowerMode<PowerMode::Sleep>() &&
		   (getChipID() == IC_ID_VALUE) &&
		   populateTrimData();
}

bool BMM150_I2C::initialiseSuspended()
{
	uint8_t value = 0;
	if (!readReg(REG_POWER_CONTROL, &value, 1))
		return false;

	_suspended = ((value & POWER_CONTROL_BIT_POWER_CONTROL) == 0);
	return true;
}

bool BMM150_I2C::populateTrimData()
{
	constexpr uint8_t TRIM_SIZE = (REG_DIG_XY1 - REG_DIG_X1) + 1;
	uint8_t trim[TRIM_SIZE];
	if (!readReg(REG_DIG_X1, trim, TRIM_SIZE))
		return false;

	_trimData.digX1 = static_cast<int8_t>(trim[REG_DIG_X1 - REG_DIG_X1]);
	_trimData.digY1 = static_cast<int8_t>(trim[REG_DIG_Y1 - REG_DIG_X1]);
	_trimData.digX2 = static_cast<int8_t>(trim[REG_DIG_X2 - REG_DIG_X1]);
	_trimData.digY2 = static_cast<int8_t>(trim[REG_DIG_Y2 - REG_DIG_X1]);

	_trimData.digZ1 = static_cast<uint16_t>((trim[REG_DIG_Z1_MSB - REG_DIG_X1] << 8) | trim[REG_DIG_Z1_LSB - REG_DIG_X1]);
	_trimData.digZ2 = static_cast<int16_t>((trim[REG_DIG_Z2_MSB - REG_DIG_X1] << 8) | trim[REG_DIG_Z2_LSB - REG_DIG_X1]);
	_trimData.digZ3 = static_cast<int16_t>((trim[REG_DIG_Z3_MSB - REG_DIG_X1] << 8) | trim[REG_DIG_Z3_LSB - REG_DIG_X1]);
	_trimData.digZ4 = static_cast<int16_t>((trim[REG_DIG_Z4_MSB - REG_DIG_X1] << 8) | trim[REG_DIG_Z4_LSB - REG_DIG_X1]);

	_trimData.digXY1 = trim[REG_DIG_XY1 - REG_DIG_X1];
	_trimData.digXY2 = static_cast<int8_t>(trim[REG_DIG_XY2 - REG_DIG_X1]);

	_trimData.digXYZ1 = static_cast<uint16_t>(((trim[REG_DIG_XYZ1_MSB - REG_DIG_X1] & DIG_XYZ1_MSB_BITS_DIG_XYZ1) << 8) |
											  trim[REG_DIG_XYZ1_LSB - REG_DIG_X1]);

	return true;
}

bool BMM150_I2C::forceMeasurement()
{
	// Registers other than power control are not accessible in suspend mode
	if (_suspended)
		return false;

	uint8_t value = 0;
	if (!readReg(REG_OP_MODE, &value, 1))
		return false;

	// Only trigger from sleep mode. Normal mode is already measuring, and forced mode means
	// a measurement is still in progress (re-triggering would stop it being written to the data registers)
	if ((value & OP_MODE_OPMODE_BITS_MASK) != OP_MODE_OPMODE_SLEEP)
		return false;

	value &= ~OP_MODE_OPMODE_BITS_MASK; // Opmode - clear bits
	value |= OP_MODE_OPMODE_FORCED;

	return writeReg(REG_OP_MODE, &value);
}

bool BMM150_I2C::softReset()
{
	// Soft reset is ignored in suspend mode
	if (_suspended)
		return false;

	uint8_t value = 0;
	if (!readReg(REG_POWER_CONTROL, &value, 1))
		return false;

	value |= (POWER_CONTROL_BIT_SOFT_RESET_7 | POWER_CONTROL_BIT_SOFT_RESET_1); // Soft Reset '1' - set both to 1 (trigger soft reset)
	if (!writeReg(REG_POWER_CONTROL, &value))
		return false;

	delay(SOFT_RESET_TIME_MILLIS);
	return true;
}

bool BMM150_I2C::normalSelfTest()
{
	if (!setPowerMode<PowerMode::Sleep>())
		return false;

	uint8_t value = 0;
	if (!readReg(REG_OP_MODE, &value, 1))
		return false;

	value |= OP_MODE_BIT_SELF_TEST; // Self Test - set to 1 (start normal self-test)
	if (!writeReg(REG_OP_MODE, &value))
		return false;

	// The device clears the Self Test bit once complete
	const unsigned long start = millis();
	do
	{
		delay(1);

		if (!readReg(REG_OP_MODE, &value, 1))
			return false;

		if ((millis() - start) >= NORMAL_SELF_TEST_TIMEOUT_MILLIS)
			return false; // Self-test failed to complete in time

	} while ((value & OP_MODE_BIT_SELF_TEST) != 0);

	// Self-test results are in bit 0 of the x, y and z low byte data registers
	constexpr uint8_t RESULT_SIZE = (REG_DATA_Z_LSB - REG_DATA_X_LSB) + 1;
	uint8_t result[RESULT_SIZE];
	if (!readReg(REG_DATA_X_LSB, result, RESULT_SIZE))
		return false;

	return ((result[REG_DATA_X_LSB - REG_DATA_X_LSB] & DATA_X_LSB_BIT_X_SELF_TEST) != 0) &&
		   ((result[REG_DATA_Y_LSB - REG_DATA_X_LSB] & DATA_Y_LSB_BIT_Y_SELF_TEST) != 0) &&
		   ((result[REG_DATA_Z_LSB - REG_DATA_X_LSB] & DATA_Z_LSB_BIT_Z_SELF_TEST) != 0);
}

bool BMM150_I2C::enableChannels(const bool channelX, const bool channelY, const bool channelZ)
{
	// Registers other than power control are not accessible in suspend mode
	if (_suspended)
		return false;

	uint8_t value = 0;
	if (!readReg(REG_INTERRUPT_AXES, &value, 1))
		return false;

	if (channelX)
		value &= ~INTERRUPT_AXES_BIT_CHANNEL_X; // Channel X - set to 0 (enabled)
	else
		value |= INTERRUPT_AXES_BIT_CHANNEL_X; // Channel X - set to 1 (disabled)

	if (channelY)
		value &= ~INTERRUPT_AXES_BIT_CHANNEL_Y; // Channel Y - set to 0 (enabled)
	else
		value |= INTERRUPT_AXES_BIT_CHANNEL_Y; // Channel Y - set to 1 (disabled)

	if (channelZ)
		value &= ~INTERRUPT_AXES_BIT_CHANNEL_Z; // Channel Z - set to 0 (enabled)
	else
		value |= INTERRUPT_AXES_BIT_CHANNEL_Z; // Channel Z - set to 1 (disabled)

	return writeReg(REG_INTERRUPT_AXES, &value);
}

bool BMM150_I2C::setOverflowInterrupt(const bool enable)
{
	// Registers other than power control are not accessible in suspend mode
	if (_suspended)
		return false;

	uint8_t value = 0;
	if (!readReg(REG_INTERRUPT_SETTINGS, &value, 1))
		return false;

	if (enable)
		value |= INTERRUPT_SETTINGS_BIT_OVERFLOW_INT_EN; // Overflow Int En - set to 1 (enabled)
	else
		value &= ~INTERRUPT_SETTINGS_BIT_OVERFLOW_INT_EN; // Overflow Int En - set to 0 (disabled)

	return writeReg(REG_INTERRUPT_SETTINGS, &value);
}

bool BMM150_I2C::enableDataOverrun(const bool enable)
{
	// Registers other than power control are not accessible in suspend mode
	if (_suspended)
		return false;

	uint8_t value = 0;
	if (!readReg(REG_INTERRUPT_SETTINGS, &value, 1))
		return false;

	if (enable)
		value |= INTERRUPT_SETTINGS_BIT_DATA_OVERRUN_EN; // Data Overrun En - set to 1 (enabled)
	else
		value &= ~INTERRUPT_SETTINGS_BIT_DATA_OVERRUN_EN; // Data Overrun En - set to 0 (disabled)

	return writeReg(REG_INTERRUPT_SETTINGS, &value);
}

bool BMM150_I2C::enableInterrupts(const bool enable, const bool latched, const bool activeHigh)
{
	// Registers other than power control are not accessible in suspend mode
	if (_suspended)
		return false;

	uint8_t value = 0;
	if (!readReg(REG_INTERRUPT_AXES, &value, 1))
		return false;

	if (enable)
		value |= INTERRUPT_AXES_BIT_INTERRUPT_PIN_EN; // Interrupt Pin En - set to 1 (enabled)
	else
		value &= ~INTERRUPT_AXES_BIT_INTERRUPT_PIN_EN; // Interrupt Pin En - set to 0 (disabled)

	if (latched)
		value |= INTERRUPT_AXES_BIT_INTERRUPT_LATCH; // Interrupt Latch - set to 1 (latched)
	else
		value &= ~INTERRUPT_AXES_BIT_INTERRUPT_LATCH; // Interrupt Latch - set to 0 (non-latched)

	if (activeHigh)
		value |= INTERRUPT_AXES_BIT_INTERRUPT_POLARITY; // Interrupt Polarity - set to 1 (active high)
	else
		value &= ~INTERRUPT_AXES_BIT_INTERRUPT_POLARITY; // Interrupt Polarity - set to 0 (active low)

	return writeReg(REG_INTERRUPT_AXES, &value);
}

bool BMM150_I2C::enableDataReadyInterrupt(const bool enable, const bool activeHigh)
{
	// Registers other than power control are not accessible in suspend mode
	if (_suspended)
		return false;

	uint8_t value = 0;
	if (!readReg(REG_INTERRUPT_AXES, &value, 1))
		return false;

	if (enable)
		value |= INTERRUPT_AXES_BIT_DATA_READY_PIN_EN; // Data Ready Pin En - set to 1 (enabled)
	else
		value &= ~INTERRUPT_AXES_BIT_DATA_READY_PIN_EN; // Data Ready Pin En - set to 0 (disabled)

	if (activeHigh)
		value |= INTERRUPT_AXES_BIT_DR_POLARITY; // DR Polarity - set to 1 (active high)
	else
		value &= ~INTERRUPT_AXES_BIT_DR_POLARITY; // DR Polarity - set to 0 (active low)

	return writeReg(REG_INTERRUPT_AXES, &value);
}

bool BMM150_I2C::getInterruptStatus(InterruptStatus &status) const
{
	// Registers other than power control are not accessible in suspend mode (they read as 0x00)
	if (_suspended)
		return false;

	uint8_t value = 0;
	if (!readReg(REG_INTERRUPT_STATUS, &value, 1))
		return false;

	status.lowX = ((value & INTERRUPT_STATUS_BIT_LOW_INT_X) != 0);
	status.lowY = ((value & INTERRUPT_STATUS_BIT_LOW_INT_Y) != 0);
	status.lowZ = ((value & INTERRUPT_STATUS_BIT_LOW_INT_Z) != 0);
	status.highX = ((value & INTERRUPT_STATUS_BIT_HIGH_INT_X) != 0);
	status.highY = ((value & INTERRUPT_STATUS_BIT_HIGH_INT_Y) != 0);
	status.highZ = ((value & INTERRUPT_STATUS_BIT_HIGH_INT_Z) != 0);
	status.overflow = ((value & INTERRUPT_STATUS_BIT_OVERFLOW) != 0);
	status.dataOverrun = ((value & INTERRUPT_STATUS_BIT_DATA_OVERRUN) != 0);

	return true;
}

void BMM150_I2C::parseRawData(const uint8_t *const data, int16_t &x, int16_t &y, int16_t &z, uint16_t &rhall) const
{
	// Combine the high and low bytes into the upper bits of a 16-bit value, then shift down to sign extend
	const int16_t dataX = static_cast<int16_t>((data[REG_DATA_X_MSB - REG_DATA_X_LSB] << 8) |
											   (data[REG_DATA_X_LSB - REG_DATA_X_LSB] & DATA_X_LSB_BITS_DATA_X));
	x = dataX >> DATA_X_LSB_DATA_X_SHIFT;

	const int16_t dataY = static_cast<int16_t>((data[REG_DATA_Y_MSB - REG_DATA_X_LSB] << 8) |
											   (data[REG_DATA_Y_LSB - REG_DATA_X_LSB] & DATA_Y_LSB_BITS_DATA_Y));
	y = dataY >> DATA_Y_LSB_DATA_Y_SHIFT;

	const int16_t dataZ = static_cast<int16_t>((data[REG_DATA_Z_MSB - REG_DATA_X_LSB] << 8) |
											   (data[REG_DATA_Z_LSB - REG_DATA_X_LSB] & DATA_Z_LSB_BITS_DATA_Z));
	z = dataZ >> DATA_Z_LSB_DATA_Z_SHIFT;

	const uint16_t dataRhall = static_cast<uint16_t>((data[REG_RHALL_MSB - REG_DATA_X_LSB] << 8) |
													 (data[REG_RHALL_LSB - REG_DATA_X_LSB] & RHALL_LSB_BITS_RHALL));
	rhall = dataRhall >> RHALL_LSB_RHALL_SHIFT;
}

bool BMM150_I2C::readRawData(int16_t &x, int16_t &y, int16_t &z, uint16_t &rhall) const
{
	// Registers other than power control are not accessible in suspend mode (they read as 0x00)
	if (_suspended)
		return false;

	uint8_t data[DATA_LENGTH];
	if (!readReg(DATA_REGISTER, data, DATA_LENGTH))
		return false;

	parseRawData(data, x, y, z, rhall);
	return true;
}

int16_t BMM150_I2C::compensateX(const int16_t rawX, const uint16_t rhall) const
{
	if (rawX == DATA_XY_OVERFLOW)
		return MAGNETIC_FIELD_OVERFLOW;

	// Use the hall resistance if available, otherwise fall back to the trim reference
	const uint16_t resistance = (rhall != 0) ? rhall : _trimData.digXYZ1;
	if (resistance == 0)
		return MAGNETIC_FIELD_OVERFLOW;

	const int32_t processCompX1 = static_cast<int32_t>(_trimData.digXYZ1) * COMPENSATION_RESISTANCE_SCALE;
	const uint16_t processCompX2 = static_cast<uint16_t>(processCompX1 / resistance) - COMPENSATION_RESISTANCE_SCALE;
	const int16_t processCompX2Signed = static_cast<int16_t>(processCompX2);
	const int32_t processCompX3 = static_cast<int32_t>(processCompX2Signed) * static_cast<int32_t>(processCompX2Signed);
	const int32_t processCompX4 = static_cast<int32_t>(_trimData.digXY2) * (processCompX3 / 128);
	const int32_t processCompX5 = static_cast<int32_t>(static_cast<int16_t>(_trimData.digXY1) * 128);
	const int32_t processCompX6 = static_cast<int32_t>(processCompX2Signed) * processCompX5;
	const int32_t processCompX7 = ((processCompX4 + processCompX6) / 512) + COMPENSATION_XY_OFFSET;
	const int32_t processCompX8 = static_cast<int32_t>(static_cast<int16_t>(_trimData.digX2) + COMPENSATION_XY2_GAIN_OFFSET);
	const int32_t processCompX9 = (processCompX7 * processCompX8) / 4096;
	const int32_t processCompX10 = static_cast<int32_t>(rawX) * processCompX9;
	const int16_t processCompX11 = static_cast<int16_t>(processCompX10 / 8192);

	// Conversion from 16 LSB/µT to µT
	return static_cast<int16_t>((processCompX11 + (static_cast<int16_t>(_trimData.digX1) * COMPENSATION_XY1_SCALE)) / 16);
}

int16_t BMM150_I2C::compensateY(const int16_t rawY, const uint16_t rhall) const
{
	if (rawY == DATA_XY_OVERFLOW)
		return MAGNETIC_FIELD_OVERFLOW;

	// Use the hall resistance if available, otherwise fall back to the trim reference
	const uint16_t resistance = (rhall != 0) ? rhall : _trimData.digXYZ1;
	if (resistance == 0)
		return MAGNETIC_FIELD_OVERFLOW;

	const int32_t processCompY1 = (static_cast<int32_t>(_trimData.digXYZ1) * COMPENSATION_RESISTANCE_SCALE) / resistance;
	const uint16_t processCompY2 = static_cast<uint16_t>(processCompY1) - COMPENSATION_RESISTANCE_SCALE;
	const int16_t processCompY2Signed = static_cast<int16_t>(processCompY2);
	const int32_t processCompY3 = static_cast<int32_t>(processCompY2Signed) * static_cast<int32_t>(processCompY2Signed);
	const int32_t processCompY4 = static_cast<int32_t>(_trimData.digXY2) * (processCompY3 / 128);
	const int32_t processCompY5 = static_cast<int32_t>(static_cast<int16_t>(_trimData.digXY1) * 128);
	const int32_t processCompY6 = (processCompY4 + (static_cast<int32_t>(processCompY2Signed) * processCompY5)) / 512;
	const int32_t processCompY7 = static_cast<int32_t>(static_cast<int16_t>(_trimData.digY2) + COMPENSATION_XY2_GAIN_OFFSET);
	const int32_t processCompY8 = ((processCompY6 + COMPENSATION_XY_OFFSET) * processCompY7) / 4096;
	const int32_t processCompY9 = static_cast<int32_t>(rawY) * processCompY8;
	const int16_t processCompY10 = static_cast<int16_t>(processCompY9 / 8192);

	// Conversion from 16 LSB/µT to µT
	return static_cast<int16_t>((processCompY10 + (static_cast<int16_t>(_trimData.digY1) * COMPENSATION_XY1_SCALE)) / 16);
}

int16_t BMM150_I2C::compensateZ(const int16_t rawZ, const uint16_t rhall) const
{
	if (rawZ == DATA_Z_OVERFLOW)
		return MAGNETIC_FIELD_OVERFLOW;

	if ((_trimData.digZ2 == 0) || (_trimData.digZ1 == 0) || (rhall == 0) || (_trimData.digXYZ1 == 0))
		return MAGNETIC_FIELD_OVERFLOW;

	const int16_t processCompZ0 = static_cast<int16_t>(rhall) - static_cast<int16_t>(_trimData.digXYZ1);
	const int32_t processCompZ1 = (static_cast<int32_t>(_trimData.digZ3) * static_cast<int32_t>(processCompZ0)) / 4;
	const int32_t processCompZ2 = static_cast<int32_t>(rawZ - _trimData.digZ4) * 32768;
	const int32_t processCompZ3 = static_cast<int32_t>(_trimData.digZ1) * (static_cast<int16_t>(rhall) * 2);
	const int16_t processCompZ4 = static_cast<int16_t>((processCompZ3 + COMPENSATION_Z_ROUNDING) / COMPENSATION_Z_SCALE);
	int32_t processCompZ5 = (processCompZ2 - processCompZ1) / (_trimData.digZ2 + processCompZ4);

	// Saturate to the int16_t range (excluding MAGNETIC_FIELD_OVERFLOW)
	if (processCompZ5 > INT16_MAX)
		processCompZ5 = INT16_MAX;
	else if (processCompZ5 < -INT16_MAX)
		processCompZ5 = -INT16_MAX;

	// Conversion from 16 LSB/µT to µT
	return static_cast<int16_t>(processCompZ5 / 16);
}

bool BMM150_I2C::parseData(const uint8_t *const data, const uint8_t length, int16_t &x, int16_t &y, int16_t &z) const
{
	if (length != DATA_LENGTH)
		return false;

	int16_t rawX = 0;
	int16_t rawY = 0;
	int16_t rawZ = 0;
	uint16_t rhall = 0;
	parseRawData(data, rawX, rawY, rawZ, rhall);

	x = compensateX(rawX, rhall);
	y = compensateY(rawY, rhall);
	z = compensateZ(rawZ, rhall);

	return true;
}

bool BMM150_I2C::readData(int16_t &x, int16_t &y, int16_t &z) const
{
	// Registers other than power control are not accessible in suspend mode (they read as 0x00)
	if (_suspended)
		return false;

	uint8_t data[DATA_LENGTH];
	if (!readReg(DATA_REGISTER, data, DATA_LENGTH))
		return false;

	return parseData(data, DATA_LENGTH, x, y, z);
}

bool BMM150_I2C::readHeading(uint16_t &degrees) const
{
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;
	if (!readData(x, y, z) || (x == MAGNETIC_FIELD_OVERFLOW) || (y == MAGNETIC_FIELD_OVERFLOW))
		return false;

	const int32_t absX = abs(x);
	const int32_t absY = abs(y);
	if ((absX == 0) && (absY == 0))
		return false; // No horizontal field, heading is undefined

	// atan(r) ≈ 45r + 16r(1 - r) degrees, where r is the smaller axis / larger axis (0 to 1, scaled by 256)
	const int32_t ratio = (min(absX, absY) * 256) / max(absX, absY);
	const int32_t octant = ((45 * ratio) + ((16 * ratio * (256 - ratio)) / 256)) / 256;
	const int32_t quadrant = (absX <= absY) ? octant : (90 - octant);

	if (x >= 0)
		degrees = static_cast<uint16_t>((y >= 0) ? quadrant : (180 - quadrant));
	else
		degrees = static_cast<uint16_t>((y < 0) ? (180 + quadrant) : ((360 - quadrant) % 360));

	return true;
}
