#include "mpu6500_i2c.h"

MPU6500_I2C::MPU6500_I2C(TwoWire &i2cWire, const I2CAddress i2cAddr) : I2C_Master(static_cast<uint8_t>(i2cAddr), i2cWire) {}

bool MPU6500_I2C::startWire()
{
	return I2C_Master::startWire() &&
		   (getWhoAmI() == IC_ID_VALUE);
}

bool MPU6500_I2C::setGyroOffsets(const int16_t x, const int16_t y, const int16_t z)
{
	const uint8_t offsets[] = {
		static_cast<uint8_t>(static_cast<uint16_t>(x) >> 8), // XG_OFFSET_H: X_OFFS_USR [15:8]
		static_cast<uint8_t>(x),							 // XG_OFFSET_L: X_OFFS_USR [7:0]
		static_cast<uint8_t>(static_cast<uint16_t>(y) >> 8), // YG_OFFSET_H: Y_OFFS_USR [15:8]
		static_cast<uint8_t>(y),							 // YG_OFFSET_L: Y_OFFS_USR [7:0]
		static_cast<uint8_t>(static_cast<uint16_t>(z) >> 8), // ZG_OFFSET_H: Z_OFFS_USR [15:8]
		static_cast<uint8_t>(z)								 // ZG_OFFSET_L: Z_OFFS_USR [7:0]
	};

	return writeReg(REG_XG_OFFSET_H, offsets, sizeof(offsets)); // XG_OFFSET_H to ZG_OFFSET_L are contiguous
}

bool MPU6500_I2C::writeAccelOffset(const uint8_t regH, const uint8_t regL, const uint8_t offsBits, const uint8_t offsShift, const int16_t offset)
{
	// Keep the reserved bit in the L register
	uint8_t valueL = 0;
	if (!readReg(regL, &valueL, 1))
		return false;

	// Shift the 15-bit offset into the upper bits of a 16-bit value, so it splits across H [14:7] and L [6:0]
	const uint16_t shiftedOffset = static_cast<uint16_t>(offset) << offsShift;

	const uint8_t data[] = {
		static_cast<uint8_t>(shiftedOffset >> 8),								// H: offset [14:7]
		static_cast<uint8_t>((valueL & ~offsBits) | (shiftedOffset & offsBits)) // L: offset [6:0], reserved bit kept
	};

	return writeReg(regH, data, sizeof(data)); // H and L are contiguous
}

bool MPU6500_I2C::readAccelOffset(const uint8_t regH, const uint8_t regL, const uint8_t offsBits, const uint8_t offsShift, int16_t &offset) const
{
	uint8_t data[2];
	if (!readReg(regH, data, sizeof(data))) // H and L are contiguous
		return false;

	// Combine H and L into the upper bits of a 16-bit value, then shift down to sign extend
	const int16_t combined = static_cast<int16_t>((data[regH - regH] << 8) | (data[regL - regH] & offsBits));
	offset = combined >> offsShift;

	return true;
}

bool MPU6500_I2C::setAccelOffsets(const int16_t x, const int16_t y, const int16_t z)
{
	return writeAccelOffset(REG_XA_OFFSET_H, REG_XA_OFFSET_L, XA_OFFSET_L_BITS_XA_OFFS, XA_OFFSET_L_XA_OFFS_SHIFT, x) &&
		   writeAccelOffset(REG_YA_OFFSET_H, REG_YA_OFFSET_L, YA_OFFSET_L_BITS_YA_OFFS, YA_OFFSET_L_YA_OFFS_SHIFT, y) &&
		   writeAccelOffset(REG_ZA_OFFSET_H, REG_ZA_OFFSET_L, ZA_OFFSET_L_BITS_ZA_OFFS, ZA_OFFSET_L_ZA_OFFS_SHIFT, z);
}

bool MPU6500_I2C::getAccelOffsets(int16_t &x, int16_t &y, int16_t &z) const
{
	return readAccelOffset(REG_XA_OFFSET_H, REG_XA_OFFSET_L, XA_OFFSET_L_BITS_XA_OFFS, XA_OFFSET_L_XA_OFFS_SHIFT, x) &&
		   readAccelOffset(REG_YA_OFFSET_H, REG_YA_OFFSET_L, YA_OFFSET_L_BITS_YA_OFFS, YA_OFFSET_L_YA_OFFS_SHIFT, y) &&
		   readAccelOffset(REG_ZA_OFFSET_H, REG_ZA_OFFSET_L, ZA_OFFSET_L_BITS_ZA_OFFS, ZA_OFFSET_L_ZA_OFFS_SHIFT, z);
}

bool MPU6500_I2C::setInterruptConfig(const bool activeHigh, const bool openDrain, const bool latched, const bool clearOnAnyRead)
{
	uint8_t value = 0;
	if (!readReg(REG_INT_PIN_CFG, &value, 1))
		return false;

	if (activeHigh)
		value &= ~INT_PIN_CFG_BIT_ACTL; // ACTL - set to 0 (active high)
	else
		value |= INT_PIN_CFG_BIT_ACTL; // ACTL - set to 1 (active low)

	if (openDrain)
		value |= INT_PIN_CFG_BIT_OPEN; // OPEN - set to 1 (open drain)
	else
		value &= ~INT_PIN_CFG_BIT_OPEN; // OPEN - set to 0 (push-pull)

	if (latched)
		value |= INT_PIN_CFG_BIT_LATCH_INT_EN; // LATCH_INT_EN - set to 1 (held until status cleared)
	else
		value &= ~INT_PIN_CFG_BIT_LATCH_INT_EN; // LATCH_INT_EN - set to 0 (50 µs pulse)

	if (clearOnAnyRead)
		value |= INT_PIN_CFG_BIT_INT_ANYRD_2CLEAR; // INT_ANYRD_2CLEAR - set to 1 (cleared by any read)
	else
		value &= ~INT_PIN_CFG_BIT_INT_ANYRD_2CLEAR; // INT_ANYRD_2CLEAR - set to 0 (cleared by reading INT_STATUS)

	return writeReg(REG_INT_PIN_CFG, &value);
}

bool MPU6500_I2C::enableDataReadyInterrupt(const bool enable)
{
	uint8_t value = 0;
	if (!readReg(REG_INT_ENABLE, &value, 1))
		return false;

	if (enable)
		value |= INT_ENABLE_BIT_RAW_RDY_EN; // RAW_RDY_EN - set to 1 (enabled)
	else
		value &= ~INT_ENABLE_BIT_RAW_RDY_EN; // RAW_RDY_EN - set to 0 (disabled)

	return writeReg(REG_INT_ENABLE, &value);
}

bool MPU6500_I2C::enableFifoOverflowInterrupt(const bool enable)
{
	uint8_t value = 0;
	if (!readReg(REG_INT_ENABLE, &value, 1))
		return false;

	if (enable)
		value |= INT_ENABLE_BIT_FIFO_OVERFLOW_EN; // FIFO_OVERFLOW_EN - set to 1 (enabled)
	else
		value &= ~INT_ENABLE_BIT_FIFO_OVERFLOW_EN; // FIFO_OVERFLOW_EN - set to 0 (disabled)

	return writeReg(REG_INT_ENABLE, &value);
}

bool MPU6500_I2C::enableFsyncInterrupt(const bool enable, const bool activeHigh)
{
	// FSYNC pin as an interrupt source
	{
		uint8_t value = 0;
		if (!readReg(REG_INT_PIN_CFG, &value, 1))
			return false;

		if (enable)
			value |= INT_PIN_CFG_BIT_FSYNC_INT_MODE_EN; // FSYNC_INT_MODE_EN - set to 1 (FSYNC pin used as an interrupt)
		else
			value &= ~INT_PIN_CFG_BIT_FSYNC_INT_MODE_EN; // FSYNC_INT_MODE_EN - set to 0 (disabled)

		if (activeHigh)
			value &= ~INT_PIN_CFG_BIT_ACTL_FSYNC; // ACTL_FSYNC - set to 0 (active high)
		else
			value |= INT_PIN_CFG_BIT_ACTL_FSYNC; // ACTL_FSYNC - set to 1 (active low)

		if (!writeReg(REG_INT_PIN_CFG, &value))
			return false;
	}

	// FSYNC interrupt on the INT pin
	{
		uint8_t value = 0;
		if (!readReg(REG_INT_ENABLE, &value, 1))
			return false;

		if (enable)
			value |= INT_ENABLE_BIT_FSYNC_INT_EN; // FSYNC_INT_EN - set to 1 (enabled)
		else
			value &= ~INT_ENABLE_BIT_FSYNC_INT_EN; // FSYNC_INT_EN - set to 0 (disabled)

		return writeReg(REG_INT_ENABLE, &value);
	}
}

bool MPU6500_I2C::getInterruptStatus(InterruptStatus &status) const
{
	uint8_t value = 0;
	if (!readReg(REG_INT_STATUS, &value, 1))
		return false;

	status.rawDataReady = ((value & INT_STATUS_BIT_RAW_DATA_RDY_INT) != 0);
	status.dmp = ((value & INT_STATUS_BIT_DMP_INT) != 0);
	status.fsync = ((value & INT_STATUS_BIT_FSYNC_INT) != 0);
	status.fifoOverflow = ((value & INT_STATUS_BIT_FIFO_OVERFLOW_INT) != 0);
	status.wakeOnMotion = ((value & INT_STATUS_BIT_WOM_INT) != 0);

	return true;
}

bool MPU6500_I2C::setI2CMasterConfig(const bool waitForExternalSensor, const bool stopBetweenReads, const bool multiMaster, const bool delayShadow)
{
	// I2C master control
	{
		uint8_t value = 0;
		if (!readReg(REG_I2C_MST_CTRL, &value, 1))
			return false;

		if (waitForExternalSensor)
			value |= I2C_MST_CTRL_BIT_WAIT_FOR_ES; // WAIT_FOR_ES - set to 1 (delay data ready interrupt)
		else
			value &= ~I2C_MST_CTRL_BIT_WAIT_FOR_ES; // WAIT_FOR_ES - set to 0 (no delay)

		if (stopBetweenReads)
			value |= I2C_MST_CTRL_BIT_I2C_MST_P_NSR; // I2C_MST_P_NSR - set to 1 (stop between reads)
		else
			value &= ~I2C_MST_CTRL_BIT_I2C_MST_P_NSR; // I2C_MST_P_NSR - set to 0 (restart between reads)

		if (multiMaster)
			value |= I2C_MST_CTRL_BIT_MULT_MST_EN; // MULT_MST_EN - set to 1 (multi-master enabled)
		else
			value &= ~I2C_MST_CTRL_BIT_MULT_MST_EN; // MULT_MST_EN - set to 0 (multi-master disabled)

		if (!writeReg(REG_I2C_MST_CTRL, &value))
			return false;
	}

	// I2C master delay control
	{
		uint8_t value = 0;
		if (!readReg(REG_I2C_MST_DELAY_CTRL, &value, 1))
			return false;

		if (delayShadow)
			value |= I2C_MST_DELAY_CTRL_BIT_DELAY_ES_SHADOW; // DELAY_ES_SHADOW - set to 1 (shadow once all data received)
		else
			value &= ~I2C_MST_DELAY_CTRL_BIT_DELAY_ES_SHADOW; // DELAY_ES_SHADOW - set to 0 (no delay)

		return writeReg(REG_I2C_MST_DELAY_CTRL, &value);
	}
}

bool MPU6500_I2C::setI2CMasterDelay(const uint8_t delay)
{
	if (delay > I2C_SLV4_CTRL_BITS_I2C_MST_DLY)
		return false;

	uint8_t value = 0;
	if (!readReg(REG_I2C_SLV4_CTRL, &value, 1))
		return false;

	value &= ~I2C_SLV4_CTRL_BITS_I2C_MST_DLY; // I2C_MST_DLY - clear bits
	value |= delay;							  // I2C_MST_DLY - set to delay

	return writeReg(REG_I2C_SLV4_CTRL, &value);
}

bool MPU6500_I2C::readExternalSensorData(uint8_t *const data, const uint8_t index, const uint8_t length) const
{
	if ((length == 0) || ((index + length) > EXT_SENS_DATA_COUNT))
		return false;

	return readReg(REG_EXT_SENS_DATA_00 + index, data, length);
}
