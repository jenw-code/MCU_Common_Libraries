#include "mpu6500_i2c.h"

MPU6500_I2C::MPU6500_I2C(TwoWire &i2cWire, const I2CAddress i2cAddr) : I2C_Master(static_cast<uint8_t>(i2cAddr), i2cWire) {}

bool MPU6500_I2C::startWire()
{
	return I2C_Master::startWire() &&	   // The device must respond at its address before any of its registers are used
		   (readChipID() == IC_ID_VALUE) && // Confirm the device is an MPU-6500
		   initialiseFullScales();
}

bool MPU6500_I2C::initialiseFullScales()
{
	uint8_t gyroConfig = 0;
	uint8_t accelConfig = 0;
	if (!readReg(REG_GYRO_CONFIG, &gyroConfig, 1) || !readReg(REG_ACCEL_CONFIG, &accelConfig, 1))
		return false;

	switch (gyroConfig & GYRO_CONFIG_GYRO_FS_SEL_BITS_MASK)
	{
	case GYRO_CONFIG_GYRO_FS_SEL_250:
		_gyroFactor = GYRO_FACTOR_250DPS;
		break;
	case GYRO_CONFIG_GYRO_FS_SEL_500:
		_gyroFactor = GYRO_FACTOR_500DPS;
		break;
	case GYRO_CONFIG_GYRO_FS_SEL_1000:
		_gyroFactor = GYRO_FACTOR_1000DPS;
		break;
	case GYRO_CONFIG_GYRO_FS_SEL_2000:
		_gyroFactor = GYRO_FACTOR_2000DPS;
		break;
	default:
		return false;
	}

	switch (accelConfig & ACCEL_CONFIG_ACCEL_FS_SEL_BITS_MASK)
	{
	case ACCEL_CONFIG_ACCEL_FS_SEL_2G:
		_accelShift = ACCEL_SHIFT_2G;
		break;
	case ACCEL_CONFIG_ACCEL_FS_SEL_4G:
		_accelShift = ACCEL_SHIFT_4G;
		break;
	case ACCEL_CONFIG_ACCEL_FS_SEL_8G:
		_accelShift = ACCEL_SHIFT_8G;
		break;
	case ACCEL_CONFIG_ACCEL_FS_SEL_16G:
		_accelShift = ACCEL_SHIFT_16G;
		break;
	default:
		return false;
	}

	return true;
}

bool MPU6500_I2C::enableTemperatureSensor(const bool enable)
{
	uint8_t value = 0;
	if (!readReg(REG_PWR_MGMT_1, &value, 1))
		return false;

	if (enable)
		value &= ~PWR_MGMT_1_BIT_TEMP_DIS; // TEMP_DIS - set to 0 (temperature sensor enabled)
	else
		value |= PWR_MGMT_1_BIT_TEMP_DIS; // TEMP_DIS - set to 1 (temperature sensor disabled)

	return writeReg(REG_PWR_MGMT_1, &value);
}

bool MPU6500_I2C::setGyroOffsets(const int16_t x, const int16_t y, const int16_t z)
{
	// The offsets are written in one transfer, in register order
	static_assert((REG_XG_OFFSET_L == (REG_XG_OFFSET_H + 1)) &&
					  (REG_YG_OFFSET_H == (REG_XG_OFFSET_H + 2)) &&
					  (REG_YG_OFFSET_L == (REG_XG_OFFSET_H + 3)) &&
					  (REG_ZG_OFFSET_H == (REG_XG_OFFSET_H + 4)) &&
					  (REG_ZG_OFFSET_L == (REG_XG_OFFSET_H + 5)),
				  "XG_OFFSET_H to ZG_OFFSET_L must be contiguous");

	const uint8_t offsets[] = {
		static_cast<uint8_t>(static_cast<uint16_t>(x) >> 8), // XG_OFFSET_H: X offset high byte
		static_cast<uint8_t>(x),							 // XG_OFFSET_L: X offset low byte
		static_cast<uint8_t>(static_cast<uint16_t>(y) >> 8), // YG_OFFSET_H: Y offset high byte
		static_cast<uint8_t>(y),							 // YG_OFFSET_L: Y offset low byte
		static_cast<uint8_t>(static_cast<uint16_t>(z) >> 8), // ZG_OFFSET_H: Z offset high byte
		static_cast<uint8_t>(z)								 // ZG_OFFSET_L: Z offset low byte
	};

	return writeReg(REG_XG_OFFSET_H, offsets, sizeof(offsets));
}

bool MPU6500_I2C::writeAccelOffset(const uint8_t regH, const uint8_t regL, const uint8_t offsetBits, const uint8_t offsetShift, const int16_t offset)
{
	// Keep the reserved bit in the L register
	uint8_t valueL = 0;
	if (!readReg(regL, &valueL, 1))
		return false;

	// Bit 0 of the low byte is reserved, so the offset is shifted up past it
	const uint16_t shiftedOffset = static_cast<uint16_t>(offset) << offsetShift;

	const uint8_t valueH = static_cast<uint8_t>(shiftedOffset >> 8);
	valueL = static_cast<uint8_t>((valueL & ~offsetBits) | (shiftedOffset & offsetBits)); // Reserved bit kept

	return writeReg(regH, &valueH) &&
		   writeReg(regL, &valueL);
}

bool MPU6500_I2C::readAccelOffset(const uint8_t regH, const uint8_t regL, const uint8_t offsetBits, const uint8_t offsetShift, int16_t &offset) const
{
	uint8_t valueH = 0;
	uint8_t valueL = 0;
	if (!readReg(regH, &valueH, 1) || !readReg(regL, &valueL, 1))
		return false;

	// Combine H and L into the upper bits of a 16-bit value, then shift down to sign extend
	const int16_t combined = static_cast<int16_t>((valueH << 8) | (valueL & offsetBits));
	offset = combined >> offsetShift;

	return true;
}

bool MPU6500_I2C::setAccelOffsets(const int16_t x, const int16_t y, const int16_t z)
{
	return writeAccelOffset(REG_XA_OFFSET_H, REG_XA_OFFSET_L, XA_OFFSET_L_BITS_XA_OFFS, XA_OFFSET_L_XA_OFFS_SHIFT, x) &&
		   writeAccelOffset(REG_YA_OFFSET_H, REG_YA_OFFSET_L, YA_OFFSET_L_BITS_YA_OFFS, YA_OFFSET_L_YA_OFFS_SHIFT, y) &&
		   writeAccelOffset(REG_ZA_OFFSET_H, REG_ZA_OFFSET_L, ZA_OFFSET_L_BITS_ZA_OFFS, ZA_OFFSET_L_ZA_OFFS_SHIFT, z);
}

bool MPU6500_I2C::readAccelOffsets(int16_t &x, int16_t &y, int16_t &z) const
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

bool MPU6500_I2C::readExternalSensorData(uint8_t *const data, const uint8_t index, const uint8_t size) const
{
	// Register index is addressed as EXT_SENS_DATA_00 + index, and read in one transfer
	static_assert(REG_EXT_SENS_DATA_23 == (REG_EXT_SENS_DATA_00 + (EXT_SENS_DATA_SIZE - 1)),
				  "EXT_SENS_DATA_00 to EXT_SENS_DATA_23 must be contiguous");

	if ((size == 0) || ((index + size) > EXT_SENS_DATA_SIZE))
		return false;

	return readReg(REG_EXT_SENS_DATA_00 + index, data, size);
}

bool MPU6500_I2C::transferSlave4(const uint8_t addr, const uint8_t reg, const uint8_t data, const bool registerDisabled)
{
	// The setup is written in one transfer, in register order
	static_assert((REG_I2C_SLV4_REG == (REG_I2C_SLV4_ADDR + 1)) &&
					  (REG_I2C_SLV4_DO == (REG_I2C_SLV4_ADDR + 2)) &&
					  (REG_I2C_SLV4_CTRL == (REG_I2C_SLV4_ADDR + 3)),
				  "I2C_SLV4_ADDR to I2C_SLV4_CTRL must be contiguous");

	// Keep the I2C master delay and done interrupt settings, which share the control register
	uint8_t ctrl = 0;
	if (!readReg(REG_I2C_SLV4_CTRL, &ctrl, 1))
		return false;

	ctrl |= I2C_SLV4_CTRL_BIT_I2C_SLV4_EN; // I2C_SLV4_EN - set to 1 (start transfer)
	if (registerDisabled)
		ctrl |= I2C_SLV4_CTRL_BIT_I2C_SLV4_REG_DIS; // I2C_SLV4_REG_DIS - set to 1 (data only)
	else
		ctrl &= ~I2C_SLV4_CTRL_BIT_I2C_SLV4_REG_DIS; // I2C_SLV4_REG_DIS - set to 0 (register address written)

	const uint8_t setup[] = {
		addr, // I2C_SLV4_ADDR
		reg,  // I2C_SLV4_REG
		data, // I2C_SLV4_DO (written before the transfer is enabled)
		ctrl  // I2C_SLV4_CTRL
	};

	if (!writeReg(REG_I2C_SLV4_ADDR, setup, sizeof(setup)))
		return false;

	// Wait for the transfer to complete. Note: reading the status clears all of its bits, including other slaves' NACKs
	const unsigned long start = millis();
	do
	{
		uint8_t status = 0;
		if (!readReg(REG_I2C_MST_STATUS, &status, 1))
			return false;

		if ((status & I2C_MST_STATUS_BIT_I2C_SLV4_NACK) != 0)
			return false;

		if ((status & I2C_MST_STATUS_BIT_I2C_SLV4_DONE) != 0)
			return true;

		delay(1);

	} while ((millis() - start) < SLAVE4_TRANSFER_TIMEOUT_MILLIS);

	return false; // Transfer failed to complete in time
}

bool MPU6500_I2C::writeSlave4(const uint8_t address, const uint8_t reg, const uint8_t data, const bool registerDisabled)
{
	const uint8_t addr = address & I2C_SLV4_ADDR_BITS_I2C_ID_4; // I2C_SLV4_RNW - 0 (write)
	return transferSlave4(addr, reg, data, registerDisabled);
}

bool MPU6500_I2C::readSlave4(const uint8_t address, const uint8_t reg, uint8_t &data, const bool registerDisabled)
{
	const uint8_t addr = I2C_SLV4_ADDR_BIT_I2C_SLV4_RNW | (address & I2C_SLV4_ADDR_BITS_I2C_ID_4); // I2C_SLV4_RNW - 1 (read)
	if (!transferSlave4(addr, reg, 0, registerDisabled))
		return false;

	return readReg(REG_I2C_SLV4_DI, &data, 1);
}

bool MPU6500_I2C::enableSlave4DoneInterrupt(const bool enable)
{
	uint8_t value = 0;
	if (!readReg(REG_I2C_SLV4_CTRL, &value, 1))
		return false;

	if (enable)
		value |= I2C_SLV4_CTRL_BIT_SLV4_DONE_INT_EN; // SLV4_DONE_INT_EN - set to 1 (enabled)
	else
		value &= ~I2C_SLV4_CTRL_BIT_SLV4_DONE_INT_EN; // SLV4_DONE_INT_EN - set to 0 (disabled)

	return writeReg(REG_I2C_SLV4_CTRL, &value);
}

bool MPU6500_I2C::getI2CMasterStatus(I2CMasterStatus &status) const
{
	uint8_t value = 0;
	if (!readReg(REG_I2C_MST_STATUS, &value, 1))
		return false;

	status.slave0Nack = ((value & I2C_MST_STATUS_BIT_I2C_SLV0_NACK) != 0);
	status.slave1Nack = ((value & I2C_MST_STATUS_BIT_I2C_SLV1_NACK) != 0);
	status.slave2Nack = ((value & I2C_MST_STATUS_BIT_I2C_SLV2_NACK) != 0);
	status.slave3Nack = ((value & I2C_MST_STATUS_BIT_I2C_SLV3_NACK) != 0);
	status.slave4Nack = ((value & I2C_MST_STATUS_BIT_I2C_SLV4_NACK) != 0);
	status.lostArbitration = ((value & I2C_MST_STATUS_BIT_I2C_LOST_ARB) != 0);
	status.slave4Done = ((value & I2C_MST_STATUS_BIT_I2C_SLV4_DONE) != 0);
	status.passThrough = ((value & I2C_MST_STATUS_BIT_PASS_THROUGH) != 0);

	return true;
}

bool MPU6500_I2C::enableFifo(const bool enable)
{
	uint8_t value = 0;
	if (!readReg(REG_USER_CTRL, &value, 1))
		return false;

	if (enable)
		value |= USER_CTRL_BIT_FIFO_EN; // FIFO_EN - set to 1 (enabled)
	else
		value &= ~USER_CTRL_BIT_FIFO_EN; // FIFO_EN - set to 0 (disabled)

	return writeReg(REG_USER_CTRL, &value);
}

bool MPU6500_I2C::resetFifo()
{
	uint8_t value = 0;
	if (!readReg(REG_USER_CTRL, &value, 1))
		return false;

	value |= USER_CTRL_BIT_FIFO_RST; // FIFO_RST - set to 1 (reset FIFO, auto-cleared)

	return writeReg(REG_USER_CTRL, &value);
}

bool MPU6500_I2C::setFifoSources(const FifoSources &sources)
{
	uint8_t fifoEn = 0;
	if (sources.temperature)
		fifoEn |= FIFO_EN_BIT_TEMP_OUT; // TEMP_OUT - set to 1
	if (sources.gyroX)
		fifoEn |= FIFO_EN_BIT_GYRO_XOUT; // GYRO_XOUT - set to 1
	if (sources.gyroY)
		fifoEn |= FIFO_EN_BIT_GYRO_YOUT; // GYRO_YOUT - set to 1
	if (sources.gyroZ)
		fifoEn |= FIFO_EN_BIT_GYRO_ZOUT; // GYRO_ZOUT - set to 1
	if (sources.accel)
		fifoEn |= FIFO_EN_BIT_ACCEL; // ACCEL - set to 1
	if (sources.slave0)
		fifoEn |= FIFO_EN_BIT_SLV_0; // SLV_0 - set to 1
	if (sources.slave1)
		fifoEn |= FIFO_EN_BIT_SLV_1; // SLV_1 - set to 1
	if (sources.slave2)
		fifoEn |= FIFO_EN_BIT_SLV_2; // SLV_2 - set to 1

	if (!writeReg(REG_FIFO_EN, &fifoEn))
		return false;

	// Slave 3's FIFO enable is in the I2C master control register
	uint8_t value = 0;
	if (!readReg(REG_I2C_MST_CTRL, &value, 1))
		return false;

	if (sources.slave3)
		value |= I2C_MST_CTRL_BIT_SLV_3_FIFO_EN; // SLV_3_FIFO_EN - set to 1
	else
		value &= ~I2C_MST_CTRL_BIT_SLV_3_FIFO_EN; // SLV_3_FIFO_EN - set to 0

	return writeReg(REG_I2C_MST_CTRL, &value);
}

bool MPU6500_I2C::readFifoCount(uint16_t &count) const
{
	// Both bytes are read in one transfer. Reading the high byte first latches both bytes
	static_assert(REG_FIFO_COUNT_L == (REG_FIFO_COUNT_H + 1), "FIFO_COUNT_H and FIFO_COUNT_L must be contiguous");

	// Position of each byte in data. The read starts at FIFO_COUNT_H,
	// so each position is its register's offset from that one
	constexpr uint8_t FIFO_COUNT_H_INDEX = REG_FIFO_COUNT_H - REG_FIFO_COUNT_H;
	constexpr uint8_t FIFO_COUNT_L_INDEX = REG_FIFO_COUNT_L - REG_FIFO_COUNT_H;

	uint8_t data[2];
	if (!readReg(REG_FIFO_COUNT_H, data, sizeof(data)))
		return false;

	count = static_cast<uint16_t>(((data[FIFO_COUNT_H_INDEX] & FIFO_COUNT_H_BITS_FIFO_CNT) << 8) |
								  data[FIFO_COUNT_L_INDEX]);

	return true;
}

bool MPU6500_I2C::readFifo(uint8_t *const data, const uint16_t size) const
{
	if ((size == 0) || (size > FIFO_SIZE))
		return false;

	// Each read of FIFO_R_W returns the next FIFO byte, so the data is read in chunks that fit the I2C buffer
	uint16_t bytesRead = 0;
	while (bytesRead < size)
	{
		const uint16_t remaining = size - bytesRead;
		const uint8_t chunkSize = static_cast<uint8_t>((remaining > MAX_I2C_BUFFER_SIZE) ? MAX_I2C_BUFFER_SIZE : remaining);

		if (!readReg(REG_FIFO_R_W, &data[bytesRead], chunkSize))
			return false;

		bytesRead += chunkSize;
	}

	return true;
}

bool MPU6500_I2C::reset()
{
	uint8_t value = 0;
	if (!readReg(REG_PWR_MGMT_1, &value, 1))
		return false;

	value |= PWR_MGMT_1_BIT_DEVICE_RESET; // DEVICE_RESET - set to 1 (reset all registers, auto-cleared)
	if (!writeReg(REG_PWR_MGMT_1, &value))
		return false;

	delay(RESET_TIME_MILLIS);
	return true;
}

bool MPU6500_I2C::resetSignalPaths(const bool gyro, const bool accel, const bool temperature)
{
	uint8_t value = 0;
	if (gyro)
		value |= SIGNAL_PATH_RESET_BIT_GYRO_RST; // GYRO_RST - set to 1 (reset gyro signal path)
	if (accel)
		value |= SIGNAL_PATH_RESET_BIT_ACCEL_RST; // ACCEL_RST - set to 1 (reset accel signal path)
	if (temperature)
		value |= SIGNAL_PATH_RESET_BIT_TEMP_RST; // TEMP_RST - set to 1 (reset temperature signal path)

	return writeReg(REG_SIGNAL_PATH_RESET, &value);
}

bool MPU6500_I2C::resetSensors()
{
	uint8_t value = 0;
	if (!readReg(REG_USER_CTRL, &value, 1))
		return false;

	value |= USER_CTRL_BIT_SIG_COND_RST; // SIG_COND_RST - set to 1 (reset signal paths and clear sensor registers)

	return writeReg(REG_USER_CTRL, &value);
}

bool MPU6500_I2C::resetI2CMaster()
{
	uint8_t value = 0;
	if (!readReg(REG_USER_CTRL, &value, 1))
		return false;

	value |= USER_CTRL_BIT_I2C_MST_RST; // I2C_MST_RST - set to 1 (reset I2C master, auto-cleared)

	return writeReg(REG_USER_CTRL, &value);
}

bool MPU6500_I2C::resetDmp()
{
	uint8_t value = 0;
	if (!readReg(REG_USER_CTRL, &value, 1))
		return false;

	// The reset only takes effect while the DMP is disabled
	if ((value & USER_CTRL_BIT_DMP_EN) != 0)
		return false;

	value |= USER_CTRL_BIT_DMP_RST; // DMP_RST - set to 1 (reset DMP, auto-cleared)

	return writeReg(REG_USER_CTRL, &value);
}

void MPU6500_I2C::parseRawData(const uint8_t *const data, RawSensorData &raw) const
{
	// Position of each data register in data. The read starts at DATA_REGISTER,
	// so each position is its register's offset from that one
	constexpr uint8_t ACCEL_XOUT_H_INDEX = REG_ACCEL_XOUT_H - DATA_REGISTER;
	constexpr uint8_t ACCEL_XOUT_L_INDEX = REG_ACCEL_XOUT_L - DATA_REGISTER;
	constexpr uint8_t ACCEL_YOUT_H_INDEX = REG_ACCEL_YOUT_H - DATA_REGISTER;
	constexpr uint8_t ACCEL_YOUT_L_INDEX = REG_ACCEL_YOUT_L - DATA_REGISTER;
	constexpr uint8_t ACCEL_ZOUT_H_INDEX = REG_ACCEL_ZOUT_H - DATA_REGISTER;
	constexpr uint8_t ACCEL_ZOUT_L_INDEX = REG_ACCEL_ZOUT_L - DATA_REGISTER;
	constexpr uint8_t TEMP_OUT_H_INDEX = REG_TEMP_OUT_H - DATA_REGISTER;
	constexpr uint8_t TEMP_OUT_L_INDEX = REG_TEMP_OUT_L - DATA_REGISTER;
	constexpr uint8_t GYRO_XOUT_H_INDEX = REG_GYRO_XOUT_H - DATA_REGISTER;
	constexpr uint8_t GYRO_XOUT_L_INDEX = REG_GYRO_XOUT_L - DATA_REGISTER;
	constexpr uint8_t GYRO_YOUT_H_INDEX = REG_GYRO_YOUT_H - DATA_REGISTER;
	constexpr uint8_t GYRO_YOUT_L_INDEX = REG_GYRO_YOUT_L - DATA_REGISTER;
	constexpr uint8_t GYRO_ZOUT_H_INDEX = REG_GYRO_ZOUT_H - DATA_REGISTER;
	constexpr uint8_t GYRO_ZOUT_L_INDEX = REG_GYRO_ZOUT_L - DATA_REGISTER;

	raw.accelX = static_cast<int16_t>((data[ACCEL_XOUT_H_INDEX] << 8) | data[ACCEL_XOUT_L_INDEX]);
	raw.accelY = static_cast<int16_t>((data[ACCEL_YOUT_H_INDEX] << 8) | data[ACCEL_YOUT_L_INDEX]);
	raw.accelZ = static_cast<int16_t>((data[ACCEL_ZOUT_H_INDEX] << 8) | data[ACCEL_ZOUT_L_INDEX]);
	raw.temperature = static_cast<int16_t>((data[TEMP_OUT_H_INDEX] << 8) | data[TEMP_OUT_L_INDEX]);
	raw.gyroX = static_cast<int16_t>((data[GYRO_XOUT_H_INDEX] << 8) | data[GYRO_XOUT_L_INDEX]);
	raw.gyroY = static_cast<int16_t>((data[GYRO_YOUT_H_INDEX] << 8) | data[GYRO_YOUT_L_INDEX]);
	raw.gyroZ = static_cast<int16_t>((data[GYRO_ZOUT_H_INDEX] << 8) | data[GYRO_ZOUT_L_INDEX]);
}

bool MPU6500_I2C::readRawData(RawSensorData &raw) const
{
	uint8_t data[DATA_SIZE];
	if (!readReg(DATA_REGISTER, data, DATA_SIZE))
		return false;

	parseRawData(data, raw);
	return true;
}

int32_t MPU6500_I2C::temperatureFromRaw(const int16_t raw)
{
	return ((static_cast<int32_t>(raw) * TEMPERATURE_FACTOR) >> SCALE_SHIFT) + TEMPERATURE_OFFSET;
}

bool MPU6500_I2C::parseData(const uint8_t *const data, const uint8_t size, SensorData &result) const
{
	if (size != DATA_SIZE)
		return false;

	RawSensorData raw;
	parseRawData(data, raw);

	result.accelX = static_cast<int16_t>((static_cast<int32_t>(raw.accelX) * MG_PER_G) >> _accelShift);
	result.accelY = static_cast<int16_t>((static_cast<int32_t>(raw.accelY) * MG_PER_G) >> _accelShift);
	result.accelZ = static_cast<int16_t>((static_cast<int32_t>(raw.accelZ) * MG_PER_G) >> _accelShift);
	result.temperature = temperatureFromRaw(raw.temperature);
	result.gyroX = (static_cast<int32_t>(raw.gyroX) * _gyroFactor) >> SCALE_SHIFT;
	result.gyroY = (static_cast<int32_t>(raw.gyroY) * _gyroFactor) >> SCALE_SHIFT;
	result.gyroZ = (static_cast<int32_t>(raw.gyroZ) * _gyroFactor) >> SCALE_SHIFT;

	return true;
}

bool MPU6500_I2C::readData(SensorData &result) const
{
	uint8_t data[DATA_SIZE];
	if (!readReg(DATA_REGISTER, data, DATA_SIZE))
		return false;

	return parseData(data, DATA_SIZE, result);
}

bool MPU6500_I2C::readSelfTestAverageRawData(RawSensorData &average) const
{
	int32_t accelX = 0;
	int32_t accelY = 0;
	int32_t accelZ = 0;
	int32_t temperature = 0;
	int32_t gyroX = 0;
	int32_t gyroY = 0;
	int32_t gyroZ = 0;

	for (uint8_t sample = 0; sample < SELF_TEST_SAMPLE_COUNT; ++sample)
	{
		RawSensorData raw;
		if (!readRawData(raw))
			return false;

		accelX += raw.accelX;
		accelY += raw.accelY;
		accelZ += raw.accelZ;
		temperature += raw.temperature;
		gyroX += raw.gyroX;
		gyroY += raw.gyroY;
		gyroZ += raw.gyroZ;

		delay(SELF_TEST_SAMPLE_PERIOD_MILLIS);
	}

	average.accelX = static_cast<int16_t>(accelX / SELF_TEST_SAMPLE_COUNT);
	average.accelY = static_cast<int16_t>(accelY / SELF_TEST_SAMPLE_COUNT);
	average.accelZ = static_cast<int16_t>(accelZ / SELF_TEST_SAMPLE_COUNT);
	average.temperature = static_cast<int16_t>(temperature / SELF_TEST_SAMPLE_COUNT);
	average.gyroX = static_cast<int16_t>(gyroX / SELF_TEST_SAMPLE_COUNT);
	average.gyroY = static_cast<int16_t>(gyroY / SELF_TEST_SAMPLE_COUNT);
	average.gyroZ = static_cast<int16_t>(gyroZ / SELF_TEST_SAMPLE_COUNT);

	return true;
}

bool MPU6500_I2C::validGyroSelfTestResponse(const int32_t response)
{
	const int32_t magnitude = (response < 0) ? -response : response;
	return magnitude >= SELF_TEST_GYRO_MIN_RESPONSE;
}

bool MPU6500_I2C::validAccelSelfTestResponse(const int32_t response)
{
	const int32_t magnitude = (response < 0) ? -response : response;
	return (magnitude >= SELF_TEST_ACCEL_MIN_RESPONSE) && (magnitude <= SELF_TEST_ACCEL_MAX_RESPONSE);
}

bool MPU6500_I2C::validSelfTestTemperature(const int16_t rawTemperature)
{
	const int32_t temperature = temperatureFromRaw(rawTemperature);
	return (temperature >= SELF_TEST_TEMPERATURE_MIN) &&
		   (temperature <= static_cast<int32_t>(SELF_TEST_TEMPERATURE_MAX));
}

bool MPU6500_I2C::readSelfTestAverages(RawSensorData &normal, RawSensorData &testing)
{
	if (!writeReg(REG_SMPLRT_DIV, SELF_TEST_CONFIG, sizeof(SELF_TEST_CONFIG)))
		return false;

	// The test configuration sets ±250 dps and ±2 g, so the data scaling must match
	_gyroFactor = GYRO_FACTOR_250DPS;
	_accelShift = ACCEL_SHIFT_2G;

	delay(SELF_TEST_SETTLE_TIME_MILLIS);
	if (!readSelfTestAverageRawData(normal))
		return false;

	if (!writeReg(REG_GYRO_CONFIG, SELF_TEST_ENABLE, sizeof(SELF_TEST_ENABLE)))
		return false;

	delay(SELF_TEST_SETTLE_TIME_MILLIS);
	if (!readSelfTestAverageRawData(testing))
		return false;

	// Turn the self-test off by writing the test configuration again
	if (!writeReg(REG_SMPLRT_DIV, SELF_TEST_CONFIG, sizeof(SELF_TEST_CONFIG)))
		return false;

	delay(SELF_TEST_SETTLE_TIME_MILLIS);
	return true;
}

bool MPU6500_I2C::normalSelfTest()
{
	// Turn on everything the test checks
	if (!setPowerMode<PowerMode::SixAxis>() || !enableTemperatureSensor(true))
		return false;

	RawSensorData normal;
	RawSensorData testing;
	if (!readSelfTestAverages(normal, testing))
		return false;

	return validGyroSelfTestResponse(testing.gyroX - normal.gyroX) &&
		   validGyroSelfTestResponse(testing.gyroY - normal.gyroY) &&
		   validGyroSelfTestResponse(testing.gyroZ - normal.gyroZ) &&
		   validAccelSelfTestResponse(testing.accelX - normal.accelX) &&
		   validAccelSelfTestResponse(testing.accelY - normal.accelY) &&
		   validAccelSelfTestResponse(testing.accelZ - normal.accelZ) &&
		   validSelfTestTemperature(normal.temperature);
}

bool MPU6500_I2C::disableWakeOnMotion()
{
	// Wake-on-motion interrupt
	{
		uint8_t value = 0;
		if (!readReg(REG_INT_ENABLE, &value, 1))
			return false;

		value &= ~INT_ENABLE_BIT_WOM_EN; // WOM_EN - set to 0 (disabled)
		if (!writeReg(REG_INT_ENABLE, &value))
			return false;
	}

	// Motion detection logic
	{
		uint8_t value = 0;
		if (!readReg(REG_ACCEL_INTEL_CTRL, &value, 1))
			return false;

		value &= ~(ACCEL_INTEL_CTRL_BIT_ACCEL_INTEL_EN | ACCEL_INTEL_CTRL_BIT_ACCEL_INTEL_MODE); // ACCEL_INTEL_EN, ACCEL_INTEL_MODE - set to 0
		return writeReg(REG_ACCEL_INTEL_CTRL, &value);
	}
}

bool MPU6500_I2C::enableLowPower()
{
	// The low-power rate and filter are set before entering low-power mode, as the datasheet does
	return enableTemperatureSensor(false) &&
		   setAccelFilter<AccelFilter::Bandwidth_1130Hz>() &&
		   setLowPowerAccelRate<LowPowerAccelRate::Rate_15_63Hz>() &&
		   setPowerMode<PowerMode::LowPowerAccelerometer>();
}
