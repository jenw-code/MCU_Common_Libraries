#include "bmm150_i2c.h"

BMM150_I2C::BMM150_I2C(TwoWire &i2cWire, const I2CAddress i2cAddr) : I2C_Master(static_cast<uint8_t>(i2cAddr), i2cWire) {}

bool BMM150_I2C::startWire()
{
	return I2C_Master::startWire() &&
		   setPowerMode<PowerMode::Sleep>() &&
		   (getChipID() == IC_ID_VALUE);
}

bool BMM150_I2C::forceMeasurement()
{
	// Opmode register is only accessible once out of suspend mode
	uint8_t value = 0;
	if (!readReg(REG_POWER_CONTROL, &value, 1) || ((value & POWER_CONTROL_BIT_POWER_CONTROL) == 0))
		return false;

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
