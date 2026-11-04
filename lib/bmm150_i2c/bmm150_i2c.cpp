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

bool BMM150_I2C::setOverflowInterrupt(const bool enable)
{
	uint8_t value = 0;
	if (!readReg(REG_INTERRUPT_SETTINGS, &value, 1))
		return false;

	if (enable)
		value |= INTERRUPT_SETTINGS_BIT_OVERFLOW_INT_EN; // Overflow Int En - set to 1 (enabled)
	else
		value &= ~INTERRUPT_SETTINGS_BIT_OVERFLOW_INT_EN; // Overflow Int En - set to 0 (disabled)

	return writeReg(REG_INTERRUPT_SETTINGS, &value);
}

bool BMM150_I2C::setDataOverrun(const bool enable)
{
	uint8_t value = 0;
	if (!readReg(REG_INTERRUPT_SETTINGS, &value, 1))
		return false;

	if (enable)
		value |= INTERRUPT_SETTINGS_BIT_DATA_OVERRUN_EN; // Data Overrun En - set to 1 (enabled)
	else
		value &= ~INTERRUPT_SETTINGS_BIT_DATA_OVERRUN_EN; // Data Overrun En - set to 0 (disabled)

	return writeReg(REG_INTERRUPT_SETTINGS, &value);
}

bool BMM150_I2C::setInterruptPin(const bool enable, const bool latched, const bool activeHigh)
{
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

bool BMM150_I2C::setDataReadyPin(const bool enable, const bool activeHigh)
{
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
