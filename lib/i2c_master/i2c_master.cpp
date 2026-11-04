#include "i2c_master.h"

I2C_Master::I2C_Master(const uint8_t i2cAddr, TwoWire &i2cWire) : _address(i2cAddr),
																  _wire(i2cWire) {}

bool I2C_Master::begin(const uint8_t retryCount)
{
	if (!_wireStarted)
	{
		uint8_t iRetry = 0;
		do
		{
			if (iRetry > 0)
				delay(100);

			_wireStarted = startWire();

			++iRetry;
		} while (!_wireStarted && (iRetry < retryCount));
	}

	return _wireStarted;
}

bool I2C_Master::startWire()
{
	// Start comms as Master
	_wire.begin();

	// Now check we can find slave device
	_wire.beginTransmission(_address);

	return (_wire.endTransmission() == 0);
}

bool I2C_Master::readReg(const uint8_t reg, uint8_t *const pBuf, const size_t size) const
{
	if ((size == 0) || (size > MAX_I2C_BUFFER_LENGTH))
		return false;

	// Cast both arguments to uint8_t so the call is not ambiguous between requestFrom() overloads on different cores
	const uint8_t requestSize = static_cast<uint8_t>(size);

	_wire.beginTransmission(_address);
	if ((_wire.write(reg) == 1) &&
		(_wire.endTransmission(false) == 0) &&
		(_wire.requestFrom(_address, requestSize) == requestSize))
	{
		uint8_t iBuffer = 0;
		do
		{
			pBuf[iBuffer] = _wire.read();
			++iBuffer;
		} while (iBuffer < requestSize);

		return true;
	}

	return false;
}

bool I2C_Master::writeReg(const uint8_t reg)
{
	_wire.beginTransmission(_address);
	const bool writeSuccess = (_wire.write(reg) == 1);
	return (_wire.endTransmission() == 0) && writeSuccess;
}

bool I2C_Master::writeReg(const uint8_t reg, const uint8_t *const pBuf, const size_t size)
{
	// One byte of the buffer is used by the reg value
	if (size > (MAX_I2C_BUFFER_LENGTH - 1))
		return false;

	_wire.beginTransmission(_address);
	const bool writeSuccess = (_wire.write(reg) == 1) && (_wire.write(pBuf, size) == size);
	return (_wire.endTransmission() == 0) && writeSuccess;
}

bool I2C_Master::writeRegExtended(const uint8_t reg, const uint8_t *const pBuf, const size_t size)
{
	// The Wire buffer has a size limit (MAX_I2C_BUFFER_LENGTH) and one byte of it is used by the reg value.
	// We therefore need to chunk the data accordingly.
	constexpr size_t CHUNK_SIZE = 16; // Needs be divisible by 2 and less than MAX_I2C_BUFFER_LENGTH
	static_assert(CHUNK_SIZE < MAX_I2C_BUFFER_LENGTH, "CHUNK_SIZE must be less than MAX_I2C_BUFFER_LENGTH");

	size_t bytesWritten = 0;

	while (bytesWritten < size)
	{
		const size_t byteCountToWrite = ((size - bytesWritten) > CHUNK_SIZE) ? CHUNK_SIZE : (size - bytesWritten);

		_wire.beginTransmission(_address);
		if (_wire.write(static_cast<uint8_t>(reg + bytesWritten)) != 1)
			return false;

		if (_wire.write(&pBuf[bytesWritten], byteCountToWrite) != byteCountToWrite)
			return false;

		if (_wire.endTransmission() != 0)
			return false;

		bytesWritten += byteCountToWrite;
	}

	return true;
}

bool I2C_Master::setSpeed(const uint32_t desiredclk)
{
	_wire.setClock(desiredclk);
	return true;
}
