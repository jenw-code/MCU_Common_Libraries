#pragma once

#include <Arduino.h>
#include <Wire.h>

// The size of the Wire buffer is named differently on each core.
// If none of the known names are defined we fall back to 32 (the smallest common size).
#if defined(I2C_BUFFER_LENGTH) // ESP32
#define I2C_MASTER_WIRE_BUFFER_LENGTH I2C_BUFFER_LENGTH
#elif defined(BUFFER_LENGTH) // AVR, STM32
#define I2C_MASTER_WIRE_BUFFER_LENGTH BUFFER_LENGTH
#elif defined(WIRE_BUFFER_SIZE) // RP2040 (arduino-pico)
#define I2C_MASTER_WIRE_BUFFER_LENGTH WIRE_BUFFER_SIZE
#else
#define I2C_MASTER_WIRE_BUFFER_LENGTH 32
#endif

/// @brief Base class used to facilitate communication with an I2C device.
/// @brief This runs on the Master
class I2C_Master
{
private:
	const uint8_t _address;
	TwoWire &_wire;
	bool _wireStarted = false;

protected:
	// Capped at 255 as requestFrom() takes a uint8_t length on most cores
	static constexpr size_t MAX_I2C_BUFFER_LENGTH = (I2C_MASTER_WIRE_BUFFER_LENGTH > 255) ? 255 : I2C_MASTER_WIRE_BUFFER_LENGTH;

	/// @brief Read data from sensor chip register
	/// @param reg  - Chip register to read
	/// @param pBuf - Buffer for store data to read
	/// @param size - Number of data to read
	/// @return True if successful
	bool readReg(const uint8_t reg, uint8_t *const pBuf, const size_t size) const;

	/// @brief Read one byte and return it
	/// @param reg  - Chip register to read
	/// @return The data
	uint8_t readReg(const uint8_t reg) const
	{
		uint8_t pBuf = 0;
		return readReg(reg, &pBuf, 1) ? pBuf : 0;
	}

	/// @brief Sent notify to sensor register. This is useful to send commands that have no data
	/// @param reg  - Chip register to write
	/// @return True if successful
	bool writeReg(const uint8_t reg);

	/// @brief Write data to sensor register
	/// @param reg  - Chip register to write
	/// @param pBuf - Buffer for store data to write
	/// @param size - Number of data to in buffer
	/// @return True if successful
	bool writeReg(const uint8_t reg, const uint8_t *const pBuf, const size_t size);

	/// @brief Write one byte of data to sensor register
	/// @param reg  - Chip register to write
	/// @param pBuf - Buffer of one byte for store data to write
	/// @return True if successful
	bool writeReg(const uint8_t reg, const uint8_t *const pBuf)
	{
		return writeReg(reg, pBuf, 1);
	}

	/// @brief Write data to sensor register that can be larger than max I2C buffer size
	/// @param reg  - Chip register to write
	/// @param pBuf - Buffer for store data to write
	/// @param size - Number of data to in buffer
	/// @return True if successful
	bool writeRegExtended(const uint8_t reg, const uint8_t *const pBuf, const size_t size);

	/// @brief Changes speed of I2C comms
	/// @param desiredclk Speed
	/// @return True if successful
	bool setSpeed(const uint32_t desiredclk);

	/// @brief Routine to start device. Called from begin on each try / retry
	/// @return True if successfully started device
	virtual bool startWire();

public:
	explicit I2C_Master(const uint8_t i2cAddr, TwoWire &i2cWire = Wire);

	/// @brief Start device
	/// @param retryCount The number of retries before returning fail
	/// @return True if successfully started device
	bool begin(const uint8_t retryCount = 3);

	uint8_t address() const
	{
		return _address;
	}

	bool started() const
	{
		return _wireStarted;
	}
};
