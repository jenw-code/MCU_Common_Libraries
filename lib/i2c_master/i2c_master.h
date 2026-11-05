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

/// @brief Base class for talking to an I2C device, with the MCU as the master
class I2C_Master
{
private:
	const uint8_t _address;
	TwoWire &_wire;
	bool _wireStarted = false;

protected:
	// Capped at UINT8_MAX as requestFrom() takes a uint8_t length on most cores
	static constexpr size_t MAX_I2C_BUFFER_LENGTH = (I2C_MASTER_WIRE_BUFFER_LENGTH > UINT8_MAX) ? UINT8_MAX : I2C_MASTER_WIRE_BUFFER_LENGTH;

	/// @brief Read from consecutive registers, starting at reg
	/// @param reg  - First register to read
	/// @param pBuf - Buffer to receive the data
	/// @param size - Number of bytes to read
	/// @return True if successful
	bool readReg(const uint8_t reg, uint8_t *const pBuf, const size_t size) const;

	/// @brief Read one register and return its value
	/// @brief Note: Returns 0 on failure, so only use for registers where 0 is never a valid value
	/// @param reg  - Register to read
	/// @return The register value
	uint8_t readReg(const uint8_t reg) const
	{
		uint8_t pBuf = 0;
		return readReg(reg, &pBuf, 1) ? pBuf : 0;
	}

	/// @brief Write a register address with no data. Useful for commands that have no data
	/// @param reg  - Register to write
	/// @return True if successful
	bool writeReg(const uint8_t reg);

	/// @brief Write to consecutive registers, starting at reg
	/// @param reg  - First register to write
	/// @param pBuf - Data to write
	/// @param size - Number of bytes to write
	/// @return True if successful
	bool writeReg(const uint8_t reg, const uint8_t *const pBuf, const size_t size);

	/// @brief Write one register
	/// @param reg  - Register to write
	/// @param pBuf - Byte to write
	/// @return True if successful
	bool writeReg(const uint8_t reg, const uint8_t *const pBuf)
	{
		return writeReg(reg, pBuf, 1);
	}

	/// @brief Write to consecutive registers, starting at reg, splitting data larger than the I2C buffer into several transfers
	/// @param reg  - First register to write
	/// @param pBuf - Data to write
	/// @param size - Number of bytes to write
	/// @return True if successful
	bool writeRegExtended(const uint8_t reg, const uint8_t *const pBuf, const size_t size);

	/// @brief Set the I2C clock speed
	/// @param desiredclk Clock speed in Hz
	/// @return True if successful
	bool setSpeed(const uint32_t desiredclk);

	/// @brief Start the device. Called by begin on each attempt
	/// @return True if the device started
	virtual bool startWire();

public:
	/// @param i2cAddr The device's 7-bit I2C address
	/// @param i2cWire The I2C bus the device is connected to
	explicit I2C_Master(const uint8_t i2cAddr, TwoWire &i2cWire = Wire);

	/// @brief Start the device
	/// @param retryCount Number of attempts before giving up
	/// @return True if the device started
	bool begin(const uint8_t retryCount = 3);

	/// @brief Get the device's I2C address
	/// @return The 7-bit I2C address
	uint8_t address() const
	{
		return _address;
	}

	/// @brief Get whether the device has started
	/// @return True if begin succeeded
	bool started() const
	{
		return _wireStarted;
	}
};
