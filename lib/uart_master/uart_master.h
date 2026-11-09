#pragma once

#include <Arduino.h>

/// @brief Base class for talking to a UART device, used in the same way as I2C_Master
class UART_Master
{
public:
	enum class BaudRate : uint32_t
	{
		Baud9600 = 9600,
		Baud19200 = 19200,
		Baud38400 = 38400,
		Baud57600 = 57600,
		Baud115200 = 115200,
		Baud230400 = 230400,
		Baud460800 = 460800,
		Baud921600 = 921600,
		Baud1000000 = 1000000,
		Baud2000000 = 2000000
	};

private:
	HardwareSerial &_serial;
	const BaudRate _baudRate;
	bool _serialStarted = false;

protected:
	/// @brief Write bytes to the device
	/// @param pBuf - Data to write
	/// @param size - Number of bytes to write
	/// @return True if every byte was written
	bool writeBytes(const uint8_t *const pBuf, const size_t size);

	/// @brief Get how many received bytes are waiting to be read
	/// @return Number of bytes waiting
	size_t bytesAvailable() const;

	/// @brief Read one received byte
	/// @brief Note: Only call when bytesAvailable() is above 0
	/// @return The byte read
	uint8_t readByte();

	/// @brief Start the device. Called by begin on each attempt
	/// @return True if the device started
	virtual bool startSerial();

public:
	/// @param serial The UART the device is connected to (hardware or PIO serial)
	/// @param baudRate The device's baud rate
	UART_Master(HardwareSerial &serial, const BaudRate baudRate);

	// Virtual, so a device can be deleted through a base pointer
	virtual ~UART_Master() = default;

	/// @brief Start the device
	/// @param retryCount Number of attempts before giving up
	/// @return True if the device started
	bool begin(const uint8_t retryCount = 3);

	/// @brief Get whether the device has started
	/// @return True if begin succeeded
	bool started() const
	{
		return _serialStarted;
	}
};
