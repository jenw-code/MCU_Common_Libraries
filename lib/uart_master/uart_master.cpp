#include "uart_master.h"

UART_Master::UART_Master(HardwareSerial &serial, const BaudRate baudRate) : _serial(serial),
																			_baudRate(baudRate) {}

bool UART_Master::begin(const uint8_t retryCount)
{
	if (!_serialStarted)
	{
		uint8_t iRetry = 0;
		do
		{
			if (iRetry > 0)
				delay(100);

			_serialStarted = startSerial();

			++iRetry;
		} while (!_serialStarted && (iRetry < retryCount));
	}

	return _serialStarted;
}

bool UART_Master::startSerial()
{
	// UART has no acknowledge, so a derived class checks that its device responds
	_serial.begin(static_cast<uint32_t>(_baudRate));
	return true;
}

bool UART_Master::writeBytes(const uint8_t *const pBuf, const size_t size)
{
	return (_serial.write(pBuf, size) == size);
}

size_t UART_Master::bytesAvailable() const
{
	return static_cast<size_t>(_serial.available());
}

uint8_t UART_Master::readByte()
{
	return static_cast<uint8_t>(_serial.read());
}
