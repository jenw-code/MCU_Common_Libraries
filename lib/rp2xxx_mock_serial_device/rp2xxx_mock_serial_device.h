#pragma once

#include <Arduino.h>
#include "mock_serial_base.h"
#include "rp2xxx_uart.h"

/// @brief A mock device for testing: it sends the mock data out of a real UART, as an external device would
/// @brief Its TX pin is wired to the RX pin of the class being tested
/// @brief Run it on its own core (e.g. from setup1()), so it keeps its own timing, like a real device
class RP2xxx_Mock_Serial_Device final : public Mock_Serial_Base
{
private:
	const RP2xxx_UART_TX_t _tx; // The UART the mock device sends on

	/// @brief Send a byte, waiting while the TX FIFO is full
	/// @param byte The byte to send
	void writeByte(const uint8_t byte)
	{
		while ((*_tx.statusAddress & _tx.fullMask) != 0)
		{
			// Wait for space in the TX FIFO
		}

		*_tx.address = byte;
	}

public:
	/// @param tx The UART to send on, set up by RP2xxx_PIO_UART or RP2xxx_UART
	explicit RP2xxx_Mock_Serial_Device(const RP2xxx_UART_TX_t &tx) : _tx(tx) {}

	/// @brief Send the data, each byte as soon as it has arrived. Returns once all the data has been sent
	void run()
	{
		start();

		const size_t dataLength = calcDataLength();
		size_t bytesSent = 0;
		while (bytesSent < dataLength)
		{
			const size_t bytesArrived = calcBytesArrived();
			while (bytesSent < bytesArrived)
			{
				writeByte(getByte(bytesSent));
				++bytesSent;
			}
		}
	}
};
