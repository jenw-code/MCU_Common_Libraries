#pragma once

#include <Arduino.h>
#include <hardware/dma.h>
#include "mock_serial_base.h"
#include "rp235x_uart_dma.h"

/// @brief A mock device for testing: it sends the mock data out of a real UART by DMA, as an external device would
/// @brief Its TX pin is wired to the RX pin of the class being tested
/// @brief Run it on its own core (e.g. from setup1()), so it keeps its own timing, like a real device
class RP235x_Mock_Serial_Device final : public Mock_Serial_Base
{
private:
	const RP235x_UART_DMA_TX_t _tx; // The DMA the mock device sends on

public:
	/// @param tx The DMA to send on, set up by RP235x_UART_DMA
	explicit RP235x_Mock_Serial_Device(const RP235x_UART_DMA_TX_t &tx) : _tx(tx) {}

	/// @brief Send the data, each data array as soon as it has arrived. Returns once the last data array has been
	/// @brief handed to the DMA
	void run()
	{
		start();

		for (uint8_t index = 0; index < getArrayCount(); ++index)
		{
			while (calcArraysArrived() <= index)
			{
				// Wait for the data array to arrive
			}

			// The channel can only send one data array at a time
			while (dma_channel_is_busy(_tx.channel))
			{
				// Wait for the previous data array to finish sending
			}

			const DataArray &array = getArray(index);
			dma_channel_transfer_from_buffer_now(_tx.channel, array.data, array.length);
		}
	}
};
