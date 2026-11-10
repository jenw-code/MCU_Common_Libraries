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
	// A counting pattern (0 to 255) that sendPattern sends repeatedly, so any length can be sent from a small buffer
	static constexpr uint16_t PATTERN_SIZE = 256;
	static constexpr uint8_t PATTERN_ADDRESS_BITS = 8; // The DMA's reads wrap within 2^8 = 256 bytes
	static_assert((1 << PATTERN_ADDRESS_BITS) == PATTERN_SIZE, "The DMA's reads must wrap on the pattern's size");

	const RP235x_UART_DMA_TX_t _tx; // The DMA the mock device sends on

	// The DMA's reads wrap on a boundary of the pattern's size, so the pattern must be aligned to it
	alignas(PATTERN_SIZE) uint8_t _pattern[PATTERN_SIZE] = {};

	/// @brief Populate the pattern with a count from 0 to 255, so a receiver can check each byte
	/// @brief is one more than the last, and spot any that were dropped
	void populatePattern()
	{
		for (uint16_t index = 0; index < PATTERN_SIZE; ++index)
			_pattern[index] = static_cast<uint8_t>(index);
	}

	/// @brief Set whether the DMA's reads wrap round the pattern. The channel keeps the setting,
	/// @brief so every send sets it the way it needs it
	/// @param wrapReads True to wrap reads within the pattern's size, false to read straight through
	void setReadWrap(const bool wrapReads)
	{
		dma_channel_config config = dma_get_channel_config(_tx.channel);
		channel_config_set_ring(&config, false, wrapReads ? PATTERN_ADDRESS_BITS : 0); // 0 bits turns wrapping off
		dma_channel_set_config(_tx.channel, &config, false);
	}

	/// @brief Wait for the DMA to finish its current send, as the channel can only send one thing at a time
	void waitForSend() const
	{
		while (dma_channel_is_busy(_tx.channel))
		{
			// Wait for the current send to finish
		}
	}

public:
	/// @param tx The DMA to send on, set up by RP235x_UART_DMA
	explicit RP235x_Mock_Serial_Device(const RP235x_UART_DMA_TX_t &tx) : _tx(tx)
	{
		populatePattern();
	}

	/// @brief Send the data, each data array as soon as it has arrived. Returns once the last data array has been
	/// @brief handed to the DMA
	void run()
	{
		start();

		// A pattern send may still be in progress, using read wrapping
		waitForSend();
		setReadWrap(false);

		for (uint8_t index = 0; index < getArrayCount(); ++index)
		{
			while (calcArraysArrived() <= index)
			{
				// Wait for the data array to arrive
			}

			waitForSend();

			const DataArray &array = getArray(index);
			dma_channel_transfer_from_buffer_now(_tx.channel, array.data, array.length);
		}
	}

	/// @brief Send a counting pattern (0 to 255, repeated) of any length, e.g. to test a link with a large message
	/// @brief Returns once the DMA has started, so the core is free while it sends (see sendInProgress)
	/// @param length Number of bytes to send
	void sendPattern(const uint32_t length)
	{
		waitForSend();
		setReadWrap(true);

		dma_channel_transfer_from_buffer_now(_tx.channel, _pattern, length);
	}

	/// @brief Get whether the DMA is still sending
	/// @return True while a send is in progress
	bool sendInProgress() const
	{
		return dma_channel_is_busy(_tx.channel);
	}
};
