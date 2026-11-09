#pragma once

#include <Arduino.h>
#include <hardware/dma.h>
#include "rp2xxx_uart.h"

/// @brief The DMA channel used to receive a UART, and the size of its ring buffer. The channel is held as its index,
/// @brief so a config can be a constexpr
/// @tparam BUFFER_SIZE The ring buffer's size (bytes). A power of 2, from 2 to 32768
template <uint16_t BUFFER_SIZE>
struct RP235x_UART_DMA_Config_t final
{
	uint8_t channel = 0; // 0 to NUM_DMA_CHANNELS - 1
};

/// @brief A UART receiver whose bytes a DMA copies into a ring buffer: the ring, and the channel filling it
/// @tparam BUFFER_SIZE The ring buffer's size (bytes)
template <uint16_t BUFFER_SIZE>
struct RP235x_UART_DMA_RX_t final
{
	volatile uint8_t *buffer = nullptr; // The ring buffer, declared by the caller of setup_RX
	uint channel = 0;
	uint32_t baudRate = 0; // The UART's baud rate (bits per second)
};

/// @brief Helper that has a DMA copy every byte a UART receives into a ring buffer, so no core time is spent
/// @brief per byte
class RP235x_UART_DMA final
{
private:
	/// @brief Calculate how many of the write address's low bits the DMA ring wraps within
	/// @brief e.g. 64 bytes gives 6 bits (2^6 = 64)
	/// @param bufferSize The ring buffer's size (bytes)
	/// @return The number of bits. Rounded up if bufferSize isn't a power of 2 (e.g. 70 bytes gives 7 bits)
	static constexpr uint8_t calcRingAddressBits(const uint16_t bufferSize)
	{
		uint8_t bits = 0;
		while ((1 << bits) < bufferSize)
			++bits;

		return bits;
	}

public:
	// A utility, so it's never instantiated
	RP235x_UART_DMA() = delete;

	/// @brief Set up a DMA that copies every byte the UART receives into a ring buffer, endlessly
	/// @tparam BUFFER_SIZE The ring buffer's size (bytes), given by dmaConfig. A power of 2, from 2 to 32768
	/// @param dmaConfig The DMA channel to use, and the ring buffer's size
	/// @param uartRX The UART to receive from, set up by RP2xxx_PIO_UART or RP2xxx_UART
	/// @param buffer The ring buffer. It must be aligned to its own size, and stay valid while the DMA runs
	/// @param dmaRX Receives the DMA receiver's info. Only changed if the DMA is set up
	/// @return True if set up, false if the channel doesn't exist or is already claimed, the UART hasn't been set
	/// 		up, or the buffer isn't aligned to its own size
	template <uint16_t BUFFER_SIZE>
	static bool setup_RX(const RP235x_UART_DMA_Config_t<BUFFER_SIZE> &dmaConfig, const RP2xxx_UART_RX_t &uartRX,
						 volatile uint8_t (&buffer)[BUFFER_SIZE], RP235x_UART_DMA_RX_t<BUFFER_SIZE> &dmaRX)
	{
		static_assert((BUFFER_SIZE >= 2) && (BUFFER_SIZE <= 32768), "The DMA ring supports 2 to 32768 bytes");

		constexpr uint8_t RING_ADDRESS_BITS = calcRingAddressBits(BUFFER_SIZE);
		static_assert((1 << RING_ADDRESS_BITS) == BUFFER_SIZE, "The DMA ring wraps on a power of 2, so the buffer's size must be one");

		if (dmaConfig.channel >= NUM_DMA_CHANNELS)
			return false;

		if (uartRX.address == nullptr)
			return false;

		// The DMA wraps its write address on a boundary of the buffer's size, so the buffer must be aligned to it
		if ((reinterpret_cast<uintptr_t>(buffer) % BUFFER_SIZE) != 0)
			return false;

		// dma_channel_claim() halts if the channel is already claimed, rather than failing
		if (dma_channel_is_claimed(dmaConfig.channel))
			return false;

		dma_channel_claim(dmaConfig.channel);

		// Clear any bytes left in the buffer, so they can't be read as received
		for (uint16_t index = 0; index < BUFFER_SIZE; ++index)
			buffer[index] = 0;

		dma_channel_config config = dma_channel_get_default_config(dmaConfig.channel);
		channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
		channel_config_set_read_increment(&config, false);
		channel_config_set_write_increment(&config, true);
		channel_config_set_ring(&config, true, RING_ADDRESS_BITS);
		channel_config_set_dreq(&config, uartRX.dreq);

		// The transfer is endless, so the DMA keeps filling the ring buffer for as long as the UART receives
		dma_channel_configure(
			dmaConfig.channel,
			&config,
			buffer,
			uartRX.address,
			dma_encode_endless_transfer_count(),
			true);

		dmaRX.buffer = buffer;
		dmaRX.channel = dmaConfig.channel;
		dmaRX.baudRate = uartRX.baudRate;

		return true;
	}

	/// @brief Tear down a DMA receiver: stop its channel and unclaim it
	/// @tparam BUFFER_SIZE The ring buffer's size (bytes), given by dmaConfig
	/// @param dmaConfig The config given to setup_RX
	template <uint16_t BUFFER_SIZE>
	static void teardown_RX(const RP235x_UART_DMA_Config_t<BUFFER_SIZE> &dmaConfig)
	{
		dma_channel_abort(dmaConfig.channel);
		dma_channel_unclaim(dmaConfig.channel);
	}
};
