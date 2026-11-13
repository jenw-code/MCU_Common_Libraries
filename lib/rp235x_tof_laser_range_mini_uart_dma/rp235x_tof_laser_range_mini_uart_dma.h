#pragma once

#include <Arduino.h>
#include <hardware/dma.h>
#include <pico/time.h>
#include "tof_laser_range_mini.h"
#include "rp235x_uart.h"
#include "rp235x_uart_dma.h"

/// @brief Class for the Waveshare TOF Laser Range Sensor Mini (a Nooploop TOFSense-F2 Mini), over UART
/// @brief Uses Nooploop's NLink TOFSense protocol
/// @brief Receiving: DMA copies every byte received into a ring buffer, so no core time is spent per byte
/// @brief Sending: the core writes each request straight to the UART's TX FIFO. Requests are a few short frames,
/// @brief which don't justify tying up a DMA channel
/// @tparam BAUD_RATE The device's baud rate. The default is the rate the device is supplied with
/// @tparam DMA_BUFFER_SIZE The DMA ring buffer's size (bytes). It must be larger than the bytes kept for the
/// longest frame
template <UART_Master::BaudRate BAUD_RATE = UART_Master::BaudRate::Baud921600, uint16_t DMA_BUFFER_SIZE = 64>
class RP235x_TOF_Laser_Range_Mini_UART_DMA final : public TOF_Laser_Range_Mini
{
private:
	// readFrame keeps the latest bytes it needs for a frame, and the ring must still hold them. A full ring's
	// read and write positions are the same, so it would read as empty: the ring must be larger than the bytes kept
	static_assert(calcBytesKept(FRAME0_SIZE) < DMA_BUFFER_SIZE, "The DMA buffer must be larger than the bytes kept for a NLink_TOFSense_Frame0");
	static_assert(calcBytesKept(F_SETTING_FRAME0_SIZE) < DMA_BUFFER_SIZE, "The DMA buffer must be larger than the bytes kept for a NLink_TOFSense_F_Setting_Frame0");

	const RP235x_UART_DMA_RX_Ring<DMA_BUFFER_SIZE> _dmaRX; // The DMA ring the device's data is received into
	const RP235x_UART_TX _tx;							// The UART requests are sent to the device on
	bool _started = false;

	uint16_t _readIndex = 0; // Where the next byte will be read from in the DMA buffer

	/// @brief Read where the DMA will write its next byte in the buffer
	/// @return The DMA buffer index
	uint16_t readDMABufferWriteIndex() const
	{
		const uintptr_t writeAddress = dma_channel_hw_addr(_dmaRX.channel)->write_addr;
		const uintptr_t dmaBufferStart = reinterpret_cast<uintptr_t>(_dmaRX.buffer);

		return static_cast<uint16_t>(writeAddress - dmaBufferStart);
	}

protected:
	/// @brief Write bytes to the device, waiting whenever the TX FIFO is full
	/// @param bytes The bytes to write
	/// @param size Number of bytes
	/// @return Always true: every byte is written once the TX FIFO has space
	bool writeBytes(const uint8_t *const bytes, const size_t size) override
	{
		for (size_t index = 0; index < size; ++index)
		{
			while ((*_tx.statusAddress & _tx.fullMask) != 0)
			{
				// Wait for space in the TX FIFO
			}

			*_tx.address = bytes[index];
		}

		return true;
	}

	/// @brief Get how many received bytes are waiting to be read
	/// @return Number of bytes waiting
	size_t bytesAvailable() const override
	{
		return (readDMABufferWriteIndex() + DMA_BUFFER_SIZE - _readIndex) % DMA_BUFFER_SIZE;
	}

	/// @brief Read one received byte
	/// @brief Note: Only call when bytesAvailable() is above 0
	/// @return The byte read
	uint8_t readByte() override
	{
		const uint8_t byte = _dmaRX.buffer[_readIndex];
		_readIndex = (_readIndex + 1) % DMA_BUFFER_SIZE;

		return byte;
	}

	/// @brief Discard all but the latest received bytes, so the next byte read is the oldest one kept. The DMA
	/// @brief overwrites the oldest bytes as it wraps, so the bytes kept are counted back from where it will write next
	/// @param sizeToKeep Number of the latest bytes to keep, less than the DMA buffer's size
	void discardBytes(const size_t sizeToKeep) override
	{
		_readIndex = (readDMABufferWriteIndex() + DMA_BUFFER_SIZE - sizeToKeep) % DMA_BUFFER_SIZE;
	}

	/// @brief Check the setup, and confirm the device responds
	/// @return True if the setup is valid and the device replied
	bool startDevice()
	{
		// The DMA must be receiving and the UART able to send, at the device's baud rate
		if ((_dmaRX.buffer == nullptr) || (_tx.address == nullptr))
			return false;

		if ((_dmaRX.baudRate != static_cast<uint32_t>(BAUD_RATE)) || (_tx.baudRate != static_cast<uint32_t>(BAUD_RATE)))
			return false;

		return initialiseSettings(); // Initialising the settings also confirms the device is running
	}

public:
	/// @param dmaRX The DMA ring to receive the device's data into, set up by RP235x_UART_DMA
	/// @param tx The UART to send requests to the device on, set up by RP235x_PIO_UART or RP235x_UART
	RP235x_TOF_Laser_Range_Mini_UART_DMA(const RP235x_UART_DMA_RX_Ring<DMA_BUFFER_SIZE> &dmaRX,
										 const RP235x_UART_TX &tx) : TOF_Laser_Range_Mini(static_cast<uint32_t>(BAUD_RATE)),
																	   _dmaRX(dmaRX),
																	   _tx(tx) {}

	/// @brief Start the device and confirm it responds
	/// @param retryCount Number of attempts before giving up
	/// @return True if the device started
	bool begin(const uint8_t retryCount = 3) override
	{
		if (!_started)
		{
			uint8_t attempt = 0;
			do
			{
				if (attempt > 0)
					delay(100);

				_started = startDevice();

				++attempt;
			} while (!_started && (attempt < retryCount));
		}

		return _started;
	}
};
