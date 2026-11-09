#pragma once

#include <Arduino.h>
#include <hardware/dma.h>
#include <pico/time.h>
#include "tof_laser_range_mini.h"
#include "rp2xxx_uart.h"

/// @brief Class for the Waveshare TOF Laser Range Sensor Mini (a Nooploop TOFSense-F2 Mini), over UART, received by DMA
/// @brief Uses Nooploop's NLink TOFSense protocol. DMA copies every byte received into a buffer, so no core time
/// @brief is spent per byte, and the latest frame can be read at any time, including from an interrupt
/// @tparam BAUD_RATE The device's baud rate. The default is the rate the device is supplied with
template <UART_Master::BaudRate BAUD_RATE = UART_Master::BaudRate::Baud921600>
class RP235x_TOF_Laser_Range_Mini_UART_DMA final : public TOF_Laser_Range_Mini
{
private:
	// The DMA buffer wraps on its own size, so its size is a power of 2
	static constexpr uint16_t DMA_BUFFER_SIZE = 64;

	int _dmaChannel = -1; // -1 until begin() claims a channel

	// The DMA wraps its write address on a boundary of the buffer's size, so the buffer must be aligned to it
	alignas(DMA_BUFFER_SIZE) volatile uint8_t _dmaBuffer[DMA_BUFFER_SIZE] = {};

	// readFrame keeps the latest bytes it needs for a frame, and the DMA buffer must still hold them. A full buffer's
	// read and write positions are the same, so it would read as empty: the buffer must be larger than the bytes kept
	static_assert(calcBytesKept(FRAME0_LENGTH) < DMA_BUFFER_SIZE, "The DMA buffer must be larger than the bytes kept for a NLink_TOFSense_Frame0");
	static_assert(calcBytesKept(F_SETTING_FRAME0_LENGTH) < DMA_BUFFER_SIZE, "The DMA buffer must be larger than the bytes kept for a NLink_TOFSense_F_Setting_Frame0");

	uint16_t _readIndex = 0; // Where the next byte will be read from in the DMA buffer

	const RP2xxx_UART_RX_TX_t _uart; // The UART the device is connected to
	bool _started = false;

	/// @brief Read where the DMA will write its next byte in the buffer
	/// @return The DMA buffer index
	uint16_t readDMABufferWriteIndex() const
	{
		const uintptr_t writeAddress = dma_channel_hw_addr(static_cast<uint>(_dmaChannel))->write_addr;
		const uintptr_t dmaBufferStart = reinterpret_cast<uintptr_t>(_dmaBuffer);

		return static_cast<uint16_t>(writeAddress - dmaBufferStart);
	}

protected:
	/// @brief Write bytes to the device, waiting whenever the TX FIFO is full
	/// @param bytes The bytes to write
	/// @param byteCount Number of bytes
	/// @return Always true: every byte is written once the TX FIFO has space
	bool writeBytes(const uint8_t *const bytes, const size_t byteCount) override
	{
		for (size_t index = 0; index < byteCount; ++index)
		{
			while ((*_uart.tx.statusAddress & _uart.tx.fullMask) != 0)
			{
				// Wait for space in the TX FIFO
			}

			*_uart.tx.address = bytes[index];
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
		const uint8_t byte = _dmaBuffer[_readIndex];
		_readIndex = (_readIndex + 1) % DMA_BUFFER_SIZE;

		return byte;
	}

	/// @brief Discard all but the latest received bytes, so the next byte read is the oldest one kept. The DMA
	/// @brief overwrites the oldest bytes as it wraps, so the bytes kept are counted back from where it will write next
	/// @param keepCount Number of the latest bytes to keep, less than the DMA buffer's size
	void discardBytes(const size_t keepCount) override
	{
		_readIndex = (readDMABufferWriteIndex() + DMA_BUFFER_SIZE - keepCount) % DMA_BUFFER_SIZE;
	}

	/// @brief Start the DMA copying every byte received into the DMA buffer, and confirm the device responds
	/// @return True if started and the device replied
	bool startDevice()
	{
		// The UART must be able to receive and send, at the device's baud rate
		if ((_uart.rx.address == nullptr) || (_uart.tx.address == nullptr))
			return false;

		if ((_uart.rx.baudRate != static_cast<uint32_t>(BAUD_RATE)) || (_uart.tx.baudRate != static_cast<uint32_t>(BAUD_RATE)))
			return false;

		Settings settings;

		return RP235x_UART_DMA::startRing(_uart, _dmaBuffer, _dmaChannel) &&
			   readSettings(settings); // Read settings - this confirms the device is running
	}

public:
	/// @param uart The UART the device is connected to, created by RP2xxx_PIO_UART or RP2xxx_UART
	explicit RP235x_TOF_Laser_Range_Mini_UART_DMA(const RP2xxx_UART_RX_TX_t &uart)
		: TOF_Laser_Range_Mini(static_cast<uint32_t>(BAUD_RATE)), _uart(uart) {}

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
