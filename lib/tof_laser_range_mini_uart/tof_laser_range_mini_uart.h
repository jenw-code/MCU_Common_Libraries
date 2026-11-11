#pragma once

#include <Arduino.h>
#include "tof_laser_range_mini.h"
#include "uart_master.h"

/// @brief Class for the Waveshare TOF Laser Range Sensor Mini (a Nooploop TOFSense-F2 Mini), over UART
/// @brief Uses Nooploop's NLink TOFSense protocol
/// @tparam BAUD_RATE The device's baud rate. The default is the rate the device is supplied with
template <UART_Master::BaudRate BAUD_RATE = UART_Master::BaudRate::Baud921600>
class TOF_Laser_Range_Mini_UART final : public TOF_Laser_Range_Mini, public UART_Master
{
protected:
	/// @brief Write bytes to the device
	/// @param bytes The bytes to write
	/// @param size Number of bytes
	/// @return True if every byte was written
	bool writeBytes(const uint8_t *const bytes, const size_t size) override
	{
		return UART_Master::writeBytes(bytes, size);
	}

	/// @brief Get how many received bytes are waiting to be read
	/// @return Number of bytes waiting
	size_t bytesAvailable() const override
	{
		return UART_Master::bytesAvailable();
	}

	/// @brief Read one received byte
	/// @brief Note: Only call when bytesAvailable() is above 0
	/// @return The byte read
	uint8_t readByte() override
	{
		return UART_Master::readByte();
	}

	/// @brief Discard all but the latest received bytes, so the next byte read is the oldest one kept
	/// @param sizeToKeep Number of the latest bytes to keep
	void discardBytes(const size_t sizeToKeep) override
	{
		const size_t bytesWaiting = UART_Master::bytesAvailable();
		if (bytesWaiting > sizeToKeep)
		{
			const size_t bytesToDiscard = bytesWaiting - sizeToKeep;
			for (size_t index = 0; index < bytesToDiscard; ++index)
				UART_Master::readByte();
		}
	}

	/// @brief Start the device and confirm it responds
	/// @return True if successful
	bool startSerial() override
	{
		Settings settings;

		return UART_Master::startSerial() &&
			   readSettings(settings); // Read settings - this confirms the device is running
	}

public:
	/// @param serial The UART the device is connected to (hardware or PIO serial)
	explicit TOF_Laser_Range_Mini_UART(HardwareSerial &serial)
		: TOF_Laser_Range_Mini(static_cast<uint32_t>(BAUD_RATE)), UART_Master(serial, BAUD_RATE) {}

	/// @brief Start the device and confirm it responds
	/// @param retryCount Number of attempts before giving up
	/// @return True if the device started
	bool begin(const uint8_t retryCount = 3) override
	{
		return UART_Master::begin(retryCount);
	}
};
