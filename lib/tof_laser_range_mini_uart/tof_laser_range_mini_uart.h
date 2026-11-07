#pragma once

#include <Arduino.h>
#include "uart_master.h"

/// @brief Class for the Waveshare TOF Laser Range Sensor Mini (a Nooploop TOFSense-F2 Mini), over UART
/// @brief Uses Nooploop's NLink TOFSense protocol
/// @tparam BAUD_RATE The module's baud rate. The default is the rate the module is supplied with
template <UART_Master::BaudRate BAUD_RATE = UART_Master::BaudRate::Baud921600>
class TOF_Laser_Range_Mini_UART final : public UART_Master
{
public:
	struct Settings final
	{
		uint8_t id = 0;			   // id: Node ID
		uint32_t systemTime = 0;   // system_time: Node system time, since the module powered up (ms)
		uint8_t mode = 0;		   // mode: A bit map covering the output mode, range mode, interface mode and low power
		uint32_t uartBaudRate = 0; // uart_baudrate: UART baud rate
		uint16_t bandStart = 0;	   // band_start: I/O mode threshold start, 0 to 25000 (mm)
		uint16_t bandWidth = 0;	   // band width: I/O mode threshold width, 0 to 25000 (mm)
		uint16_t refreshRate = 0;  // refresh rate: Data refresh rate (Hz)
		uint8_t filterFactor = 0;  // filter factor: 0 to 255 (default 5)
	};

	// How the module sends measurements over UART
	enum class OutputMode : uint8_t
	{
		Continuous = 0, // Active output: the module sends measurements continuously, at its refresh rate
		Request = 1		// Query output: the module only sends a measurement when requested
	};

private:
	OutputMode _outputMode = OutputMode::Continuous; // The module is supplied in continuous (active) output mode

	// NLink_TOFSense_Frame0: measurement output frame, sent by the module
	static constexpr uint8_t FRAME0_FRAME_HEADER_INDEX = 0;		// Frame Header (uint8)
	static constexpr uint8_t FRAME0_FUNCTION_MARK_INDEX = 1;	// Function Mark (uint8)
	static constexpr uint8_t FRAME0_RESERVED_INDEX = 2;			// reserved (uint8)
	static constexpr uint8_t FRAME0_ID_INDEX = 3;				// id (uint8)
	static constexpr uint8_t FRAME0_SYSTEM_TIME_INDEX = 4;		// System_time (uint32, little-endian, 4 bytes)
	static constexpr uint8_t FRAME0_DIS_INDEX = 8;				// dis (uint24, little-endian, 3 bytes)
	static constexpr uint8_t FRAME0_DIS_STATUS_INDEX = 11;		// dis_status (uint8)
	static constexpr uint8_t FRAME0_SIGNAL_STRENGTH_INDEX = 12; // signal_strength (uint16, little-endian, 2 bytes)
	static constexpr uint8_t FRAME0_RANGE_PRECISION_INDEX = 14; // range_precision (uint8)
	static constexpr uint8_t FRAME0_SUM_CHECK_INDEX = 15;		// Sum Check (uint8)
	static constexpr uint8_t FRAME0_LENGTH = 16;				// Frame length (bytes)
	static_assert(FRAME0_SUM_CHECK_INDEX < FRAME0_LENGTH, "The Sum Check must be inside the frame");

	// NLink_TOFSense_Read_Frame0: query frame, sent to the module to request one NLink_TOFSense_Frame0
	static constexpr uint8_t READ_FRAME0_FRAME_HEADER_INDEX = 0;  // Frame Header (uint8)
	static constexpr uint8_t READ_FRAME0_FUNCTION_MARK_INDEX = 1; // Function Mark (uint8)
	static constexpr uint8_t READ_FRAME0_RESERVED_1_INDEX = 2;	  // reserved (uint16, 2 bytes)
	static constexpr uint8_t READ_FRAME0_ID_INDEX = 4;			  // id (uint8)
	static constexpr uint8_t READ_FRAME0_RESERVED_2_INDEX = 5;	  // reserved (uint16, 2 bytes)
	static constexpr uint8_t READ_FRAME0_SUM_CHECK_INDEX = 7;	  // Sum Check (uint8)
	static constexpr uint8_t READ_FRAME0_LENGTH = 8;			  // Frame length (bytes)
	static_assert(READ_FRAME0_SUM_CHECK_INDEX < READ_FRAME0_LENGTH, "The Sum Check must be inside the frame");

	// NLink_TOFSense_F_Setting_Frame0: settings frame, sent to the module to write or read its settings, and sent
	// back by the module in reply to a read
	static constexpr uint8_t F_SETTING_FRAME0_FRAME_HEADER_INDEX = 0;	// Frame Header (uint8)
	static constexpr uint8_t F_SETTING_FRAME0_FUNCTION_MARK_INDEX = 1;	// Function Mark (uint8)
	static constexpr uint8_t F_SETTING_FRAME0_MIX_INDEX = 2;			// mix (uint8)
	static constexpr uint8_t F_SETTING_FRAME0_RESERVED_1_INDEX = 3;		// reserved (1 byte)
	static constexpr uint8_t F_SETTING_FRAME0_ID_INDEX = 4;				// id (uint8)
	static constexpr uint8_t F_SETTING_FRAME0_SYSTEM_TIME_INDEX = 5;	// system_time (uint32, little-endian, 4 bytes)
	static constexpr uint8_t F_SETTING_FRAME0_MODE_INDEX = 9;			// mode (uint8)
	static constexpr uint8_t F_SETTING_FRAME0_RESERVED_2_INDEX = 10;	// reserved (2 bytes)
	static constexpr uint8_t F_SETTING_FRAME0_UART_BAUDRATE_INDEX = 12; // uart_baudrate (uint24, little-endian, 3 bytes)
	static constexpr uint8_t F_SETTING_FRAME0_RESERVED_3_INDEX = 15;	// reserved (4 bytes)
	static constexpr uint8_t F_SETTING_FRAME0_BAND_START_INDEX = 19;	// band_start (uint16, little-endian, 2 bytes)
	static constexpr uint8_t F_SETTING_FRAME0_BAND_WIDTH_INDEX = 21;	// band width (uint16, little-endian, 2 bytes)
	static constexpr uint8_t F_SETTING_FRAME0_RESERVED_4_INDEX = 23;	// reserved (1 byte)
	static constexpr uint8_t F_SETTING_FRAME0_REFRESH_RATE_INDEX = 24;	// refresh rate (uint16, little-endian, 2 bytes)
	static constexpr uint8_t F_SETTING_FRAME0_FILTER_FACTOR_INDEX = 26; // filter factor (uint8)
	static constexpr uint8_t F_SETTING_FRAME0_RESERVED_5_INDEX = 27;	// reserved (4 bytes)
	static constexpr uint8_t F_SETTING_FRAME0_SUM_CHECK_INDEX = 31;		// Sum Check (uint8)
	static constexpr uint8_t F_SETTING_FRAME0_LENGTH = 32;				// Frame length (bytes)
	static_assert(F_SETTING_FRAME0_SUM_CHECK_INDEX < F_SETTING_FRAME0_LENGTH, "The Sum Check must be inside the frame");

	// NLink_TOFSense_F_Setting_Frame0 mix bits
	static constexpr uint8_t MIX_READ = (1 << 0); // bit0: 0 = write, 1 = read

	// Frame values
	static constexpr uint8_t FRAME_HEADER = 0x57;					// Frame Header of NLink_TOFSense_Frame0 and NLink_TOFSense_Read_Frame0
	static constexpr uint8_t FRAME0_FUNCTION_MARK = 0x00;			// Function Mark of NLink_TOFSense_Frame0
	static constexpr uint8_t READ_FRAME0_FUNCTION_MARK = 0x10;		// Function Mark of NLink_TOFSense_Read_Frame0
	static constexpr uint8_t F_SETTING_FRAME0_FRAME_HEADER = 0x54;	// Frame Header of NLink_TOFSense_F_Setting_Frame0
	static constexpr uint8_t F_SETTING_FRAME0_FUNCTION_MARK = 0x20; // Function Mark of NLink_TOFSense_F_Setting_Frame0
	static constexpr uint8_t RESERVED_VALUE = 0xFF;					// Reserved bytes are sent as 0xFF

	static constexpr uint8_t FACTORY_ID = 0; // Every module is supplied with ID 0

	/// @brief Calculate the time to transmit bytes over the UART at BAUD_RATE
	/// @param byteCount Number of bytes
	/// @return The time (µs)
	static constexpr uint32_t calcTransmitMicros(const uint8_t byteCount)
	{
		constexpr uint32_t BITS_PER_UART_BYTE = 10; // Start bit, 8 data bits and stop bit
		constexpr uint32_t MICROS_PER_SECOND = 1000000;

		return (static_cast<uint32_t>(byteCount) * BITS_PER_UART_BYTE * MICROS_PER_SECOND) / static_cast<uint32_t>(BAUD_RATE);
	}

	/// @brief Calculate a frame's Sum Check, from every byte of the frame except the Sum Check itself
	/// @param frame The frame
	/// @param length The frame's length (bytes)
	/// @param sumCheckIndex The index of the frame's Sum Check
	/// @return The Sum Check
	static uint8_t calcSumCheck(const uint8_t *const frame, const uint8_t length, const uint8_t sumCheckIndex)
	{
		uint32_t sum = 0;
		for (uint8_t index = 0; index < length; ++index)
		{
			if (index != sumCheckIndex)
				sum += frame[index];
		}

		// The Sum Check is the lowest byte of the sum
		return static_cast<uint8_t>(sum);
	}

	/// @brief Wait for the latest valid frame, using rolling validation. Every byte received is written to a buffer one
	/// @brief frame long, so that if a frame is invalid, the search can carry on from the next header within the buffer
	/// @tparam HEADER_BYTE_ONE The frame's first header byte (its Frame Header)
	/// @tparam HEADER_BYTE_TWO The frame's second header byte (its Function Mark)
	/// @tparam SUM_CHECK_INDEX The index of the frame's Sum Check
	/// @tparam LENGTH The frame's length (bytes)
	/// @param frame Receives the frame, in order from its Frame Header. Only changed if a frame is found
	/// @param timeoutMillis How long to wait for more bytes (ms). 0 uses only the bytes already received
	/// @return True if a valid frame was found in time
	template <uint8_t HEADER_BYTE_ONE, uint8_t HEADER_BYTE_TWO, uint8_t SUM_CHECK_INDEX, uint8_t LENGTH>
	bool waitForFrame(uint8_t (&frame)[LENGTH], const uint32_t timeoutMillis)
	{
		static_assert(HEADER_BYTE_ONE != 0, "The buffer starts as zeros, so a first header byte of 0 could be found before any byte is received");
		static_assert(LENGTH <= INT8_MAX, "frameStartIndex is an int8_t, so it must be able to hold every index in the buffer");
		static_assert(SUM_CHECK_INDEX < LENGTH, "The Sum Check must be inside the frame");

		// Frames build up if they aren't read as often as they're sent. Fewer than two frames' worth of bytes can't
		// hold two complete frames, so once the older bytes are discarded, the first valid frame found is the latest
		constexpr size_t MAX_BYTES_KEPT = (2 * static_cast<size_t>(LENGTH)) - 1;

		size_t bytesWaiting = bytesAvailable();
		if (bytesWaiting > MAX_BYTES_KEPT)
		{
			const size_t bytesToDiscard = bytesWaiting - MAX_BYTES_KEPT;
			for (size_t index = 0; index < bytesToDiscard; ++index)
				readByte();

			bytesWaiting = MAX_BYTES_KEPT;
		}

		uint8_t buffer[LENGTH] = {};
		uint8_t bufferIndex = 0;	 // Where the latest byte is
		int8_t frameStartIndex = -1; // Where the frame's first header byte is, or -1 if no header has been found
		const uint32_t startMillis = millis();

		// Every byte recorded is processed, but more are only waited for until the timeout
		while ((bytesWaiting > 0) || ((millis() - startMillis) < timeoutMillis))
		{
			if (bytesWaiting > 0)
			{
				bufferIndex = (bufferIndex + 1) % LENGTH;
				buffer[bufferIndex] = readByte();
				--bytesWaiting;

				if (frameStartIndex < 0)
				{
					// Have we found a header?
					const uint8_t previousIndex = (bufferIndex + LENGTH - 1) % LENGTH;
					if ((buffer[previousIndex] == HEADER_BYTE_ONE) && (buffer[bufferIndex] == HEADER_BYTE_TWO))
					{
						frameStartIndex = static_cast<int8_t>(previousIndex); // Start collecting the frame..
					}
				}
				// The latest byte is the frame's last when the next position is the frame's start
				else if (((bufferIndex + 1) % LENGTH) == frameStartIndex)
				{
					// The frame is full. Does it pass the sum check?
					const uint8_t frameStart = static_cast<uint8_t>(frameStartIndex);
					const uint8_t bufferSumCheckIndex = (frameStart + SUM_CHECK_INDEX) % LENGTH;
					if (buffer[bufferSumCheckIndex] == calcSumCheck(buffer, LENGTH, bufferSumCheckIndex))
					{
						// The frame is valid. It may wrap round the buffer, so it's copied out in order, starting from its header
						for (uint8_t frameIndex = 0; frameIndex < LENGTH; ++frameIndex)
							frame[frameIndex] = buffer[(frameStart + frameIndex) % LENGTH];

						return true;
					}

					// The frame is invalid, so carry on from the next header within the buffer (if there is one)
					frameStartIndex = -1;
					for (uint8_t offset = 1; offset < (LENGTH - 1); ++offset)
					{
						const uint8_t firstIndex = (frameStart + offset) % LENGTH;
						const uint8_t secondIndex = (firstIndex + 1) % LENGTH;

						if ((buffer[firstIndex] == HEADER_BYTE_ONE) && (buffer[secondIndex] == HEADER_BYTE_TWO))
						{
							frameStartIndex = static_cast<int8_t>(firstIndex);
							break;
						}
					}
				}
			}
			else
				bytesWaiting = bytesAvailable();
		}

		return false;
	}

	/// @brief Parse a little-endian field from a frame
	/// @tparam T The field's type
	/// @param frame The frame
	/// @param fieldIndex The field's index within the frame
	/// @param byteCount The field's size in the frame (bytes), up to the size of T. Defaults to the size of T
	/// @return The field's value
	template <typename T>
	static T parseField(const uint8_t *const frame, const uint8_t fieldIndex, const uint8_t byteCount = sizeof(T))
	{
		constexpr uint8_t BITS_PER_BYTE = 8;

		T value = 0;
		for (uint8_t byteIndex = 0; byteIndex < byteCount; ++byteIndex)
			value |= static_cast<T>(static_cast<T>(frame[fieldIndex + byteIndex]) << (BITS_PER_BYTE * byteIndex));

		return value;
	}

	/// @brief Parse the settings from a valid NLink_TOFSense_F_Setting_Frame0
	/// @param frame The frame
	/// @return The settings
	static Settings parseSettings(const uint8_t (&frame)[F_SETTING_FRAME0_LENGTH])
	{
		Settings settings;
		settings.id = parseField<uint8_t>(frame, F_SETTING_FRAME0_ID_INDEX);
		settings.systemTime = parseField<uint32_t>(frame, F_SETTING_FRAME0_SYSTEM_TIME_INDEX);
		settings.mode = parseField<uint8_t>(frame, F_SETTING_FRAME0_MODE_INDEX);
		settings.uartBaudRate = parseField<uint32_t>(frame, F_SETTING_FRAME0_UART_BAUDRATE_INDEX, 3); // uint24 in the frame
		settings.bandStart = parseField<uint16_t>(frame, F_SETTING_FRAME0_BAND_START_INDEX);
		settings.bandWidth = parseField<uint16_t>(frame, F_SETTING_FRAME0_BAND_WIDTH_INDEX);
		settings.refreshRate = parseField<uint16_t>(frame, F_SETTING_FRAME0_REFRESH_RATE_INDEX);
		settings.filterFactor = parseField<uint8_t>(frame, F_SETTING_FRAME0_FILTER_FACTOR_INDEX);

		return settings;
	}

	/// @brief Send the module a request to read its settings
	/// @return True if the request was sent
	bool requestSettings()
	{
		// HARDWARE CHECK NEEDED: the manual only shows a settings write, so this read request is unconfirmed.
		// Check the module replies to it.

		// Fields the request doesn't set, including the reserved ones, are sent as 0xFF, as in the manual's write example
		constexpr uint8_t NOT_SET_VALUE = 0xFF;

		// Create request buffer and set all values to the "not set" value.
		// We'll then assign the values needed to request the settings.
		uint8_t request[F_SETTING_FRAME0_LENGTH];
		memset(request, NOT_SET_VALUE, F_SETTING_FRAME0_LENGTH);

		request[F_SETTING_FRAME0_FRAME_HEADER_INDEX] = F_SETTING_FRAME0_FRAME_HEADER;
		request[F_SETTING_FRAME0_FUNCTION_MARK_INDEX] = F_SETTING_FRAME0_FUNCTION_MARK;
		request[F_SETTING_FRAME0_MIX_INDEX] = MIX_READ;
		request[F_SETTING_FRAME0_ID_INDEX] = FACTORY_ID;
		request[F_SETTING_FRAME0_SUM_CHECK_INDEX] = calcSumCheck(request, F_SETTING_FRAME0_LENGTH, F_SETTING_FRAME0_SUM_CHECK_INDEX);

		return writeBytes(request, F_SETTING_FRAME0_LENGTH);
	}

	/// @brief Read the module's settings, from the NLink_TOFSense_F_Setting_Frame0 it sends in reply to a request
	/// @param settings Receives the settings. Only changed if the read succeeds
	/// @return True if the request was sent and a frame with a valid Sum Check arrived in time
	bool readSettings(Settings &settings)
	{
		// waitForFrame relies on the Frame Header coming first, followed by the Function Mark
		static_assert(F_SETTING_FRAME0_FRAME_HEADER_INDEX == 0, "The Frame Header must be the frame's first byte");
		static_assert(F_SETTING_FRAME0_FUNCTION_MARK_INDEX == (F_SETTING_FRAME0_FRAME_HEADER_INDEX + 1), "The Function Mark must follow the Frame Header");

		if (!requestSettings())
			return false;

		// Allows for the request and reply at the slowest baud rate, plus any measurement frames sent in between
		constexpr uint32_t RESPONSE_TIMEOUT_MILLIS = 250;

		uint8_t frame[F_SETTING_FRAME0_LENGTH];
		if (!waitForFrame<F_SETTING_FRAME0_FRAME_HEADER, F_SETTING_FRAME0_FUNCTION_MARK, F_SETTING_FRAME0_SUM_CHECK_INDEX>(frame, RESPONSE_TIMEOUT_MILLIS))
			return false;

		settings = parseSettings(frame);
		return true;
	}

	/// @brief Send the module a request for one measurement, which it sends back as an NLink_TOFSense_Frame0
	/// @return True if the request was sent
	bool requestMeasurement()
	{
		// Create request buffer and set all values to the reserved value.
		// We'll then assign the values needed to request a measurement.
		uint8_t request[READ_FRAME0_LENGTH];
		memset(request, RESERVED_VALUE, READ_FRAME0_LENGTH);

		request[READ_FRAME0_FRAME_HEADER_INDEX] = FRAME_HEADER;
		request[READ_FRAME0_FUNCTION_MARK_INDEX] = READ_FRAME0_FUNCTION_MARK;
		request[READ_FRAME0_ID_INDEX] = FACTORY_ID;
		request[READ_FRAME0_SUM_CHECK_INDEX] = calcSumCheck(request, READ_FRAME0_LENGTH, READ_FRAME0_SUM_CHECK_INDEX);

		return writeBytes(request, READ_FRAME0_LENGTH);
	}

protected:
	/// @brief Start the module and confirm it responds
	/// @return True if successful
	bool startSerial() override
	{
		Settings settings;

		return UART_Master::startSerial() &&
			   readSettings(settings); // Read settings - this confirms the device is running
	}

public:
	/// @param serial The UART the module is connected to (hardware or PIO serial)
	explicit TOF_Laser_Range_Mini_UART(HardwareSerial &serial) : UART_Master(serial, BAUD_RATE) {}

	/// @brief Set how the module sends measurements
	/// @brief TODO: not yet implemented, so this always returns false and the mode stays Continuous
	/// @param mode The output mode
	/// @return True if the mode was set
	bool setOutputMode(const OutputMode mode)
	{
		// TODO: write the output mode to the module (bit1 of the settings frame's mode byte, by read-modify-write) once
		// the hardware has confirmed the mode byte's layout. Until then, the mode can't be changed
		return false;
	}

	/// @brief Read the distance from the latest valid NLink_TOFSense_Frame0. In Request mode, a measurement is
	/// @brief requested first
	/// @param distance Receives the distance (mm). Only changed if a frame is found
	/// @param timeoutMillis How long to wait for more bytes (ms). 0 uses only the bytes already received
	/// @return True if a valid frame was found in time
	bool readDistance(uint32_t &distance, const uint32_t timeoutMillis = 10)
	{
		// waitForFrame relies on the Frame Header coming first, followed by the Function Mark
		static_assert(FRAME0_FRAME_HEADER_INDEX == 0, "The Frame Header must be the frame's first byte");
		static_assert(FRAME0_FUNCTION_MARK_INDEX == (FRAME0_FRAME_HEADER_INDEX + 1), "The Function Mark must follow the Frame Header");

		if (_outputMode == OutputMode::Continuous)
		{
			// Nothing to do: the module sends measurements without being asked
		}
		else if (_outputMode == OutputMode::Request)
		{
			if (!requestMeasurement())
				return false;

			// An estimate, as the manual doesn't give the module's processing time. Check it on the hardware
			constexpr uint32_t MODULE_PROCESSING_MICROS = 2000;

			// The request and reply have to be transmitted, and the module has to process the request, so that the
			// reply should be waiting before waitForFrame's timeout starts
			const uint8_t bytesTransmitted = static_cast<uint8_t>(READ_FRAME0_LENGTH + FRAME0_LENGTH);
			delayMicroseconds(calcTransmitMicros(bytesTransmitted) + MODULE_PROCESSING_MICROS);
		}
		else
			return false;

		uint8_t frame[FRAME0_LENGTH];
		if (!waitForFrame<FRAME_HEADER, FRAME0_FUNCTION_MARK, FRAME0_SUM_CHECK_INDEX>(frame, timeoutMillis))
			return false;

		distance = parseField<uint32_t>(frame, FRAME0_DIS_INDEX, 3); // uint24 in the frame
		return true;
	}
};
