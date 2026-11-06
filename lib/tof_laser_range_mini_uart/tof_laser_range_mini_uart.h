#pragma once

#include <Arduino.h>
#include "uart_master.h"

/// @brief Class for the Waveshare TOF Laser Range Sensor Mini (a Nooploop TOFSense-F2 Mini), over UART
/// @brief Uses Nooploop's NLink TOFSense protocol
class TOF_Laser_Range_Mini_UART final : public UART_Master
{
public:
	struct Measurement final
	{
		uint8_t id = 0;				 // id: Module ID
		uint32_t systemTime = 0;	 // System_time: Time since the module powered up (ms)
		uint32_t distance = 0;		 // dis: Distance (mm)
		uint8_t distanceStatus = 0;	 // dis_status: 1 = Valid measurement distance, 0 = Invalid
		uint16_t signalStrength = 0; // signal_strength: Strength of the return signal (higher is stronger)
		uint8_t rangePrecision = 0;	 // range_precision: Ranging accuracy (cm, smaller is better). Not valid on the Mini
	};

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

private:
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

	/// @brief Calculate a frame's Sum Check, from every byte of the frame except the Sum Check itself
	/// @param frame The frame
	/// @param length The frame's length (bytes)
	/// @param sumCheckIndex The index of the frame's Sum Check
	/// @return The Sum Check
	static uint8_t calcSumCheck(const uint8_t *const frame, const uint8_t length, const uint8_t sumCheckIndex);

	/// @brief Wait for a valid frame, using rolling validation. Every byte received is written to a buffer one frame
	/// @brief long, so that if a frame is invalid, the search can carry on from the next header within the buffer
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

		uint8_t buffer[LENGTH] = {};
		uint8_t bufferIndex = 0;	 // Where the latest byte is
		int8_t frameStartIndex = -1; // Where the frame's first header byte is, or -1 if no header has been found
		const uint32_t startMillis = millis();
		size_t bytesWaiting = bytesAvailable();

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
	bool requestSettings();

	/// @brief Read the module's settings, from the NLink_TOFSense_F_Setting_Frame0 it sends in reply to a request
	/// @param settings Receives the settings. Only changed if the read succeeds
	/// @return True if the request was sent and a frame with a valid Sum Check arrived in time
	bool readSettings(Settings &settings);

protected:
	/// @brief Start the module and confirm it responds
	/// @return True if successful
	bool startSerial() override;

public:
	/// @param serial The UART the module is connected to (hardware or PIO serial)
	/// @param baudRate The module's baud rate. The default is the rate the module is supplied with
	explicit TOF_Laser_Range_Mini_UART(HardwareSerial &serial, const BaudRate baudRate = BaudRate::Baud921600);

	/// @brief Read the distance from the next valid NLink_TOFSense_Frame0, which the module sends in active output mode
	/// @param distance Receives the distance (mm). Only changed if a frame is found
	/// @param timeoutMillis How long to wait for more bytes (ms). 0 uses only the bytes already received
	/// @return True if a valid frame was found in time
	bool readDistance(uint32_t &distance, const uint32_t timeoutMillis);
};
