#pragma once

#include <Arduino.h>

class TOF_Laser_Range_Mini; // Forward declaration

typedef void (*TOF_Laser_Range_Mini_Callback)(TOF_Laser_Range_Mini &sender, const uint32_t distance);

/// @brief Base class for the Waveshare TOF Laser Range Sensor Mini (a Nooploop TOFSense-F2 Mini)
/// @brief Holds Nooploop's NLink TOFSense protocol, shared by the versions that receive it in different ways
class TOF_Laser_Range_Mini
{
public:
	struct Settings final
	{
		uint8_t id = 0;			   // id: Node ID
		uint32_t systemTime = 0;   // system_time: Node system time, since the device powered up (ms)
		uint8_t mode = 0;		   // mode: A bit map covering the output mode, range mode, interface mode and low power
		uint32_t uartBaudRate = 0; // uart_baudrate: UART baud rate
		uint16_t bandStart = 0;	   // band_start: I/O mode threshold start, 0 to 25000 (mm)
		uint16_t bandWidth = 0;	   // band width: I/O mode threshold width, 0 to 25000 (mm)
		uint16_t refreshRate = 0;  // refresh rate: Data refresh rate (Hz)
		uint8_t filterFactor = 0;  // filter factor: 0 to 255 (default 5)
	};

	struct Measurement final
	{
		uint8_t id = 0;				 // id: Node ID
		uint32_t systemTime = 0;	 // System_time: Node system time, since the device powered up (ms)
		uint32_t distance = 0;		 // dis: Distance (mm)
		uint8_t distanceStatus = 0;	 // dis_status: Distance status
		uint16_t signalStrength = 0; // signal_strength: Signal strength
		uint8_t rangePrecision = 0;	 // range_precision: Range precision
	};

	// How the device sends measurements over UART
	enum class OutputMode : uint8_t
	{
		Unknown = 0,	// Not read from the device yet
		Continuous = 1, // Active output: the device sends measurements continuously, at its refresh rate
		Request = 2		// Query output: the device only sends a measurement when requested
	};

	// The refresh rates the device accepts
	enum class RefreshRate : uint16_t
	{
		Unknown = 0, // Not read from the device yet, or not a rate the device lists
		Rate_1Hz = 1,
		Rate_2Hz = 2,
		Rate_5Hz = 5,
		Rate_10Hz = 10,
		Rate_25Hz = 25,
		Rate_50Hz = 50,
		Rate_100Hz = 100,
		Rate_200Hz = 200,
		Rate_350Hz = 350
	};

protected:
	struct AutoChecker
	{
		repeating_timer_t timer = {};					  // Timer triggering check
		uint32_t minDistanceThreshold = 0;				  // (mm)
		TOF_Laser_Range_Mini_Callback callback = nullptr; // Function to call when minDistanceThreshold met
	};

	OutputMode _outputMode = OutputMode::Unknown;	 // Read from the device in startDevice, as the device keeps its mode
	RefreshRate _refreshRate = RefreshRate::Unknown; // Read from the device in startDevice, as the device keeps its rate
	AutoChecker _autoChecker;
	volatile bool _checkDistanceRequired = false;

	const uint32_t _baudRate; // The UART's baud rate (bits per second), for timing the device's replies

	// NLink_TOFSense_Frame0: measurement output frame, sent by the device
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
	static constexpr uint8_t FRAME0_SIZE = 16;					// Frame size (bytes)
	static_assert(FRAME0_SUM_CHECK_INDEX < FRAME0_SIZE, "The Sum Check must be inside the frame");

	// Each version's readFrame relies on the Frame Header coming first, followed by the Function Mark
	static_assert(FRAME0_FRAME_HEADER_INDEX == 0, "The Frame Header must be the frame's first byte");
	static_assert(FRAME0_FUNCTION_MARK_INDEX == (FRAME0_FRAME_HEADER_INDEX + 1), "The Function Mark must follow the Frame Header");

	// NLink_TOFSense_Read_Frame0: query frame, sent to the device to request one NLink_TOFSense_Frame0
	static constexpr uint8_t READ_FRAME0_FRAME_HEADER_INDEX = 0;  // Frame Header (uint8)
	static constexpr uint8_t READ_FRAME0_FUNCTION_MARK_INDEX = 1; // Function Mark (uint8)
	static constexpr uint8_t READ_FRAME0_RESERVED_1_INDEX = 2;	  // reserved (uint16, 2 bytes)
	static constexpr uint8_t READ_FRAME0_ID_INDEX = 4;			  // id (uint8)
	static constexpr uint8_t READ_FRAME0_RESERVED_2_INDEX = 5;	  // reserved (uint16, 2 bytes)
	static constexpr uint8_t READ_FRAME0_SUM_CHECK_INDEX = 7;	  // Sum Check (uint8)
	static constexpr uint8_t READ_FRAME0_SIZE = 8;				  // Frame size (bytes)
	static_assert(READ_FRAME0_SUM_CHECK_INDEX < READ_FRAME0_SIZE, "The Sum Check must be inside the frame");

	// NLink_TOFSense_F_Setting_Frame0: settings frame, sent to the device to write or read its settings, and sent
	// back by the device in reply to a read
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
	static constexpr uint8_t F_SETTING_FRAME0_SIZE = 32;				// Frame size (bytes)
	static_assert(F_SETTING_FRAME0_SUM_CHECK_INDEX < F_SETTING_FRAME0_SIZE, "The Sum Check must be inside the frame");

	// Each version's readFrame relies on the Frame Header coming first, followed by the Function Mark
	static_assert(F_SETTING_FRAME0_FRAME_HEADER_INDEX == 0, "The Frame Header must be the frame's first byte");
	static_assert(F_SETTING_FRAME0_FUNCTION_MARK_INDEX == (F_SETTING_FRAME0_FRAME_HEADER_INDEX + 1), "The Function Mark must follow the Frame Header");

	// NLink_TOFSense_F_Setting_Frame0 mix bits
	static constexpr uint8_t MIX_BIT_MIX = (1 << 0); // mix: 0 = write, 1 = read

	// NLink_TOFSense_F_Setting_Frame0 mode bits
	static constexpr uint8_t MODE_BIT_RESERVED_0 = (1 << 0);  // Reserved
	static constexpr uint8_t MODE_BIT_OUTPUT_MODE = (1 << 1); // output mode: 0 = active, 1 = inquire

	static constexpr uint8_t MODE_BIT_RANGE_MODE_0 = (1 << 2); // Range Mode uses two bits:
	static constexpr uint8_t MODE_BIT_RANGE_MODE_1 = (1 << 3); // 00: Short range, 01: Medium range, 10: Long Range

	static constexpr uint8_t MODE_BIT_INTERFACE_MODE_0 = (1 << 4); // Interface Mode uses two bits:
	static constexpr uint8_t MODE_BIT_INTERFACE_MODE_1 = (1 << 5); // 00: UART, 01: CAN, 10: I/O, 11: I2C

	static constexpr uint8_t MODE_BIT_RESERVED_6 = (1 << 6); // Reserved
	static constexpr uint8_t MODE_BIT_RESERVED_7 = (1 << 7); // Reserved

	// Frame values
	static constexpr uint8_t FRAME_HEADER = 0x57;					// Frame Header of NLink_TOFSense_Frame0 and NLink_TOFSense_Read_Frame0
	static constexpr uint8_t FRAME0_FUNCTION_MARK = 0x00;			// Function Mark of NLink_TOFSense_Frame0
	static constexpr uint8_t READ_FRAME0_FUNCTION_MARK = 0x10;		// Function Mark of NLink_TOFSense_Read_Frame0
	static constexpr uint8_t F_SETTING_FRAME0_FRAME_HEADER = 0x54;	// Frame Header of NLink_TOFSense_F_Setting_Frame0
	static constexpr uint8_t F_SETTING_FRAME0_FUNCTION_MARK = 0x20; // Function Mark of NLink_TOFSense_F_Setting_Frame0
	static constexpr uint8_t RESERVED_VALUE = 0xFF;					// Reserved bytes are sent as 0xFF

	static constexpr uint8_t FACTORY_ID = 0; // Every device is supplied with ID 0

	// An estimate, as the manual doesn't give the device's processing time. Check it on the hardware
	static constexpr uint32_t DEVICE_PROCESSING_MICROS = 2000;

	/// @brief Calculate a frame's Sum Check, from every byte of the frame except the Sum Check itself
	/// @param frame The frame
	/// @param size The frame's size (bytes)
	/// @param sumCheckIndex The index of the frame's Sum Check
	/// @return The Sum Check
	static uint8_t calcSumCheck(const uint8_t *const frame, const uint8_t size, const uint8_t sumCheckIndex)
	{
		uint32_t sum = 0;
		for (uint8_t index = 0; index < size; ++index)
		{
			if (index != sumCheckIndex)
				sum += frame[index];
		}

		// The Sum Check is the lowest byte of the sum
		return static_cast<uint8_t>(sum);
	}

	/// @brief Calculate the time to transmit bytes over the UART, e.g. 32 bytes at 921600 baud gives 347 µs
	/// @param size Number of bytes
	/// @param baudRate The UART's baud rate (bits per second)
	/// @return The time (µs)
	static constexpr uint32_t calcTransmitMicros(const uint8_t size, const uint32_t baudRate)
	{
		constexpr uint32_t BITS_PER_UART_BYTE = 10; // Start bit, 8 data bits and stop bit
		constexpr uint32_t MICROS_PER_SECOND = 1000000;

		return (static_cast<uint32_t>(size) * BITS_PER_UART_BYTE * MICROS_PER_SECOND) / baudRate;
	}

	/// @brief Unpack a little-endian field from a frame
	/// @tparam T The field's type
	/// @param frame The frame
	/// @param fieldIndex The field's index within the frame
	/// @param size The field's size in the frame (bytes), up to the size of T. Defaults to the size of T
	/// @return The field's value
	template <typename T>
	static T unpackField(const uint8_t *const frame, const uint8_t fieldIndex, const uint8_t size = sizeof(T))
	{
		constexpr uint8_t BITS_PER_BYTE = 8;

		T value = 0;
		for (uint8_t byteIndex = 0; byteIndex < size; ++byteIndex)
			value |= static_cast<T>(static_cast<T>(frame[fieldIndex + byteIndex]) << (BITS_PER_BYTE * byteIndex));

		return value;
	}

	/// @brief Pack a little-endian field into a frame
	/// @tparam T The field's type
	/// @param frame The frame
	/// @param fieldIndex The field's index within the frame
	/// @param value The field's value
	/// @param size The field's size in the frame (bytes), up to the size of T. Defaults to the size of T
	template <typename T>
	static void packField(uint8_t *const frame, const uint8_t fieldIndex, const T value, const uint8_t size = sizeof(T))
	{
		constexpr uint8_t BITS_PER_BYTE = 8;

		for (uint8_t byteIndex = 0; byteIndex < size; ++byteIndex)
			frame[fieldIndex + byteIndex] = static_cast<uint8_t>(value >> (BITS_PER_BYTE * byteIndex));
	}

	/// @brief Calculate a settings mode value with an output mode applied, by setting or clearing its output mode bit
	/// @param mode The settings mode value
	/// @param outputMode The output mode. Unknown leaves the mode value unchanged
	/// @return The updated mode value
	static uint8_t applyOutputMode(const uint8_t mode, const OutputMode outputMode)
	{
		if (outputMode == OutputMode::Continuous)
			return static_cast<uint8_t>(mode & ~MODE_BIT_OUTPUT_MODE);
		else if (outputMode == OutputMode::Request)
			return static_cast<uint8_t>(mode | MODE_BIT_OUTPUT_MODE);
		else
			return mode;
	}

	/// @brief Convert a refresh rate read from the device into a RefreshRate
	/// @param rate The refresh rate (Hz)
	/// @return The refresh rate, or Unknown if it isn't one the device lists
	static RefreshRate toRefreshRate(const uint16_t rate)
	{
		switch (rate)
		{
		case static_cast<uint16_t>(RefreshRate::Rate_1Hz):
			return RefreshRate::Rate_1Hz;
		case static_cast<uint16_t>(RefreshRate::Rate_2Hz):
			return RefreshRate::Rate_2Hz;
		case static_cast<uint16_t>(RefreshRate::Rate_5Hz):
			return RefreshRate::Rate_5Hz;
		case static_cast<uint16_t>(RefreshRate::Rate_10Hz):
			return RefreshRate::Rate_10Hz;
		case static_cast<uint16_t>(RefreshRate::Rate_25Hz):
			return RefreshRate::Rate_25Hz;
		case static_cast<uint16_t>(RefreshRate::Rate_50Hz):
			return RefreshRate::Rate_50Hz;
		case static_cast<uint16_t>(RefreshRate::Rate_100Hz):
			return RefreshRate::Rate_100Hz;
		case static_cast<uint16_t>(RefreshRate::Rate_200Hz):
			return RefreshRate::Rate_200Hz;
		case static_cast<uint16_t>(RefreshRate::Rate_350Hz):
			return RefreshRate::Rate_350Hz;
		default:
			return RefreshRate::Unknown;
		}
	}

	/// @brief Populate an NLink_TOFSense_F_Setting_Frame0 with the settings, to write them to the device
	/// @param frame Receives the settings frame
	/// @param settings The settings to write. Every field is written, so they should come from a read
	static void populateSettingsFrame(uint8_t (&frame)[F_SETTING_FRAME0_SIZE], const Settings &settings)
	{
		// Fields the write doesn't set, including the reserved ones, are sent as 0xFF, as in the manual's write example
		constexpr uint8_t NOT_SET_VALUE = 0xFF;

		// Set all values to the "not set" value.
		// We'll then assign the values being written.
		memset(frame, NOT_SET_VALUE, F_SETTING_FRAME0_SIZE);

		frame[F_SETTING_FRAME0_FRAME_HEADER_INDEX] = F_SETTING_FRAME0_FRAME_HEADER;
		frame[F_SETTING_FRAME0_FUNCTION_MARK_INDEX] = F_SETTING_FRAME0_FUNCTION_MARK;
		frame[F_SETTING_FRAME0_MIX_INDEX] = 0; // MIX_BIT_MIX clear: a write
		frame[F_SETTING_FRAME0_ID_INDEX] = settings.id;
		frame[F_SETTING_FRAME0_MODE_INDEX] = settings.mode;
		packField<uint32_t>(frame, F_SETTING_FRAME0_UART_BAUDRATE_INDEX, settings.uartBaudRate, 3); // uint24 in the frame
		packField<uint16_t>(frame, F_SETTING_FRAME0_BAND_START_INDEX, settings.bandStart);
		packField<uint16_t>(frame, F_SETTING_FRAME0_BAND_WIDTH_INDEX, settings.bandWidth);
		packField<uint16_t>(frame, F_SETTING_FRAME0_REFRESH_RATE_INDEX, settings.refreshRate);
		frame[F_SETTING_FRAME0_FILTER_FACTOR_INDEX] = settings.filterFactor;
		frame[F_SETTING_FRAME0_SUM_CHECK_INDEX] = calcSumCheck(frame, F_SETTING_FRAME0_SIZE, F_SETTING_FRAME0_SUM_CHECK_INDEX);
	}

	/// @brief Populate a request to read the device's settings, as an NLink_TOFSense_F_Setting_Frame0
	/// @param request Receives the request
	static void populateSettingsRequestFrame(uint8_t (&request)[F_SETTING_FRAME0_SIZE])
	{
		// HARDWARE CHECK NEEDED: the manual only shows a settings write, so this read request is unconfirmed.
		// Check the device replies to it.

		// Fields the request doesn't set, including the reserved ones, are sent as 0xFF, as in the manual's write example
		constexpr uint8_t NOT_SET_VALUE = 0xFF;

		// Set all values to the "not set" value.
		// We'll then assign the values needed to request the settings.
		memset(request, NOT_SET_VALUE, F_SETTING_FRAME0_SIZE);

		request[F_SETTING_FRAME0_FRAME_HEADER_INDEX] = F_SETTING_FRAME0_FRAME_HEADER;
		request[F_SETTING_FRAME0_FUNCTION_MARK_INDEX] = F_SETTING_FRAME0_FUNCTION_MARK;
		request[F_SETTING_FRAME0_MIX_INDEX] = MIX_BIT_MIX;
		request[F_SETTING_FRAME0_ID_INDEX] = FACTORY_ID;
		request[F_SETTING_FRAME0_SUM_CHECK_INDEX] = calcSumCheck(request, F_SETTING_FRAME0_SIZE, F_SETTING_FRAME0_SUM_CHECK_INDEX);
	}

	/// @brief Populate a request for one measurement, as an NLink_TOFSense_Read_Frame0
	/// @param request Receives the request
	static void populateMeasurementRequestFrame(uint8_t (&request)[READ_FRAME0_SIZE])
	{
		// Set all values to the reserved value.
		// We'll then assign the values needed to request a measurement.
		memset(request, RESERVED_VALUE, READ_FRAME0_SIZE);

		request[READ_FRAME0_FRAME_HEADER_INDEX] = FRAME_HEADER;
		request[READ_FRAME0_FUNCTION_MARK_INDEX] = READ_FRAME0_FUNCTION_MARK;
		request[READ_FRAME0_ID_INDEX] = FACTORY_ID;
		request[READ_FRAME0_SUM_CHECK_INDEX] = calcSumCheck(request, READ_FRAME0_SIZE, READ_FRAME0_SUM_CHECK_INDEX);
	}

	/// @brief Parse the measurement from a valid NLink_TOFSense_Frame0
	/// @param frame The frame
	/// @return The measurement
	static Measurement parseMeasurement(const uint8_t (&frame)[FRAME0_SIZE])
	{
		Measurement measurement;
		measurement.id = unpackField<uint8_t>(frame, FRAME0_ID_INDEX);
		measurement.systemTime = unpackField<uint32_t>(frame, FRAME0_SYSTEM_TIME_INDEX);
		measurement.distance = unpackField<uint32_t>(frame, FRAME0_DIS_INDEX, 3); // uint24 in the frame
		measurement.distanceStatus = unpackField<uint8_t>(frame, FRAME0_DIS_STATUS_INDEX);
		measurement.signalStrength = unpackField<uint16_t>(frame, FRAME0_SIGNAL_STRENGTH_INDEX);
		measurement.rangePrecision = unpackField<uint8_t>(frame, FRAME0_RANGE_PRECISION_INDEX);

		return measurement;
	}

	/// @brief Parse the settings from a valid NLink_TOFSense_F_Setting_Frame0
	/// @param frame The frame
	/// @return The settings
	static Settings parseSettings(const uint8_t (&frame)[F_SETTING_FRAME0_SIZE])
	{
		Settings settings;
		settings.id = unpackField<uint8_t>(frame, F_SETTING_FRAME0_ID_INDEX);
		settings.systemTime = unpackField<uint32_t>(frame, F_SETTING_FRAME0_SYSTEM_TIME_INDEX);
		settings.mode = unpackField<uint8_t>(frame, F_SETTING_FRAME0_MODE_INDEX);
		settings.uartBaudRate = unpackField<uint32_t>(frame, F_SETTING_FRAME0_UART_BAUDRATE_INDEX, 3); // uint24 in the frame
		settings.bandStart = unpackField<uint16_t>(frame, F_SETTING_FRAME0_BAND_START_INDEX);
		settings.bandWidth = unpackField<uint16_t>(frame, F_SETTING_FRAME0_BAND_WIDTH_INDEX);
		settings.refreshRate = unpackField<uint16_t>(frame, F_SETTING_FRAME0_REFRESH_RATE_INDEX);
		settings.filterFactor = unpackField<uint8_t>(frame, F_SETTING_FRAME0_FILTER_FACTOR_INDEX);

		return settings;
	}

	/// @brief Read a measurement, which calls the callback if the distance is below the threshold.
	/// @brief Called by the repeating timer, in interrupt context
	/// @param timer The repeating timer, whose user data is the instance being checked
	/// @return True, to keep the timer repeating
	static bool handleAutoCheckTimer(repeating_timer_t *timer)
	{
		if (timer->user_data != nullptr)
		{
			TOF_Laser_Range_Mini *lidar = static_cast<TOF_Laser_Range_Mini *>(timer->user_data);
			lidar->_checkDistanceRequired = true;
		}

		return true;
	}

	/// @brief Write bytes to the device
	/// @param bytes The bytes to write
	/// @param size Number of bytes
	/// @return True if every byte was written
	virtual bool writeBytes(const uint8_t *const bytes, const size_t size) = 0;

	/// @brief Get how many received bytes are waiting to be read
	/// @return Number of bytes waiting
	virtual size_t bytesAvailable() const = 0;

	/// @brief Read one received byte
	/// @brief Note: Only call when bytesAvailable() is above 0
	/// @return The byte read
	virtual uint8_t readByte() = 0;

	/// @brief Discard all but the latest received bytes, so the next byte read is the oldest one kept
	/// @param sizeToKeep Number of the latest bytes to keep
	virtual void discardBytes(const size_t sizeToKeep) = 0;

	/// @brief Calculate how many of the latest received bytes readFrame keeps, e.g. a 16 byte frame gives 31 bytes
	/// @param frameSize The frame's size (bytes)
	/// @return Number of bytes kept
	static constexpr size_t calcBytesKept(const uint8_t frameSize)
	{
		// Fewer than two frames' worth of bytes can't hold two complete frames, so
		// once the older bytes are discarded, the first valid frame found is the latest
		return (2 * static_cast<size_t>(frameSize)) - 1;
	}

	/// @brief Wait for the latest valid frame, using rolling validation. Every byte received is written to a buffer the
	/// @brief size of one frame, so that if a frame is invalid, the search can carry on from the next header within the buffer
	/// @tparam HEADER_BYTE_ONE The frame's first header byte (its Frame Header)
	/// @tparam HEADER_BYTE_TWO The frame's second header byte (its Function Mark)
	/// @tparam SUM_CHECK_INDEX The index of the frame's Sum Check
	/// @tparam SIZE The frame's size (bytes)
	/// @param frame Receives the frame, in order from its Frame Header. Only changed if a frame is found
	/// @param timeoutMillis How long to wait for more bytes (ms). 0 uses only the bytes already received
	/// @return True if a valid frame was found in time
	template <uint8_t HEADER_BYTE_ONE, uint8_t HEADER_BYTE_TWO, uint8_t SUM_CHECK_INDEX, uint8_t SIZE>
	bool readFrame(uint8_t (&frame)[SIZE], const uint32_t timeoutMillis)
	{
		static_assert(HEADER_BYTE_ONE != 0, "Buffers start as zeros, so a first header byte of 0 could be found before any byte is received");
		static_assert(SIZE <= INT8_MAX, "frameStartIndex is an int8_t, so it must be able to hold every index in the buffer");
		static_assert(SUM_CHECK_INDEX < SIZE, "The Sum Check must be inside the frame");

		// Frames build up if they aren't read as often as they're sent
		discardBytes(calcBytesKept(SIZE));
		size_t bytesWaiting = bytesAvailable();

		uint8_t buffer[SIZE] = {};
		uint8_t bufferIndex = 0;	 // Where the latest byte is
		int8_t frameStartIndex = -1; // Where the frame's first header byte is, or -1 if no header has been found
		const uint32_t startMillis = millis();

		// Every byte recorded is processed, but more are only waited for until the timeout
		while ((bytesWaiting > 0) || ((millis() - startMillis) < timeoutMillis))
		{
			if (bytesWaiting > 0)
			{
				bufferIndex = (bufferIndex + 1) % SIZE;
				buffer[bufferIndex] = readByte();
				--bytesWaiting;

				if (frameStartIndex < 0)
				{
					// Have we found a header?
					const uint8_t previousIndex = (bufferIndex + SIZE - 1) % SIZE;
					if ((buffer[previousIndex] == HEADER_BYTE_ONE) && (buffer[bufferIndex] == HEADER_BYTE_TWO))
					{
						frameStartIndex = static_cast<int8_t>(previousIndex); // Start collecting the frame..
					}
				}
				// The latest byte is the frame's last when the next position is the frame's start
				else if (((bufferIndex + 1) % SIZE) == frameStartIndex)
				{
					// The frame is full. Does it pass the sum check?
					const uint8_t frameStart = static_cast<uint8_t>(frameStartIndex);
					const uint8_t bufferSumCheckIndex = (frameStart + SUM_CHECK_INDEX) % SIZE;
					if (buffer[bufferSumCheckIndex] == calcSumCheck(buffer, SIZE, bufferSumCheckIndex))
					{
						// The frame is valid. It may wrap round the buffer, so it's copied out in order, starting from its header
						for (uint8_t frameIndex = 0; frameIndex < SIZE; ++frameIndex)
							frame[frameIndex] = buffer[(frameStart + frameIndex) % SIZE];

						return true;
					}

					// The frame is invalid, so carry on from the next header within the buffer (if there is one)
					frameStartIndex = -1;
					for (uint8_t offset = 1; offset < (SIZE - 1); ++offset)
					{
						const uint8_t firstIndex = (frameStart + offset) % SIZE;
						const uint8_t secondIndex = (firstIndex + 1) % SIZE;

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

	/// @brief Send the device a request to read its settings
	/// @return True if the request was sent
	bool requestSettings()
	{
		uint8_t request[F_SETTING_FRAME0_SIZE];
		populateSettingsRequestFrame(request);

		return writeBytes(request, F_SETTING_FRAME0_SIZE);
	}

	/// @brief Write the device's settings
	/// @param settings The settings to write. Every field is written, so they should come from a read
	/// @return True if the write was sent
	bool writeSettings(const Settings &settings)
	{
		uint8_t frame[F_SETTING_FRAME0_SIZE];
		populateSettingsFrame(frame, settings);

		if (!writeBytes(frame, F_SETTING_FRAME0_SIZE))
			return false;

		// The write has to be transmitted, and the device has to apply it, before it's sent anything else
		delayMicroseconds(calcTransmitMicros(F_SETTING_FRAME0_SIZE, _baudRate) + DEVICE_PROCESSING_MICROS);

		return true;
	}

	/// @brief Send the device a request for one measurement, which it sends back as an NLink_TOFSense_Frame0
	/// @return True if the request was sent
	bool requestMeasurement()
	{
		uint8_t request[READ_FRAME0_SIZE];
		populateMeasurementRequestFrame(request);

		return writeBytes(request, READ_FRAME0_SIZE);
	}

	/// @brief Initialise the output mode and refresh rate from the device. The device keeps its settings, so
	/// @brief they may not be the ones it was supplied with
	/// @return True if the device's settings were read
	bool initialiseSettings()
	{
		_outputMode = OutputMode::Unknown;
		_refreshRate = RefreshRate::Unknown;

		Settings settings;
		if (!readSettings(settings))
			return false;

		if ((settings.mode & MODE_BIT_OUTPUT_MODE) == 0)
			_outputMode = OutputMode::Continuous;
		else
			_outputMode = OutputMode::Request;

		_refreshRate = toRefreshRate(settings.refreshRate);

		return true;
	}

	/// @brief Only created as part of a derived version
	/// @param baudRate The UART's baud rate (bits per second)
	explicit TOF_Laser_Range_Mini(const uint32_t baudRate) : _baudRate(baudRate) {}

public:
	// Virtual, so either version can be deleted through a base pointer
	virtual ~TOF_Laser_Range_Mini()
	{
		disableAutoChecker();
	}

	/// @brief Start the device and confirm it responds
	/// @param retryCount Number of attempts before giving up
	/// @return True if the device started
	virtual bool begin(const uint8_t retryCount = 3) = 0;

	/// @brief Read the device's settings, from the NLink_TOFSense_F_Setting_Frame0 it sends in reply to a request.
	/// @param settings Receives the settings. Only changed if the read succeeds
	/// @return True if the request was sent and a frame with a valid Sum Check arrived in time
	bool readSettings(Settings &settings)
	{
		if (!requestSettings())
			return false;

		// The request and reply have to be transmitted, and the device has to process the request, so that the
		// reply should be the latest settings frame when it's searched for
		const uint8_t transmittedSize = static_cast<uint8_t>(F_SETTING_FRAME0_SIZE + F_SETTING_FRAME0_SIZE);
		delayMicroseconds(calcTransmitMicros(transmittedSize, _baudRate) + DEVICE_PROCESSING_MICROS);

		// Allows for the request and reply at the slowest baud rate, plus any measurement frames sent in between
		constexpr uint32_t RESPONSE_TIMEOUT_MILLIS = 250;

		uint8_t frame[F_SETTING_FRAME0_SIZE];
		if (!readFrame<F_SETTING_FRAME0_FRAME_HEADER, F_SETTING_FRAME0_FUNCTION_MARK, F_SETTING_FRAME0_SUM_CHECK_INDEX>(frame, RESPONSE_TIMEOUT_MILLIS))
			return false;

		settings = parseSettings(frame);

		return true;
	}

	/// @brief Read the latest valid NLink_TOFSense_Frame0. In Request mode, a measurement is requested first.
	/// @param measurement Receives the measurement. Only changed if a valid frame is found
	/// @return True if a valid frame was found in time
	bool readMeasurement(Measurement &measurement)
	{
		if (_outputMode == OutputMode::Continuous)
		{
			// Nothing to do: the device sends measurements without being asked
		}
		else if (_outputMode == OutputMode::Request)
		{
			if (!requestMeasurement())
				return false;

			// The request and reply have to be transmitted, and the device has to process the request, so that the
			// reply should be the latest measurement frame when it's searched for
			const uint8_t transmittedSize = static_cast<uint8_t>(READ_FRAME0_SIZE + FRAME0_SIZE);
			delayMicroseconds(calcTransmitMicros(transmittedSize, _baudRate) + DEVICE_PROCESSING_MICROS);
		}
		else
			return false;

		// How long to wait for a measurement frame, if a valid one hasn't already been received
		constexpr uint32_t FRAME_TIMEOUT_MILLIS = 10;

		uint8_t frame[FRAME0_SIZE];
		if (!readFrame<FRAME_HEADER, FRAME0_FUNCTION_MARK, FRAME0_SUM_CHECK_INDEX>(frame, FRAME_TIMEOUT_MILLIS))
			return false;

		measurement = parseMeasurement(frame);

		return true;
	}

	/// @brief Set how the device sends measurements. The device's other settings are kept, as they're read first
	/// @brief and written back unchanged
	/// @param mode The output mode
	/// @return True if the write was sent
	bool virtual setOutputMode(const OutputMode mode)
	{
		// The device can only be set to a known mode
		if (mode == OutputMode::Unknown)
			return false;

		Settings settings;
		if (!readSettings(settings))
			return false;

		settings.mode = applyOutputMode(settings.mode, mode);

		if (!writeSettings(settings))
			return false;

		_outputMode = mode;
		return true;
	}

	/// @brief Set how often the device takes a measurement. The device's other settings are kept, as they're
	/// @brief read first and written back unchanged
	/// @param rate The refresh rate
	/// @return True if the write was sent
	bool setRefreshRate(const RefreshRate rate)
	{
		// The device can only be set to a known rate
		if (rate == RefreshRate::Unknown)
			return false;

		Settings settings;
		if (!readSettings(settings))
			return false;

		settings.refreshRate = static_cast<uint16_t>(rate);

		if (!writeSettings(settings))
			return false;

		_refreshRate = rate;
		return true;
	}

	/// @brief Set up the device in one call, with a single settings write. The defaults are the settings the
	/// @brief device is supplied with
	/// @tparam OUTPUT_MODE How the device sends measurements (default Continuous)
	/// @tparam REFRESH_RATE How often the device takes a measurement (default 50 Hz)
	/// @return True if the write was sent
	template <OutputMode OUTPUT_MODE = OutputMode::Continuous,
			  RefreshRate REFRESH_RATE = RefreshRate::Rate_50Hz>
	bool quickSetup()
	{
		static_assert(OUTPUT_MODE != OutputMode::Unknown, "The output mode must be a known mode");
		static_assert(REFRESH_RATE != RefreshRate::Unknown, "The refresh rate must be a known rate");

		Settings settings;
		if (!readSettings(settings))
			return false;

		settings.mode = applyOutputMode(settings.mode, OUTPUT_MODE);
		settings.refreshRate = static_cast<uint16_t>(REFRESH_RATE);

		if (!writeSettings(settings))
			return false;

		_outputMode = OUTPUT_MODE;
		_refreshRate = REFRESH_RATE;
		return true;
	}

	void disableAutoChecker()
	{
		cancel_repeating_timer(&_autoChecker.timer);
		_autoChecker.callback = nullptr;
	}

	/// @brief Check the distance periodically, calling the callback whenever it's below the threshold
	/// @brief This is not fully-automatic. Timer interrupt will only set flag. CheckDistance must be
	/// @brief called for any checks to be run. Make sure CheckDistance is called as often as the
	/// @brief auto-check interval requires
	/// @param intervalMillis Time between checks (ms)
	/// @param minDistanceThreshold The distance below which the callback is called (mm)
	/// @param callback Called with the distance
	/// @return True if the checks started
	bool enableAutoChecker(const uint16_t intervalMillis, const uint32_t minDistanceThreshold, const TOF_Laser_Range_Mini_Callback callback)
	{
		if (callback == nullptr)
			return false;

		disableAutoChecker();

		_autoChecker.minDistanceThreshold = minDistanceThreshold;
		_autoChecker.callback = callback;

		// A negative interval is measured from the start of one check to the start of the next,
		// so the checks keep a fixed rate
		return add_repeating_timer_ms(-static_cast<int32_t>(intervalMillis), handleAutoCheckTimer, this, &_autoChecker.timer);
	}

	void checkDistance()
	{
		// A measurement can only be read once the device's output mode is known
		if (_outputMode == OutputMode::Unknown)
			return;

		if (_checkDistanceRequired)
		{
			Measurement measurement;
			if (readMeasurement(measurement) &&
				(measurement.distance < _autoChecker.minDistanceThreshold) &&
				(_autoChecker.callback != nullptr))
			{
				_autoChecker.callback(*this, measurement.distance);
			}

			_checkDistanceRequired = false;
		}
	}

	/// @brief Get how the device sends measurements
	/// @return The output mode
	OutputMode getOutputMode() const
	{
		return _outputMode;
	}

	/// @brief Get how often the device takes a measurement
	/// @return The refresh rate
	RefreshRate getRefreshRate() const
	{
		return _refreshRate;
	}
};