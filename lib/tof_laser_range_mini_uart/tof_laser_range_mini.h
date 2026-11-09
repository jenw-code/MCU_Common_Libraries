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
		Continuous = 0, // Active output: the device sends measurements continuously, at its refresh rate
		Request = 1		// Query output: the device only sends a measurement when requested
	};

protected:
	struct AutoChecker
	{
		repeating_timer_t timer = {};					  // Timer triggering check
		uint32_t minDistanceThreshold = 0;				  // (mm)
		TOF_Laser_Range_Mini_Callback callback = nullptr; // Function to call when minDistanceThreshold met
	};

	OutputMode _outputMode = OutputMode::Continuous; // The device is supplied in continuous (active) output mode
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
	static constexpr uint8_t FRAME0_LENGTH = 16;				// Frame length (bytes)
	static_assert(FRAME0_SUM_CHECK_INDEX < FRAME0_LENGTH, "The Sum Check must be inside the frame");

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
	static constexpr uint8_t READ_FRAME0_LENGTH = 8;			  // Frame length (bytes)
	static_assert(READ_FRAME0_SUM_CHECK_INDEX < READ_FRAME0_LENGTH, "The Sum Check must be inside the frame");

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
	static constexpr uint8_t F_SETTING_FRAME0_LENGTH = 32;				// Frame length (bytes)
	static_assert(F_SETTING_FRAME0_SUM_CHECK_INDEX < F_SETTING_FRAME0_LENGTH, "The Sum Check must be inside the frame");

	// Each version's readFrame relies on the Frame Header coming first, followed by the Function Mark
	static_assert(F_SETTING_FRAME0_FRAME_HEADER_INDEX == 0, "The Frame Header must be the frame's first byte");
	static_assert(F_SETTING_FRAME0_FUNCTION_MARK_INDEX == (F_SETTING_FRAME0_FRAME_HEADER_INDEX + 1), "The Function Mark must follow the Frame Header");

	// NLink_TOFSense_F_Setting_Frame0 mix bits
	static constexpr uint8_t MIX_READ = (1 << 0); // bit0: 0 = write, 1 = read

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

	/// @brief Calculate the time to transmit bytes over the UART, e.g. 32 bytes at 921600 baud gives 347 µs
	/// @param byteCount Number of bytes
	/// @param baudRate The UART's baud rate (bits per second)
	/// @return The time (µs)
	static constexpr uint32_t calcTransmitMicros(const uint8_t byteCount, const uint32_t baudRate)
	{
		constexpr uint32_t BITS_PER_UART_BYTE = 10; // Start bit, 8 data bits and stop bit
		constexpr uint32_t MICROS_PER_SECOND = 1000000;

		return (static_cast<uint32_t>(byteCount) * BITS_PER_UART_BYTE * MICROS_PER_SECOND) / baudRate;
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

	/// @brief Populate a request to read the device's settings, as an NLink_TOFSense_F_Setting_Frame0
	/// @param request Receives the request
	static void populateSettingsRequest(uint8_t (&request)[F_SETTING_FRAME0_LENGTH])
	{
		// HARDWARE CHECK NEEDED: the manual only shows a settings write, so this read request is unconfirmed.
		// Check the device replies to it.

		// Fields the request doesn't set, including the reserved ones, are sent as 0xFF, as in the manual's write example
		constexpr uint8_t NOT_SET_VALUE = 0xFF;

		// Set all values to the "not set" value.
		// We'll then assign the values needed to request the settings.
		memset(request, NOT_SET_VALUE, F_SETTING_FRAME0_LENGTH);

		request[F_SETTING_FRAME0_FRAME_HEADER_INDEX] = F_SETTING_FRAME0_FRAME_HEADER;
		request[F_SETTING_FRAME0_FUNCTION_MARK_INDEX] = F_SETTING_FRAME0_FUNCTION_MARK;
		request[F_SETTING_FRAME0_MIX_INDEX] = MIX_READ;
		request[F_SETTING_FRAME0_ID_INDEX] = FACTORY_ID;
		request[F_SETTING_FRAME0_SUM_CHECK_INDEX] = calcSumCheck(request, F_SETTING_FRAME0_LENGTH, F_SETTING_FRAME0_SUM_CHECK_INDEX);
	}

	/// @brief Populate a request for one measurement, as an NLink_TOFSense_Read_Frame0
	/// @param request Receives the request
	static void populateMeasurementRequest(uint8_t (&request)[READ_FRAME0_LENGTH])
	{
		// Set all values to the reserved value.
		// We'll then assign the values needed to request a measurement.
		memset(request, RESERVED_VALUE, READ_FRAME0_LENGTH);

		request[READ_FRAME0_FRAME_HEADER_INDEX] = FRAME_HEADER;
		request[READ_FRAME0_FUNCTION_MARK_INDEX] = READ_FRAME0_FUNCTION_MARK;
		request[READ_FRAME0_ID_INDEX] = FACTORY_ID;
		request[READ_FRAME0_SUM_CHECK_INDEX] = calcSumCheck(request, READ_FRAME0_LENGTH, READ_FRAME0_SUM_CHECK_INDEX);
	}

	/// @brief Parse the measurement from a valid NLink_TOFSense_Frame0
	/// @param frame The frame
	/// @return The measurement
	static Measurement parseMeasurement(const uint8_t (&frame)[FRAME0_LENGTH])
	{
		Measurement measurement;
		measurement.id = parseField<uint8_t>(frame, FRAME0_ID_INDEX);
		measurement.systemTime = parseField<uint32_t>(frame, FRAME0_SYSTEM_TIME_INDEX);
		measurement.distance = parseField<uint32_t>(frame, FRAME0_DIS_INDEX, 3); // uint24 in the frame
		measurement.distanceStatus = parseField<uint8_t>(frame, FRAME0_DIS_STATUS_INDEX);
		measurement.signalStrength = parseField<uint16_t>(frame, FRAME0_SIGNAL_STRENGTH_INDEX);
		measurement.rangePrecision = parseField<uint8_t>(frame, FRAME0_RANGE_PRECISION_INDEX);

		return measurement;
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
	/// @param byteCount Number of bytes
	/// @return True if every byte was written
	virtual bool writeBytes(const uint8_t *const bytes, const size_t byteCount) = 0;

	/// @brief Get how many received bytes are waiting to be read
	/// @return Number of bytes waiting
	virtual size_t bytesAvailable() const = 0;

	/// @brief Read one received byte
	/// @brief Note: Only call when bytesAvailable() is above 0
	/// @return The byte read
	virtual uint8_t readByte() = 0;

	/// @brief Discard all but the latest received bytes, so the next byte read is the oldest one kept
	/// @param keepCount Number of the latest bytes to keep
	virtual void discardBytes(const size_t keepCount) = 0;

	/// @brief Calculate how many of the latest received bytes readFrame keeps, e.g. a 16 byte frame gives 31 bytes
	/// @param frameLength The frame's length (bytes)
	/// @return Number of bytes kept
	static constexpr size_t calcBytesKept(const uint8_t frameLength)
	{
		// Fewer than two frames' worth of bytes can't hold two complete frames, so
		// once the older bytes are discarded, the first valid frame found is the latest
		return (2 * static_cast<size_t>(frameLength)) - 1;
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
	bool readFrame(uint8_t (&frame)[LENGTH], const uint32_t timeoutMillis)
	{
		static_assert(HEADER_BYTE_ONE != 0, "Buffers start as zeros, so a first header byte of 0 could be found before any byte is received");
		static_assert(LENGTH <= INT8_MAX, "frameStartIndex is an int8_t, so it must be able to hold every index in the buffer");
		static_assert(SUM_CHECK_INDEX < LENGTH, "The Sum Check must be inside the frame");

		// Frames build up if they aren't read as often as they're sent
		discardBytes(calcBytesKept(LENGTH));
		size_t bytesWaiting = bytesAvailable();

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

	/// @brief Send the device a request to read its settings
	/// @return True if the request was sent
	bool requestSettings()
	{
		uint8_t request[F_SETTING_FRAME0_LENGTH];
		populateSettingsRequest(request);

		return writeBytes(request, F_SETTING_FRAME0_LENGTH);
	}

	/// @brief Send the device a request for one measurement, which it sends back as an NLink_TOFSense_Frame0
	/// @return True if the request was sent
	bool requestMeasurement()
	{
		uint8_t request[READ_FRAME0_LENGTH];
		populateMeasurementRequest(request);

		return writeBytes(request, READ_FRAME0_LENGTH);
	}

	/// @brief Only created as part of a derived version
	/// @param baudRate The UART's baud rate (bits per second)
	explicit TOF_Laser_Range_Mini(const uint32_t baudRate) : _baudRate(baudRate) {}

public:
	// Virtual, so either version can be deleted through a base pointer
	virtual ~TOF_Laser_Range_Mini() = default;

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
		const uint8_t bytesTransmitted = static_cast<uint8_t>(F_SETTING_FRAME0_LENGTH + F_SETTING_FRAME0_LENGTH);
		delayMicroseconds(calcTransmitMicros(bytesTransmitted, _baudRate) + DEVICE_PROCESSING_MICROS);

		// Allows for the request and reply at the slowest baud rate, plus any measurement frames sent in between
		constexpr uint32_t RESPONSE_TIMEOUT_MILLIS = 250;

		uint8_t frame[F_SETTING_FRAME0_LENGTH];
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
			const uint8_t bytesTransmitted = static_cast<uint8_t>(READ_FRAME0_LENGTH + FRAME0_LENGTH);
			delayMicroseconds(calcTransmitMicros(bytesTransmitted, _baudRate) + DEVICE_PROCESSING_MICROS);
		}
		else
			return false;

		// How long to wait for a measurement frame, if a valid one hasn't already been received
		constexpr uint32_t FRAME_TIMEOUT_MILLIS = 10;

		uint8_t frame[FRAME0_LENGTH];
		if (!readFrame<FRAME_HEADER, FRAME0_FUNCTION_MARK, FRAME0_SUM_CHECK_INDEX>(frame, FRAME_TIMEOUT_MILLIS))
			return false;

		measurement = parseMeasurement(frame);

		return true;
	}

	/// @brief Set how the device sends measurements
	/// @brief TODO: not yet implemented, so this always returns false and the mode stays Continuous
	/// @param mode The output mode
	/// @return True if the mode was set
	bool virtual setOutputMode(const OutputMode mode)
	{
		// TODO: write the output mode to the device (bit1 of the settings frame's mode byte, by read-modify-write) once
		// the hardware has confirmed the mode byte's layout. Until then, the mode can't be changed
		return false;
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
};
