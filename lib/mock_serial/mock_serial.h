#pragma once

#include <Arduino.h>

/// @brief A serial port for testing, which records the bytes written and serves a stream of received bytes.
/// @brief The stream is added as arrays, each starting with a pause (ms) before its data is received
class MockSerial final : public HardwareSerial
{
private:
    static constexpr uint8_t MAX_DATA_ARRAYS = 16;
    static constexpr size_t MAX_WRITTEN_LENGTH = 64;
    static constexpr size_t MAX_READ_LENGTH = 256;

    // Each data array is its pause (ms), followed by its data
    static constexpr size_t PAUSE_INDEX = 0;
    static constexpr size_t DATA_START_INDEX = 1;

    struct DataArray final
    {
        const uint8_t *data;
        size_t length;
    };

    DataArray _dataArrays[MAX_DATA_ARRAYS] = {};
    uint8_t _dataArrayCount = 0;

    uint8_t _currentArray = 0;           // The data array being received
    size_t _readIndex = DATA_START_INDEX; // The next byte to read in the current data array
    uint32_t _pauseStartMillis = 0;      // When the current data array's pause started
    bool _streamStarted = false;

    uint8_t _written[MAX_WRITTEN_LENGTH] = {};
    size_t _writtenLength = 0;

    uint8_t _read[MAX_READ_LENGTH] = {};
    size_t _readLength = 0;

    /// @brief Print bytes to Serial, as hex on one line
    /// @param label Text printed before the bytes
    /// @param bytes The bytes to print
    /// @param length Number of bytes to print
    static void printBytes(const char *const label, const uint8_t *const bytes, const size_t length)
    {
        Serial.print(label);
        for (size_t index = 0; index < length; ++index)
        {
            Serial.print(' ');
            if (bytes[index] < 0x10)
                Serial.print('0');
            Serial.print(bytes[index], HEX);
        }
        Serial.println();
    }

    /// @brief Move on to the next data array
    /// @param pauseStartMillis When the next data array's pause starts (ms)
    void moveToNextArray(const uint32_t pauseStartMillis)
    {
        ++_currentArray;
        _readIndex = DATA_START_INDEX;
        _pauseStartMillis = pauseStartMillis;
    }

public:
    /// @brief Add a data array to the end of the stream
    /// @tparam LENGTH The array's length (bytes), including its pause
    /// @param data The pause before the data is received (ms, 0 to 255), followed by the data. Must stay valid
    /// while the port is used
    /// @return True if added, false if MAX_DATA_ARRAYS have already been added
    template <size_t LENGTH>
    bool addData(const uint8_t (&data)[LENGTH])
    {
        static_assert(LENGTH > PAUSE_INDEX, "A data array must start with its pause (ms)");

        if (_dataArrayCount >= MAX_DATA_ARRAYS)
            return false;

        _dataArrays[_dataArrayCount] = {.data = data, .length = LENGTH};
        ++_dataArrayCount;
        return true;
    }

    /// @brief Print the bytes written to the port to Serial, as hex on one line
    void printWritten() const
    {
        printBytes("Written:", _written, _writtenLength);
    }

    /// @brief Print the bytes read from the port to Serial, as hex on one line (up to MAX_READ_LENGTH)
    void printRead() const
    {
        printBytes("Read:", _read, _readLength);
    }

    /// @brief Start the stream. Only the first call starts it, as a real device sends regardless of the port restarting
    /// @param baudRate Ignored, as there's no real port
    void begin(unsigned long baudRate) override
    {
        if (!_streamStarted)
        {
            _pauseStartMillis = millis();
            _streamStarted = true;
        }
    }

    /// @brief Start the stream. Only the first call starts it, as a real device sends regardless of the port restarting
    /// @param baudRate Ignored, as there's no real port
    /// @param config Ignored, as there's no real port
    void begin(unsigned long baudRate, uint16_t config) override
    {
        begin(baudRate);
    }

    void end() override {}

    /// @return Number of bytes received but not yet read: the current data array's, plus those of any following data
    /// @return arrays with no pause. 0 during the current data array's pause
    int available() override
    {
        if (!_streamStarted)
            return 0;

        while (_currentArray < _dataArrayCount)
        {
            const DataArray &array = _dataArrays[_currentArray];
            const uint32_t pauseMillis = static_cast<uint32_t>(array.data[PAUSE_INDEX]);

            const bool pausing = (millis() - _pauseStartMillis) < pauseMillis;
            if (pausing)
                return 0;

            const size_t unread = array.length - _readIndex;
            if (unread > 0)
            {
                // Following data arrays with no pause arrived with this one, so their data is waiting too
                size_t waiting = unread;
                for (uint8_t next = _currentArray + 1; next < _dataArrayCount; ++next)
                {
                    const DataArray &nextArray = _dataArrays[next];
                    if (nextArray.data[PAUSE_INDEX] != 0)
                        break;

                    waiting += nextArray.length - DATA_START_INDEX;
                }

                return static_cast<int>(waiting);
            }

            // A data array with only a pause: the next pause starts when this one ends
            moveToNextArray(_pauseStartMillis + pauseMillis);
        }

        return 0;
    }

    /// @return The next byte received, without reading it, or -1 if there's none
    int peek() override
    {
        if (available() == 0)
            return -1;

        return _dataArrays[_currentArray].data[_readIndex];
    }

    /// @return The next byte received, or -1 if there's none
    int read() override
    {
        if (available() == 0)
            return -1;

        const DataArray &array = _dataArrays[_currentArray];
        const uint8_t value = array.data[_readIndex];
        ++_readIndex;

        if (_readLength < MAX_READ_LENGTH)
        {
            _read[_readLength] = value;
            ++_readLength;
        }

        // The next pause starts when the current data array has been read
        if (_readIndex == array.length)
            moveToNextArray(millis());

        return value;
    }

    void flush() override {}

    /// @param value The byte to record
    /// @return 1 if the byte was recorded, 0 if the record is full
    size_t write(uint8_t value) override
    {
        if (_writtenLength >= MAX_WRITTEN_LENGTH)
            return 0;

        _written[_writtenLength] = value;
        ++_writtenLength;
        return 1;
    }

    using Print::write;

    operator bool() override
    {
        return true;
    }
};
