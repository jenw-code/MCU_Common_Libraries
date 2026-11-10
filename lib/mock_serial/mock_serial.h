#pragma once

#include <Arduino.h>
#include "mock_serial_base.h"

/// @brief A serial port for testing, which records the bytes written and serves the mock data as the bytes received
class Mock_Serial final : public Mock_Serial_Base, public HardwareSerial
{
private:
    static constexpr size_t MAX_WRITTEN_LENGTH = 64;
    static constexpr size_t MAX_READ_LENGTH = 256;

    size_t _readCount = 0; // Number of bytes of the data read so far

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

public:
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

    /// @brief Start the data arriving. Only the first call starts it, as a real device sends regardless of the
    /// @brief port restarting
    /// @param baudRate Ignored, as there's no real port
    void begin(unsigned long /* baudRate */) override
    {
        start();
    }

    /// @brief Start the data arriving. Only the first call starts it, as a real device sends regardless of the
    /// @brief port restarting
    /// @param baudRate Ignored, as there's no real port
    /// @param config Ignored, as there's no real port
    void begin(unsigned long /* baudRate */, uint16_t /* config */) override
    {
        start();
    }

    void end() override {}

    /// @return Number of bytes that have arrived but not yet been read
    int available() override
    {
        return static_cast<int>(calcBytesArrived() - _readCount);
    }

    /// @return The next byte received, without reading it, or -1 if there's none
    int peek() override
    {
        if (available() == 0)
            return -1;

        return getByte(_readCount);
    }

    /// @return The next byte received, or -1 if there's none
    int read() override
    {
        if (available() == 0)
            return -1;

        const uint8_t value = getByte(_readCount);
        ++_readCount;

        if (_readLength < MAX_READ_LENGTH)
        {
            _read[_readLength] = value;
            ++_readLength;
        }

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
