#pragma once

#include <Arduino.h>
#include "mock_serial_base.h"

/// @brief A serial port for testing, which records the bytes written and serves the mock data as the bytes received
class Mock_Serial final : public Mock_Serial_Base, public HardwareSerial
{
private:
    static constexpr size_t MAX_WRITTEN_SIZE = 64;
    static constexpr size_t MAX_READ_SIZE = 256;

    size_t _readPosition = 0; // Position in the data of the next byte to read

    uint8_t _written[MAX_WRITTEN_SIZE] = {};
    size_t _writtenSize = 0;

    uint8_t _read[MAX_READ_SIZE] = {};
    size_t _readSize = 0;

    /// @brief Print bytes to Serial, as hex on one line
    /// @param label Text printed before the bytes
    /// @param bytes The bytes to print
    /// @param size Number of bytes to print
    static void printBytes(const char *const label, const uint8_t *const bytes, const size_t size)
    {
        Serial.print(label);
        for (size_t index = 0; index < size; ++index)
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
        printBytes("Written:", _written, _writtenSize);
    }

    /// @brief Print the bytes read from the port to Serial, as hex on one line (up to MAX_READ_SIZE)
    void printRead() const
    {
        printBytes("Read:", _read, _readSize);
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
        return static_cast<int>(calcBytesArrived() - _readPosition);
    }

    /// @return The next byte received, without reading it, or -1 if there's none
    int peek() override
    {
        if (available() == 0)
            return -1;

        return getByte(_readPosition);
    }

    /// @return The next byte received, or -1 if there's none
    int read() override
    {
        if (available() == 0)
            return -1;

        const uint8_t value = getByte(_readPosition);
        ++_readPosition;

        if (_readSize < MAX_READ_SIZE)
        {
            _read[_readSize] = value;
            ++_readSize;
        }

        return value;
    }

    void flush() override {}

    /// @param value The byte to record
    /// @return 1 if the byte was recorded, 0 if the record is full
    size_t write(uint8_t value) override
    {
        if (_writtenSize >= MAX_WRITTEN_SIZE)
            return 0;

        _written[_writtenSize] = value;
        ++_writtenSize;
        return 1;
    }

    using Print::write;

    operator bool() override
    {
        return true;
    }
};
