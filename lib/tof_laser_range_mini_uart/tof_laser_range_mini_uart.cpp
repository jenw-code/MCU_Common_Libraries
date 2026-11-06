#include "tof_laser_range_mini_uart.h"

uint8_t TOF_Laser_Range_Mini_UART::calcSumCheck(const uint8_t *const frame, const uint8_t length, const uint8_t sumCheckIndex)
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

bool TOF_Laser_Range_Mini_UART::requestSettings()
{
    // HARDWARE CHECK NEEDED: the manual only gives an example of a write. This read request (a write frame with
    // mix set to read) is the natural reading of its table. Confirm the module replies to it.

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

bool TOF_Laser_Range_Mini_UART::readSettings(Settings &settings)
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

bool TOF_Laser_Range_Mini_UART::startSerial()
{
    Settings settings;

    return UART_Master::startSerial() &&
           readSettings(settings); // Read settings - this confirms the device is running
}

TOF_Laser_Range_Mini_UART::TOF_Laser_Range_Mini_UART(HardwareSerial &serial, const BaudRate baudRate) : UART_Master(serial, baudRate) {}

bool TOF_Laser_Range_Mini_UART::readDistance(uint32_t &distance, const uint32_t timeoutMillis)
{
    // waitForFrame relies on the Frame Header coming first, followed by the Function Mark
    static_assert(FRAME0_FRAME_HEADER_INDEX == 0, "The Frame Header must be the frame's first byte");
    static_assert(FRAME0_FUNCTION_MARK_INDEX == (FRAME0_FRAME_HEADER_INDEX + 1), "The Function Mark must follow the Frame Header");

    uint8_t frame[FRAME0_LENGTH];
    if (!waitForFrame<FRAME_HEADER, FRAME0_FUNCTION_MARK, FRAME0_SUM_CHECK_INDEX>(frame, timeoutMillis))
        return false;

    distance = parseField<uint32_t>(frame, FRAME0_DIS_INDEX, 3); // uint24 in the frame
    return true;
}
