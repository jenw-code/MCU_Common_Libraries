#include <Arduino.h>
#include <pins_arduino.h>
#include "common.debug.h"
#include "common.battery.h"
#include "common.battery.info.h"
#include "common.errors.h"
#include "common.versioning.h"
#include "config.h" // Includes pin mapping

#include "bmm150_i2c.h"
#include "mpu6500_i2c.h"
#include "mock_serial.h"
#include "tof_laser_range_mini_uart.h"
#include "rp235x_tof_laser_range_mini_uart_dma.h"

// Each array is the bytes received. The pause before each one is given to addData
const uint8_t SETTINGS_REPLY[] = {
    0x54, 0x20,             // Frame Header, Function Mark
    0x00,                   // mix
    0xFF,                   // reserved
    0x00,                   // id
    0xE8, 0x03, 0x00, 0x00, // system_time: 1000 ms
    0x00,                   // mode
    0xFF, 0xFF,             // reserved
    0x00, 0x10, 0x0E,       // uart_baudrate: 921600
    0xFF, 0xFF, 0xFF, 0xFF, // reserved
    0x00, 0x00,             // band_start
    0x00, 0x00,             // band width
    0xFF,                   // reserved
    0x32, 0x00,             // refresh rate: 50 Hz
    0x05,                   // filter factor
    0xFF, 0xFF, 0xFF, 0xFF, // reserved
    0xA8                    // Sum Check
};

const uint8_t SETTINGS_REPLY2[] = {
    0x54, 0x20,             // Frame Header, Function Mark
    0x00,                   // mix
    0xFF,                   // reserved
    0xFF, 0xFF, 0xFF, 0xFF, // reserved
    0x1E,                   // Sum Check
    0x54, 0x20,             // Frame Header, Function Mark
    0x00,                   // mix
    0xFF,                   // reserved
    0x00,                   // id
    0xE8, 0x03, 0x00, 0x00, // system_time: 1000 ms
    0x00,                   // mode
    0xFF, 0xFF,             // reserved
    0x00, 0x10, 0x0E,       // uart_baudrate: 921600
    0xFF, 0xFF, 0xFF, 0xFF, // reserved
    0x00, 0x00,             // band_start
    0x00, 0x00,             // band width
    0xFF,                   // reserved
    0x32, 0x00,             // refresh rate: 50 Hz
    0x05,                   // filter factor
    0xFF, 0xFF, 0xFF, 0xFF, // reserved
    0xA8                    // Sum Check
};

// A truncated frame (its first 20 bytes), then a valid frame. The real header is swallowed by the truncated
// frame's 32 bytes, so the rescan must find it inside the buffer
const uint8_t SETTINGS_REPLY3[] = {
    0x54, 0x20,             // Frame Header, Function Mark (truncated frame)
    0x00,                   // mix
    0xFF,                   // reserved
    0x00,                   // id
    0xE8, 0x03, 0x00, 0x00, // system_time: 1000 ms
    0x00,                   // mode
    0xFF, 0xFF,             // reserved
    0x00, 0x10, 0x0E,       // uart_baudrate: 921600
    0xFF, 0xFF, 0xFF, 0xFF, // reserved
    0x00,                   // band_start (first byte only, where the frame is cut off)
    0x54, 0x20,             // Frame Header, Function Mark (valid frame)
    0x00,                   // mix
    0xFF,                   // reserved
    0x00,                   // id
    0xE8, 0x03, 0x00, 0x00, // system_time: 1000 ms
    0x00,                   // mode
    0xFF, 0xFF,             // reserved
    0x00, 0x10, 0x0E,       // uart_baudrate: 921600
    0xFF, 0xFF, 0xFF, 0xFF, // reserved
    0x00, 0x00,             // band_start
    0x00, 0x00,             // band width
    0xFF,                   // reserved
    0x32, 0x00,             // refresh rate: 50 Hz
    0x05,                   // filter factor
    0xFF, 0xFF, 0xFF, 0xFF, // reserved
    0xA8                    // Sum Check
};

// An invalid frame, then junk, then a valid frame. The real header arrives after the invalid frame, so the
// rescan finds nothing and the search must pick it up from the new bytes
const uint8_t SETTINGS_REPLY4[] = {
    0x54, 0x20,                   // Frame Header, Function Mark (invalid frame)
    0x00,                         // mix
    0xFF,                         // reserved
    0x00,                         // id
    0xE8, 0x03, 0x00, 0x00,       // system_time: 1000 ms
    0x00,                         // mode
    0xFF, 0xFF,                   // reserved
    0x00, 0x10, 0x0E,             // uart_baudrate: 921600
    0xFF, 0xFF, 0xFF, 0xFF,       // reserved
    0x00, 0x00,                   // band_start
    0x00, 0x00,                   // band width
    0xFF,                         // reserved
    0x32, 0x00,                   // refresh rate: 50 Hz
    0x05,                         // filter factor
    0xFF, 0xFF, 0xFF, 0xFF,       // reserved
    0x00,                         // Sum Check: deliberately wrong (valid is 0xA8)
    0x11, 0x22, 0x33, 0x44, 0x55, // Junk
    0x54, 0x20,                   // Frame Header, Function Mark (valid frame)
    0x00,                         // mix
    0xFF,                         // reserved
    0x00,                         // id
    0xE8, 0x03, 0x00, 0x00,       // system_time: 1000 ms
    0x00,                         // mode
    0xFF, 0xFF,                   // reserved
    0x00, 0x10, 0x0E,             // uart_baudrate: 921600
    0xFF, 0xFF, 0xFF, 0xFF,       // reserved
    0x00, 0x00,                   // band_start
    0x00, 0x00,                   // band width
    0xFF,                         // reserved
    0x32, 0x00,                   // refresh rate: 50 Hz
    0x05,                         // filter factor
    0xFF, 0xFF, 0xFF, 0xFF,       // reserved
    0xA8                          // Sum Check
};

// A clean measurement frame (the manual's example). Expected distance: 2221 mm
const uint8_t MEASUREMENT_REPLY[] = {
    0x57, 0x00,             // Frame Header, Function Mark
    0xFF,                   // reserved
    0x00,                   // id
    0x9E, 0x8F, 0x00, 0x00, // System_time: 36766 ms
    0xAD, 0x08, 0x00,       // dis: 2221 mm
    0x00,                   // dis_status
    0x03, 0x00,             // signal_strength: 3
    0x06,                   // range_precision
    0x41                    // Sum Check
};

// Starts partway through a frame whose signal strength is 87 (57 00), so the first header found is that
// coincidence. Two whole frames follow, each also with a signal strength of 87. Expected distance: 2221 mm
const uint8_t MEASUREMENT_REPLY2[] = {
    0x57, 0x00,             // signal_strength: 87 (the end of a frame, so the stream starts with a coincidental header)
    0x06,                   // range_precision
    0x95,                   // Sum Check
    0x57, 0x00,             // Frame Header, Function Mark
    0xFF,                   // reserved
    0x00,                   // id
    0x9E, 0x8F, 0x00, 0x00, // System_time: 36766 ms
    0xAD, 0x08, 0x00,       // dis: 2221 mm
    0x00,                   // dis_status
    0x57, 0x00,             // signal_strength: 87
    0x06,                   // range_precision
    0x95,                   // Sum Check
    0x57, 0x00,             // Frame Header, Function Mark
    0xFF,                   // reserved
    0x00,                   // id
    0x9E, 0x8F, 0x00, 0x00, // System_time: 36766 ms
    0xAD, 0x08, 0x00,       // dis: 2221 mm
    0x00,                   // dis_status
    0x57, 0x00,             // signal_strength: 87
    0x06,                   // range_precision
    0x95                    // Sum Check
};

// A truncated frame (its first 10 bytes, 2221 mm), then a valid frame at a different distance, to show the
// valid frame is the one parsed. Expected distance: 1000 mm
const uint8_t MEASUREMENT_REPLY3[] = {
    0x57, 0x00,             // Frame Header, Function Mark (truncated frame)
    0xFF,                   // reserved
    0x00,                   // id
    0x9E, 0x8F, 0x00, 0x00, // System_time: 36766 ms
    0xAD, 0x08,             // dis (first 2 bytes only, where the frame is cut off)
    0x57, 0x00,             // Frame Header, Function Mark (valid frame)
    0xFF,                   // reserved
    0x00,                   // id
    0x9E, 0x8F, 0x00, 0x00, // System_time: 36766 ms
    0xE8, 0x03, 0x00,       // dis: 1000 mm
    0x00,                   // dis_status
    0x03, 0x00,             // signal_strength: 3
    0x06,                   // range_precision
    0x77                    // Sum Check
};

BMM150_I2C bmm;
MPU6500_I2C mpu;

constexpr RP2xxx_PIO_UART_Config_t TEST_LIDAR_UART_RX_CONFIG = {.pioIndex = 0, .stateMachine = 0, .pin = D8};
constexpr RP2xxx_PIO_UART_Config_t TEST_LIDAR_UART_TX_CONFIG = {.pioIndex = 0, .stateMachine = 1, .pin = D9};

Mock_Serial *testLidarSerial = nullptr;
TOF_Laser_Range_Mini_UART<> *testLidar = nullptr;

// The lidar's DMA version: its PIO UART and DMA allocation, and the ring buffer the DMA fills
constexpr uint16_t LIDAR_DMA_RING_SIZE = 64;
constexpr RP2xxx_PIO_UART_Config_t LIDAR1_UART_RX_CONFIG = {.pioIndex = 0, .stateMachine = 2, .pin = D4};
constexpr RP2xxx_PIO_UART_Config_t LIDAR1_UART_TX_CONFIG = {.pioIndex = 0, .stateMachine = 3, .pin = D5};
constexpr RP235x_UART_DMA_Config_t<LIDAR_DMA_RING_SIZE> LIDAR1_DMA_CONFIG = {.channel = 0};

// The DMA wraps its write address on a boundary of the ring's size, so the ring is aligned to it
alignas(LIDAR_DMA_RING_SIZE) volatile uint8_t lidar1_DMARing[LIDAR_DMA_RING_SIZE] = {};

RP235x_TOF_Laser_Range_Mini_UART_DMA<UART_Master::BaudRate::Baud921600, LIDAR_DMA_RING_SIZE> *lidar1 = nullptr;

void lidarCallback(TOF_Laser_Range_Mini &sender, const uint32_t distance)
{
    logValue("MIN DISTANCE CALLBACK. DISTANCE = ", distance);
}

/// @brief Set up the lidar's DMA version: its PIO UART, then the DMA receiving from it, then the lidar itself
/// @return True if everything was set up and the lidar responded
bool setupLidar1()
{
    RP2xxx_UART_RX_t rx = {};
    RP2xxx_UART_TX_t tx = {};
    RP235x_UART_DMA_RX_t<LIDAR_DMA_RING_SIZE> dmaRX = {};

    if (RP2xxx_PIO_UART::setup_RX(LIDAR1_UART_RX_CONFIG, UART_Master::BaudRate::Baud921600, rx) &&
        RP2xxx_PIO_UART::setup_TX(LIDAR1_UART_TX_CONFIG, UART_Master::BaudRate::Baud921600, tx) &&
        RP235x_UART_DMA::setup_RX(LIDAR1_DMA_CONFIG, rx, lidar1_DMARing, dmaRX))
    {
        lidar1 = new RP235x_TOF_Laser_Range_Mini_UART_DMA<UART_Master::BaudRate::Baud921600, LIDAR_DMA_RING_SIZE>(dmaRX, tx);
        if (lidar1->begin())
        {
            logStr("setupLidar1 Success");
            return true;
        }
    }

    logStr("setupLidar1 Fail");

    return false;
}

bool setupTestLidar()
{
    // TOF_Laser_Range_Mini_UART
    testLidarSerial = new Mock_Serial;

    testLidarSerial->addData(100, SETTINGS_REPLY4);
    testLidarSerial->addData(0, MEASUREMENT_REPLY);
    testLidarSerial->addData(0, MEASUREMENT_REPLY2);
    testLidarSerial->addData(255, MEASUREMENT_REPLY3);

    RP2xxx_UART_RX_t rx = {};
    RP2xxx_UART_TX_t tx = {};

    if (RP2xxx_PIO_UART::setup_RX(TEST_LIDAR_UART_RX_CONFIG, UART_Master::BaudRate::Baud921600, rx) &&
        RP2xxx_PIO_UART::setup_TX(TEST_LIDAR_UART_TX_CONFIG, UART_Master::BaudRate::Baud921600, tx))
    {
        testLidar = new TOF_Laser_Range_Mini_UART<>(*testLidarSerial);
        if (testLidar->begin())
        {
            logStr("setupTestLidar Success");

            testLidar->enableAutoChecker(100, 1001, lidarCallback);

            testLidarSerial->printWritten();
            testLidarSerial->printRead();
        }

        return true;
    }

    logStr("setupTestLidar Fail");

    return false;
}

void setup()
{
#if APP_DEBUG
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    logStr("Starting..");
#endif

    /*
        mpu.begin();
        mpu.quickSetup();
        bmm.begin();
    */

    setupLidar1();

    setupTestLidar();
}

//---------------------------------------- LOOP ----------------------------------------------------

void loop()
{
    if (testLidar != nullptr)
        testLidar->checkDistance();

    delay(10);
}