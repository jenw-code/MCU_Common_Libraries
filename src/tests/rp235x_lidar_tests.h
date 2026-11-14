#pragma once

// The lidar test. Run setupLidar() once, then checkLidar() from the loop, from the one .cpp that includes this
// header (it defines the test's global objects)

#include <Arduino.h>
#include "common.debug.h"

#include "tof_laser_range_mini_uart.h"
#include "rp235x_tof_laser_range_mini_uart_dma.h"

RP235x_TOF_Laser_Range_Mini_UART_DMA<UART_Master::BaudRate::Baud921600> *lidar1 = nullptr;

void lidarCallback(TOF_Laser_Range_Mini & /* sender */, const uint32_t distance)
{
	logValue("MIN DISTANCE CALLBACK. DISTANCE = ", distance);
}

constexpr uint16_t LIDAR_BUFFER_SIZE = 64;
constexpr uint16_t LIDAR_DMA_RING_SIZE = 64;

// The DMA wraps its write address on a boundary of the ring's size, so the ring is aligned to it
alignas(LIDAR_DMA_RING_SIZE) volatile uint8_t lidar1_DMARing[LIDAR_DMA_RING_SIZE] = {};

bool setupLidar()
{
	constexpr RP235x_PIO_UART_Config RX_CONFIG = {
		.pioIndex = 0,
		.stateMachine = 0,
		.pin = D3,
		.baudRate = UART_Master::BaudRate::Baud921600,
	};

	constexpr RP235x_PIO_UART_Config TX_CONFIG = {
		.pioIndex = 0,
		.stateMachine = 1,
		.pin = D2,
		.baudRate = UART_Master::BaudRate::Baud921600,
	};

	constexpr RP235x_UART_DMA_RX_Ring_Config<LIDAR_BUFFER_SIZE> LIDAR1_DMA_CONFIG = {
		.channel = 0,
	};

	RP235x_UART_RX rx = {};
	RP235x_UART_TX tx = {};
	RP235x_UART_DMA_RX_Ring<LIDAR_BUFFER_SIZE> dmaRX = {};

	if (RP235x_PIO_UART::setup_RX(RX_CONFIG, rx) &&
		RP235x_PIO_UART::setup_TX(TX_CONFIG, tx) &&
		RP235x_UART_DMA::setup_RX(LIDAR1_DMA_CONFIG, rx, lidar1_DMARing, dmaRX))
	{
		lidar1 = new RP235x_TOF_Laser_Range_Mini_UART_DMA<UART_Master::BaudRate::Baud921600, LIDAR_DMA_RING_SIZE>(dmaRX, tx);
		if (lidar1->begin())
		{
			logStr("setupLidar Success");

			lidar1->enableAutoChecker(200, 100, lidarCallback);
			return true;
		}
	}

	logStr("setupLidar Fail");

	return false;
}

/// @brief Run the lidar's distance check. Call it from the loop
void checkLidar()
{
	if (lidar1 != nullptr)
	{
		lidar1->checkDistance();
	}

	// Well under the auto-checker's 200 ms interval, so checkDistance runs each time its timer asks
	delay(20);
}
