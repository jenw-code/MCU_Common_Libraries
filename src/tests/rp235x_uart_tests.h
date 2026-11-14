#pragma once

// The RP235x_UART tests: both directions sharing one hardware UART. Run testUART(), from the one .cpp that
// includes this header (it defines the test's global objects)
// Looped back: TX (4) wired to RX (5), which are UART 1's pins

#include <Arduino.h>
#include <hardware/uart.h>
#include "common.debug.h"

#include "rp235x_uart.h"

constexpr RP235x_UART_Config UART_RX_115200 = {
	.uartIndex = 1,
	.pin = 5,
	.baudRate = UART_Master::BaudRate::Baud115200,
};

constexpr RP235x_UART_Config UART_TX_115200 = {
	.uartIndex = 1,
	.pin = 4,
	.baudRate = UART_Master::BaudRate::Baud115200,
};

constexpr RP235x_UART_Config UART_RX_9600 = {
	.uartIndex = 1,
	.pin = 5,
	.baudRate = UART_Master::BaudRate::Baud9600,
};

constexpr RP235x_UART_Config UART_TX_9600 = {
	.uartIndex = 1,
	.pin = 4,
	.baudRate = UART_Master::BaudRate::Baud9600,
};

uint32_t uartTestsRun = 0;
uint32_t uartTestsPassed = 0;

/// @brief Print one test's result
/// @param name The test's name
/// @param passed True if the test passed
void reportUARTTest(const char *name, const bool passed)
{
	++uartTestsRun;
	if (passed)
		++uartTestsPassed;

	logStr(name);
	if (passed)
		logStr("PASS");
	else
		logStr("FAIL");
}

/// @brief Send a byte and check it comes back, to show the UART is running in both directions
/// @param value The byte to send
/// @return True if the same byte was received
bool loopbackByte(const uint8_t value)
{
	uart_inst_t *const uart = uart_get_instance(UART_RX_115200.uartIndex);

	// Setting up a pin can leave a glitch byte behind, which isn't part of this check
	while (uart_is_readable(uart))
		uart_getc(uart);

	uart_putc_raw(uart, value);

	// Long enough for a byte at the slowest baud rate tested (9600: about 1 ms)
	constexpr uint32_t RECEIVE_TIMEOUT_MICROS = 5000;
	if (!uart_is_readable_within_us(uart, RECEIVE_TIMEOUT_MICROS))
		return false;

	return uart_getc(uart) == static_cast<char>(value);
}

void testUART()
{
	RP235x_UART_RX rx = {};
	RP235x_UART_TX tx = {};

	logStr("");
	logStr("---- Sharing one UART ----");
	reportUARTTest("Test 1: RX set up", RP235x_UART::setup_RX(UART_RX_115200, rx));
	reportUARTTest("Test 2: TX set up on the running UART, same baud rate", RP235x_UART::setup_TX(UART_TX_115200, tx));
	reportUARTTest("Test 3: loopback after both set up", loopbackByte(0xA5));
	reportUARTTest("Test 4: RX set up again is rejected", !RP235x_UART::setup_RX(UART_RX_115200, rx));

	logStr("");
	logStr("---- Tearing down one direction ----");
	RP235x_UART::teardown_RX(UART_RX_115200);
	reportUARTTest("Test 5: RX at a different baud rate to TX is rejected", !RP235x_UART::setup_RX(UART_RX_9600, rx));
	reportUARTTest("Test 6: RX set up again, same baud rate as TX", RP235x_UART::setup_RX(UART_RX_115200, rx));
	reportUARTTest("Test 7: loopback, so TX kept running through RX's teardown", loopbackByte(0x5A));

	logStr("");
	logStr("---- Tearing down both directions ----");
	RP235x_UART::teardown_RX(UART_RX_115200);
	RP235x_UART::teardown_TX(UART_TX_115200);
	reportUARTTest("Test 8: TX set up at a new baud rate", RP235x_UART::setup_TX(UART_TX_9600, tx));
	reportUARTTest("Test 9: RX set up at the new baud rate", RP235x_UART::setup_RX(UART_RX_9600, rx));
	reportUARTTest("Test 10: loopback at the new baud rate", loopbackByte(0x3C));

	RP235x_UART::teardown_RX(UART_RX_9600);
	RP235x_UART::teardown_TX(UART_TX_9600);

	logStr("");
	logValue("Tests run = ", uartTestsRun);
	logValue("Tests passed = ", uartTestsPassed);
}
