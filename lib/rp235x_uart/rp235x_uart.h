#pragma once

#include <Arduino.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/uart.h>
#include "uart_master.h"

/// @brief One direction of a hardware UART: the UART it uses, its pin, and its baud rate. The UART is held as its
/// @brief index, not as a uart_inst_t pointer, so a config can be a constexpr (the pointer comes from a cast that
/// @brief isn't allowed in one)
struct RP235x_UART_Config final
{
	uint8_t uartIndex = 0; // 0 to NUM_UARTS - 1
	uint8_t pin = 0;	   // The GPIO the UART receives or sends on. It must be one of that UART's pins
	UART_Master::BaudRate baudRate = UART_Master::BaudRate::Baud115200;
};

struct RP235x_UART_RX final
{
	const volatile void *address = nullptr;
	uint dreq = 0;		   // Signals while the RX FIFO holds data, for a DMA to pace its reads by
	uint32_t baudRate = 0; // (bits per second)
};

struct RP235x_UART_TX final
{
	volatile uint32_t *address = nullptr;
	const volatile uint32_t *statusAddress = nullptr;
	uint32_t fullMask = 0; // The bit in the status that's set while the TX FIFO is full
	uint dreq = 0;		   // Signals while the TX FIFO has space, for a DMA to pace its writes by
	uint32_t baudRate = 0; // (bits per second)
};

/// @brief Helper that receives and sends UART (8N1) using a hardware UART. It can't also be used through Serial1 or Serial2
/// @brief Its RX FIFO is only 32 bytes, so the bytes received must be read as they arrive, e.g. by a DMA
class RP235x_UART final
{
private:
	// One hardware UART serves both directions, so which directions use each UART is recorded.
	// Every setup and teardown uses these to ensure that RX does not affect TX, and vice versa
	static inline bool _rxInUse[NUM_UARTS] = {};
	static inline bool _txInUse[NUM_UARTS] = {};
	static inline uint32_t _baudRateInUse[NUM_UARTS] = {}; // (bits per second) 0 while the UART isn't running

	/// @brief Start one direction of a hardware UART: start the UART unless it's already running, and give the
	/// @brief direction's pin to the UART
	/// @param config The UART, the direction's pin, and the baud rate
	/// @param uartRunning True if the UART is already running
	static void startUART(const RP235x_UART_Config &config, const bool uartRunning)
	{
		uart_inst_t *const uartInstance = uart_get_instance(config.uartIndex);

		// uart_init resets the UART, so it's only called to start it.
		// It enables the DMA requests, and sets the frame format to 8N1: 8 data bits, no parity, 1 stop bit
		if (!uartRunning)
			uart_init(uartInstance, static_cast<uint>(config.baudRate));

		gpio_set_function(config.pin, UART_FUNCSEL_NUM(uartInstance, config.pin));
	}

	/// @brief Tear down one direction of a hardware UART: return the direction's pin to GPIO, and stop
	/// @brief the UART unless it's still needed
	/// @param config The UART and the direction's pin, as given to setup_RX or setup_TX
	/// @param uartStillNeeded True if the UART is still needed
	static void teardownUART(const RP235x_UART_Config &config, const bool uartStillNeeded)
	{
		gpio_deinit(config.pin);

		if (!uartStillNeeded)
			uart_deinit(uart_get_instance(config.uartIndex));
	}

public:
	// A utility, so it's never instantiated
	RP235x_UART() = delete;

	/// @brief Set up a hardware UART that receives
	/// @param rxConfig The UART to receive with, the pin to receive on, and the baud rate. The pin must be one of the
	/// UART's RX pins
	/// @param rx Receives the UART's RX info. Only changed if the UART is set up
	/// @return True if set up, false if the UART doesn't exist, already receives, or sends at a different baud rate
	static bool setup_RX(const RP235x_UART_Config &rxConfig, RP235x_UART_RX &rx)
	{
		const uint8_t uartIndex = rxConfig.uartIndex;
		if ((uartIndex >= NUM_UARTS) || _rxInUse[uartIndex])
			return false;

		if (!_txInUse[uartIndex])
			_baudRateInUse[uartIndex] = static_cast<uint32_t>(rxConfig.baudRate);

		// Both directions share the UART, so they must share its baud rate
		else if (static_cast<uint32_t>(rxConfig.baudRate) != _baudRateInUse[uartIndex])
			return false;

		// The UART is already running if it sends
		startUART(rxConfig, _txInUse[uartIndex]);

		uart_inst_t *const uartInstance = uart_get_instance(uartIndex);

		rx.address = &uart_get_hw(uartInstance)->dr;
		rx.dreq = uart_get_dreq_num(uartInstance, false);
		rx.baudRate = static_cast<uint32_t>(rxConfig.baudRate);

		_rxInUse[uartIndex] = true;
		return true;
	}

	/// @brief Set up a hardware UART that sends
	/// @param txConfig The UART to send with, the pin to send on, and the baud rate. The pin must be one of the
	/// UART's TX pins
	/// @param tx Receives the UART's TX info. Only changed if the UART is set up
	/// @return True if set up, false if the UART doesn't exist, already sends, or receives at a different baud rate
	static bool setup_TX(const RP235x_UART_Config &txConfig, RP235x_UART_TX &tx)
	{
		const uint8_t uartIndex = txConfig.uartIndex;
		if ((uartIndex >= NUM_UARTS) || _txInUse[uartIndex])
			return false;

		if (!_rxInUse[uartIndex])
			_baudRateInUse[uartIndex] = static_cast<uint32_t>(txConfig.baudRate);

		// Both directions share the UART, so they must share its baud rate
		else if (static_cast<uint32_t>(txConfig.baudRate) != _baudRateInUse[uartIndex])
			return false;

		// The UART is already running if it receives
		startUART(txConfig, _rxInUse[uartIndex]);

		uart_inst_t *const uartInstance = uart_get_instance(uartIndex);

		tx.address = &uart_get_hw(uartInstance)->dr;
		tx.statusAddress = &uart_get_hw(uartInstance)->fr;
		tx.fullMask = UART_UARTFR_TXFF_BITS;
		tx.dreq = uart_get_dreq_num(uartInstance, true);
		tx.baudRate = static_cast<uint32_t>(txConfig.baudRate);

		_txInUse[uartIndex] = true;
		return true;
	}

	/// @brief Tear down a hardware UART that receives. The UART keeps running while it still sends
	/// @param rxConfig The config given to setup_RX
	static void teardown_RX(const RP235x_UART_Config &rxConfig)
	{
		const uint8_t uartIndex = rxConfig.uartIndex;
		_rxInUse[uartIndex] = false;

		// The UART is still needed if it sends
		teardownUART(rxConfig, _txInUse[uartIndex]);

		if (!_txInUse[uartIndex])
			_baudRateInUse[uartIndex] = 0;
	}

	/// @brief Tear down a hardware UART that sends. The UART keeps running while it still receives
	/// @param txConfig The config given to setup_TX
	static void teardown_TX(const RP235x_UART_Config &txConfig)
	{
		const uint8_t uartIndex = txConfig.uartIndex;
		_txInUse[uartIndex] = false;

		// The UART is still needed if it receives
		teardownUART(txConfig, _rxInUse[uartIndex]);

		if (!_rxInUse[uartIndex])
			_baudRateInUse[uartIndex] = 0;
	}
};

//--------------------------------------------------------------------------------------------------

/// @brief One direction of a PIO UART: the state machine it uses, its pin, and its baud rate. The PIO is held as its
/// @brief index, not as a PIO pointer, so a config can be a constexpr (the pointer comes from a cast that isn't
/// @brief allowed in one)
struct RP235x_PIO_UART_Config final
{
	uint8_t pioIndex = 0;	  // 0 to NUM_PIOS - 1
	uint8_t stateMachine = 0; // 0 to NUM_PIO_STATE_MACHINES - 1
	uint8_t pin = 0;		  // The GPIO the state machine receives or sends on
	UART_Master::BaudRate baudRate = UART_Master::BaudRate::Baud115200;
};

/// @brief Helper that receives and sends UART (8N1) on any GPIOs using a PIO
/// @brief It has no RX buffer, so the bytes received must be read as they arrive, e.g. by a DMA
class RP235x_PIO_UART final
{
private:
	// Where a program has been loaded into a PIO's instruction memory
	// No default member initializers, as they can't be used inside the enclosing class. Only used in arrays
	// initialised with = {}, which zeroes every element
	struct LoadedProgram final
	{
		bool loaded;
		uint offset;
	};
	// PIO receiver: pico-examples uart_rx_mini (uart_rx.pio, Copyright (c) 2020 Raspberry Pi (Trading) Ltd.,
	// BSD-3-Clause), pre-assembled because the build doesn't run pioasm. It receives 8N1 without checking the stop
	// bit, and pushes each byte to the top byte of the RX FIFO word
	static constexpr uint16_t RX_INSTRUCTIONS[] = {
		0x2020, // wait 0 pin 0         ; Wait for start bit
		0xEA27, // set x, 7 [10]        ; Preload bit counter, delay until eye of first data bit
		0x4001, // in pins, 1           ; Sample data
		0x0642	// jmp x-- bitloop [6]  ; Loop 8 times, each iteration is 8 cycles
	};

	static constexpr uint8_t RX_LENGTH = sizeof(RX_INSTRUCTIONS) / sizeof(RX_INSTRUCTIONS[0]);
	static constexpr uint8_t RX_WRAP_TARGET = 0;	  // The whole sequence repeats
	static constexpr uint8_t RX_WRAP = RX_LENGTH - 1; // for every byte received
	static constexpr uint8_t RX_CYCLES_PER_BIT = 8;	  // Set by the instruction delays
	static constexpr uint8_t RX_BITS_PER_PUSH = 8;	  // One byte per RX FIFO word
	static constexpr uint8_t RX_FIFO_TOP_BYTE = 3;	  // Bytes are shifted in from the top of the RX FIFO word

	static constexpr pio_program_t RX_PROGRAM = {
		.instructions = RX_INSTRUCTIONS,
		.length = RX_LENGTH,
		.origin = -1, // Can be loaded anywhere in the PIO's instruction memory
		.pio_version = 0,
		.used_gpio_ranges = 0, // The program only uses pins relative to its pin settings
	};

	// PIO transmitter: pico-examples uart_tx (uart_tx.pio, Copyright (c) 2020 Raspberry Pi (Trading) Ltd.,
	// BSD-3-Clause), pre-assembled because the build doesn't run pioasm. It sends 8N1, taking each byte from the
	// bottom byte of the TX FIFO word
	static constexpr uint16_t TX_INSTRUCTIONS[] = {
		0x9FA0, // pull side 1 [7]      ; Assert stop bit, or stall with line in idle state
		0xF727, // set x, 7 side 0 [7]  ; Preload bit counter, assert start bit for 8 clocks
		0x6001, // out pins, 1          ; Shift 1 bit from OSR to the first OUT pin
		0x0642	// jmp x-- bitloop [6]  ; Each loop iteration is 8 cycles
	};

	static constexpr uint8_t TX_LENGTH = sizeof(TX_INSTRUCTIONS) / sizeof(TX_INSTRUCTIONS[0]);
	static constexpr uint8_t TX_WRAP_TARGET = 0;	  // The whole sequence repeats
	static constexpr uint8_t TX_WRAP = TX_LENGTH - 1; // for every byte sent
	static constexpr uint8_t TX_CYCLES_PER_BIT = 8;	  // Set by the instruction delays
	static constexpr uint8_t TX_SIDE_SET_BITS = 2;	  // The TX pin's side-set bit, plus the bit that makes side-set optional

	static constexpr pio_program_t TX_PROGRAM = {
		.instructions = TX_INSTRUCTIONS,
		.length = TX_LENGTH,
		.origin = -1, // Can be loaded anywhere in the PIO's instruction memory
		.pio_version = 0,
		.used_gpio_ranges = 0, // The program only uses pins relative to its pin settings
	};

	// Each program is loaded once per PIO, and shared by every state machine in it that uses the program
	static inline LoadedProgram _rxPrograms[NUM_PIOS] = {};
	static inline LoadedProgram _txPrograms[NUM_PIOS] = {};

	/// @brief Load a program into a PIO, unless it's already loaded there
	/// @param program The program
	/// @param pio The PIO
	/// @param loadedProgram Where the program is in this PIO. Updated when the program is loaded
	/// @return True if the program is loaded, false if the PIO has no room for it
	static bool loadProgram(const pio_program_t &program, const PIO pio, LoadedProgram &loadedProgram)
	{
		if (loadedProgram.loaded)
			return true;

		if (!pio_can_add_program(pio, &program))
			return false;

		loadedProgram.offset = pio_add_program(pio, &program);
		loadedProgram.loaded = true;
		return true;
	}

	/// @brief Set up a state machine for a UART: claim it, with its program loaded, checking the pin can be used by it
	/// @param config The state machine and its pin
	/// @param program The program the state machine runs
	/// @param loadedPrograms Where the program is in each PIO
	/// @param offset Receives where the program is in the state machine's PIO
	/// @return True if set up,false if the state machine doesn't exist or is already claimed, the pin is outside
	/// 		its PIO's 32 pin window, or the PIO has no room for the program
	static bool setupStateMachine(const RP235x_PIO_UART_Config &config, const pio_program_t &program,
								  LoadedProgram (&loadedPrograms)[NUM_PIOS], uint &offset)
	{
		if ((config.pioIndex >= NUM_PIOS) || (config.stateMachine >= NUM_PIO_STATE_MACHINES))
			return false;

		const PIO pio = pio_get_instance(config.pioIndex);

		// A PIO's GPIO base is shared by its state machines, so it's set as part of the PIO allocation, not here
		const uint gpioBase = pio_get_gpio_base(pio);
		if ((config.pin < gpioBase) || (config.pin >= (gpioBase + 32)))
			return false;

		// pio_sm_claim() halts if the state machine is already claimed, rather than failing
		if (pio_sm_is_claimed(pio, config.stateMachine))
			return false;

		LoadedProgram &loadedProgram = loadedPrograms[config.pioIndex];
		if (!loadProgram(program, pio, loadedProgram))
			return false;

		pio_sm_claim(pio, config.stateMachine);
		offset = loadedProgram.offset;
		return true;
	}

	/// @brief Tear down a state machine set up by setupStateMachine: stop it, unclaim it, and return its pin to GPIO
	/// @param config The state machine and its pin, as given to setupStateMachine
	static void teardownStateMachine(const RP235x_PIO_UART_Config &config)
	{
		const PIO pio = pio_get_instance(config.pioIndex);
		pio_sm_set_enabled(pio, config.stateMachine, false);
		pio_sm_unclaim(pio, config.stateMachine);
		gpio_deinit(config.pin);
	}

public:
	// A utility, so it's never instantiated
	RP235x_PIO_UART() = delete;

	/// @brief Set up a PIO UART that receives
	/// @param rxConfig The state machine to receive with, the pin to receive on, and the baud rate. Its PIO's GPIO base
	/// must already let it reach the pin
	/// @param rx Receives the UART's RX info. Only changed if the UART is set up
	/// @return True if set up, false if the state machine can't be set up (see setupStateMachine)
	static bool setup_RX(const RP235x_PIO_UART_Config &rxConfig, RP235x_UART_RX &rx)
	{
		uint offset;
		if (!setupStateMachine(rxConfig, RX_PROGRAM, _rxPrograms, offset))
			return false;

		const PIO pio = pio_get_instance(rxConfig.pioIndex);
		const uint stateMachine = rxConfig.stateMachine;
		const uint rxPin = rxConfig.pin;

		pio_sm_set_consecutive_pindirs(pio, stateMachine, rxPin, 1, false);
		pio_gpio_init(pio, rxPin);
		gpio_pull_up(rxPin); // The UART line idles high

		pio_sm_config config = pio_get_default_sm_config();
		sm_config_set_wrap(&config, offset + RX_WRAP_TARGET, offset + RX_WRAP);
		sm_config_set_in_pins(&config, rxPin);
		sm_config_set_in_shift(&config, true, true, RX_BITS_PER_PUSH); // Shift right, autopush each byte
		sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_RX);			   // Only receiving, so double the RX FIFO

		const float cyclesPerSecond = static_cast<float>(RX_CYCLES_PER_BIT) * static_cast<float>(static_cast<uint32_t>(rxConfig.baudRate));
		sm_config_set_clkdiv(&config, static_cast<float>(clock_get_hz(clk_sys)) / cyclesPerSecond);

		pio_sm_init(pio, stateMachine, offset, &config);
		pio_sm_set_enabled(pio, stateMachine, true);

		rx.address = reinterpret_cast<const volatile uint8_t *>(&pio->rxf[stateMachine]) + RX_FIFO_TOP_BYTE;
		rx.dreq = pio_get_dreq(pio, stateMachine, false);
		rx.baudRate = static_cast<uint32_t>(rxConfig.baudRate);

		return true;
	}

	/// @brief Set up a PIO UART that sends
	/// @param txConfig The state machine to send with, the pin to send on, and the baud rate. Its PIO's GPIO base must
	/// already let it reach the pin
	/// @param tx Receives the UART's TX info. Only changed if the UART is set up
	/// @return True if set up, false if the state machine can't be set up (see setupStateMachine)
	static bool setup_TX(const RP235x_PIO_UART_Config &txConfig, RP235x_UART_TX &tx)
	{
		uint offset;
		if (!setupStateMachine(txConfig, TX_PROGRAM, _txPrograms, offset))
			return false;

		const PIO pio = pio_get_instance(txConfig.pioIndex);
		const uint stateMachine = txConfig.stateMachine;
		const uint txPin = txConfig.pin;

		// The UART line idles high, so the pin is driven high before the state machine takes it over
		const uint64_t txPinMask = static_cast<uint64_t>(1) << txPin;
		pio_sm_set_pins_with_mask64(pio, stateMachine, txPinMask, txPinMask);
		pio_sm_set_consecutive_pindirs(pio, stateMachine, txPin, 1, true);
		pio_gpio_init(pio, txPin);

		pio_sm_config config = pio_get_default_sm_config();
		sm_config_set_wrap(&config, offset + TX_WRAP_TARGET, offset + TX_WRAP);
		sm_config_set_sideset(&config, TX_SIDE_SET_BITS, true, false); // Optional, and sets the pin's level, not its direction
		sm_config_set_sideset_pins(&config, txPin);
		sm_config_set_out_pins(&config, txPin, 1);
		sm_config_set_out_shift(&config, true, false, 32);	// Shift right, so bytes go LSB first. No autopull, as the program pulls
		sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX); // Only sending, so double the TX FIFO

		const float cyclesPerSecond = static_cast<float>(TX_CYCLES_PER_BIT) * static_cast<float>(static_cast<uint32_t>(txConfig.baudRate));
		sm_config_set_clkdiv(&config, static_cast<float>(clock_get_hz(clk_sys)) / cyclesPerSecond);

		pio_sm_init(pio, stateMachine, offset, &config);
		pio_sm_set_enabled(pio, stateMachine, true);

		tx.address = &pio->txf[stateMachine];
		tx.statusAddress = &pio->fstat;
		tx.fullMask = static_cast<uint32_t>(1) << (PIO_FSTAT_TXFULL_LSB + stateMachine);
		tx.dreq = pio_get_dreq(pio, stateMachine, true);
		tx.baudRate = static_cast<uint32_t>(txConfig.baudRate);

		return true;
	}

	/// @brief Tear down a PIO UART that receives, tearing down its state machine
	/// @param rxConfig The config given to setup_RX
	static void teardown_RX(const RP235x_PIO_UART_Config &rxConfig)
	{
		teardownStateMachine(rxConfig);
	}

	/// @brief Tear down a PIO UART that sends, tearing down its state machine
	/// @param txConfig The config given to setup_TX
	static void teardown_TX(const RP235x_PIO_UART_Config &txConfig)
	{
		teardownStateMachine(txConfig);
	}
};