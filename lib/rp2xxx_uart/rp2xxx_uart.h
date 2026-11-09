#pragma once

#include <Arduino.h>
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/uart.h>
#include "uart_master.h"

/// @brief One direction of a hardware UART: the UART it uses, and its pin. The UART is held as its index, not as a
/// @brief uart_inst_t pointer, so a config can be a constexpr (the pointer comes from a cast that isn't allowed in one)
struct RP2xxx_UART_Config_t final
{
	uint8_t uartIndex = 0; // 0 to NUM_UARTS - 1
	uint8_t pin = 0;	   // The GPIO the UART receives or sends on. It must be one of that UART's pins
};

struct RP2xxx_UART_RX_t final
{
	const volatile void *address = nullptr;
	uint dreq = 0;
	uint32_t baudRate = 0; // (bits per second)
};

struct RP2xxx_UART_TX_t final
{
	volatile uint32_t *address = nullptr;
	const volatile uint32_t *statusAddress = nullptr;
	uint32_t fullMask = 0; // The bit in the status that's set while the TX FIFO is full
	uint32_t baudRate = 0; // (bits per second)
};

/// @brief A UART (hardware or PIO): where each byte received is and the DREQ that signals it, and where each byte
/// @brief to send is written and how to tell when it can be
struct RP2xxx_UART_RX_TX_t final
{
	RP2xxx_UART_RX_t rx = {};
	RP2xxx_UART_TX_t tx = {};
};

/// @brief Helper that receives and sends UART (8N1) using a hardware UART. It can't also be used through Serial1 or Serial2
/// @brief Its RX FIFO is only 32 bytes, so the bytes received must be read as they arrive, e.g. by a DMA
class RP2xxx_UART final
{
private:
	/// @brief Tear down one direction of a hardware UART: stop the UART, and return its pin to GPIO
	/// @param config The UART and its pin, as given to setup_RX or setup_TX
	static void teardownUART(const RP2xxx_UART_Config_t &config)
	{
		uart_deinit(uart_get_instance(config.uartIndex));
		gpio_deinit(config.pin);
	}

public:
	// A utility, so it's never instantiated
	RP2xxx_UART() = delete;

	/// @brief Set up a hardware UART that receives
	/// @param rxConfig The UART to receive with, and the pin to receive on. The pin must be one of the UART's RX pins
	/// @param baudRate The baud rate
	/// @param rx Receives the UART's RX info. Only changed if the UART is set up
	/// @return True if set up,false if the UART doesn't exist
	static bool setup_RX(const RP2xxx_UART_Config_t &rxConfig, const UART_Master::BaudRate baudRate, RP2xxx_UART_RX_t &rx)
	{
		if (rxConfig.uartIndex >= NUM_UARTS)
			return false;

		uart_inst_t *const uartInstance = uart_get_instance(rxConfig.uartIndex);
		const uint rxPin = rxConfig.pin;

		// Enable the DMA requests (the frame format is set to 8N1: 8 data bits, no parity, 1 stop bit)
		uart_init(uartInstance, static_cast<uint>(baudRate));
		gpio_set_function(rxPin, UART_FUNCSEL_NUM(uartInstance, rxPin));

		rx.address = &uart_get_hw(uartInstance)->dr;
		rx.dreq = uart_get_dreq_num(uartInstance, false);
		rx.baudRate = static_cast<uint32_t>(baudRate);

		return true;
	}

	/// @brief Set up a hardware UART that sends
	/// @param txConfig The UART to send with, and the pin to send on. The pin must be one of the UART's TX pins
	/// @param baudRate The baud rate
	/// @param tx Receives the UART's TX info. Only changed if the UART is set up
	/// @return True if set up,false if the UART doesn't exist
	static bool setup_TX(const RP2xxx_UART_Config_t &txConfig, const UART_Master::BaudRate baudRate, RP2xxx_UART_TX_t &tx)
	{
		if (txConfig.uartIndex >= NUM_UARTS)
			return false;

		uart_inst_t *const uartInstance = uart_get_instance(txConfig.uartIndex);
		const uint txPin = txConfig.pin;

		// Enable the DMA requests (the frame format is set to 8N1: 8 data bits, no parity, 1 stop bit)
		uart_init(uartInstance, static_cast<uint>(baudRate));
		gpio_set_function(txPin, UART_FUNCSEL_NUM(uartInstance, txPin));

		tx.address = &uart_get_hw(uartInstance)->dr;
		tx.statusAddress = &uart_get_hw(uartInstance)->fr;
		tx.fullMask = UART_UARTFR_TXFF_BITS;
		tx.baudRate = static_cast<uint32_t>(baudRate);

		return true;
	}

	/// @brief Set up a hardware UART that receives and sends
	/// @param rxConfig The UART to receive with, and the pin to receive on
	/// @param txConfig The UART to send with, and the pin to send on
	/// @param baudRate The baud rate
	/// @param uart Receives the UART's info. Only changed if the UART is set up
	/// @return True if set up,false if either UART doesn't exist
	static bool setup_RX_TX(const RP2xxx_UART_Config_t &rxConfig, const RP2xxx_UART_Config_t &txConfig,
							 const UART_Master::BaudRate baudRate, RP2xxx_UART_RX_TX_t &uart)
	{
		return setup_RX(rxConfig, baudRate, uart.rx) &&
			   setup_TX(txConfig, baudRate, uart.tx);
	}

	/// @brief Tear down a hardware UART that receives
	/// @param rxConfig The config given to setup_RX
	static void teardown_RX(const RP2xxx_UART_Config_t &rxConfig)
	{
		teardownUART(rxConfig);
	}

	/// @brief Tear down a hardware UART that sends
	/// @param txConfig The config given to setup_TX
	static void teardown_TX(const RP2xxx_UART_Config_t &txConfig)
	{
		teardownUART(txConfig);
	}

	/// @brief Tear down a hardware UART that receives and sends
	/// @param rxConfig The receiving config given to setup_RX_TX
	/// @param txConfig The sending config given to setup_RX_TX
	static void teardown_RX_TX(const RP2xxx_UART_Config_t &rxConfig, const RP2xxx_UART_Config_t &txConfig)
	{
		teardown_RX(rxConfig);
		teardown_TX(txConfig);
	}
};

/// @brief Helper that has a DMA copy every byte a UART receives into RAM, so no core time is spent per byte
class RP235x_UART_DMA final
{
private:
	/// @brief Calculate how many of the write address's low bits the DMA ring wraps within
	/// @brief e.g. 64 bytes gives 6 bits (2^6 = 64)
	/// @param bufferSize The ring buffer's size (bytes)
	/// @return The number of bits. Rounded up if bufferSize isn't a power of 2 (e.g. 70 bytes gives 7 bits)
	static constexpr uint8_t calcRingAddressBits(const uint16_t bufferSize)
	{
		uint8_t bits = 0;
		while ((1 << bits) < bufferSize)
			++bits;

		return bits;
	}

public:
	// A utility, so it's never instantiated
	RP235x_UART_DMA() = delete;

	/// @brief Start a DMA copying every byte the UART receives into a ring buffer, endlessly. If the channel has
	/// @brief already been started, it's restarted for the new UART
	/// @tparam BUFFER_SIZE The ring buffer's size (bytes), deduced from buffer. A power of 2, from 2 to 32768
	/// @param uart The UART to receive from, set up by RP2xxx_PIO_UART or RP2xxx_UART
	/// @param buffer The ring buffer. It must be aligned to its own size
	/// @param channel The DMA channel. If it's -1, a free channel is claimed and it receives that channel
	/// @return True if started, false if the UART hasn't been set up, the buffer isn't aligned to its own size or
	/// 		no DMA channel was free
	template <uint16_t BUFFER_SIZE>
	static bool setupRing(const RP2xxx_UART_RX_TX_t &uart, volatile uint8_t (&buffer)[BUFFER_SIZE], int &channel)
	{
		static_assert((BUFFER_SIZE >= 2) && (BUFFER_SIZE <= 32768), "The DMA ring supports 2 to 32768 bytes");

		constexpr uint8_t RING_ADDRESS_BITS = calcRingAddressBits(BUFFER_SIZE);
		static_assert((1 << RING_ADDRESS_BITS) == BUFFER_SIZE, "The DMA ring wraps on a power of 2, so the buffer's size must be one");

		if (uart.rx.address == nullptr)
			return false;

		// The DMA wraps its write address on a boundary of the buffer's size, so the buffer must be aligned to it
		if ((reinterpret_cast<uintptr_t>(buffer) % BUFFER_SIZE) != 0)
			return false;

		if (channel < 0)
		{
			channel = dma_claim_unused_channel(false);
			if (channel < 0)
				return false;
		}
		else
		{
			// Stop the running transfer, so the channel can be restarted with the new UART
			dma_channel_abort(static_cast<uint>(channel));
		}

		// Clear any bytes from a previous UART, so they can't be read as received from the new one
		for (uint16_t index = 0; index < BUFFER_SIZE; ++index)
			buffer[index] = 0;

		dma_channel_config config = dma_channel_get_default_config(static_cast<uint>(channel));
		channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
		channel_config_set_read_increment(&config, false);
		channel_config_set_write_increment(&config, true);
		channel_config_set_ring(&config, true, RING_ADDRESS_BITS);
		channel_config_set_dreq(&config, uart.rx.dreq);

		// The transfer is endless, so the DMA keeps filling the ring buffer for as long as the UART receives
		dma_channel_configure(
			static_cast<uint>(channel),
			&config,
			buffer,
			uart.rx.address,
			dma_encode_endless_transfer_count(),
			true);

		return true;
	}

	static void teardownRing(const int channel)
	{
		if (channel >= 0)
		{
			dma_channel_abort(static_cast<uint>(channel));
			dma_channel_unclaim(static_cast<uint>(channel));
		}
	}
};

//--------------------------------------------------------------------------------------------------

/// @brief One direction of a PIO UART: the state machine it uses, and its pin. The PIO is held as its index, not as a
/// @brief PIO pointer, so a config can be a constexpr (the pointer comes from a cast that isn't allowed in one)
struct RP2xxx_PIO_UART_Config_t final
{
	uint8_t pioIndex = 0;	  // 0 to NUM_PIOS - 1
	uint8_t stateMachine = 0; // 0 to NUM_PIO_STATE_MACHINES - 1
	uint8_t pin = 0;		  // The GPIO the state machine receives or sends on
};

/// @brief Helper that receives and sends UART (8N1) on any GPIOs using a PIO
/// @brief It has no RX buffer, so the bytes received must be read as they arrive, e.g. by a DMA
class RP2xxx_PIO_UART final
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
	static bool setupStateMachine(const RP2xxx_PIO_UART_Config_t &config, const pio_program_t &program,
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
	static void teardownStateMachine(const RP2xxx_PIO_UART_Config_t &config)
	{
		const PIO pio = pio_get_instance(config.pioIndex);
		pio_sm_set_enabled(pio, config.stateMachine, false);
		pio_sm_unclaim(pio, config.stateMachine);
		gpio_deinit(config.pin);
	}

public:
	// A utility, so it's never instantiated
	RP2xxx_PIO_UART() = delete;

	/// @brief Set up a PIO UART that receives
	/// @param rxConfig The state machine to receive with, and the pin to receive on. Its PIO's GPIO base must already
	/// let it reach the pin
	/// @param baudRate The baud rate
	/// @param rx Receives the UART's RX info. Only changed if the UART is set up
	/// @return True if set up,false if the state machine can't be set up (see setupStateMachine)
	static bool setup_RX(const RP2xxx_PIO_UART_Config_t &rxConfig, const UART_Master::BaudRate baudRate, RP2xxx_UART_RX_t &rx)
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

		const float cyclesPerSecond = static_cast<float>(RX_CYCLES_PER_BIT) * static_cast<float>(static_cast<uint32_t>(baudRate));
		sm_config_set_clkdiv(&config, static_cast<float>(clock_get_hz(clk_sys)) / cyclesPerSecond);

		pio_sm_init(pio, stateMachine, offset, &config);
		pio_sm_set_enabled(pio, stateMachine, true);

		rx.address = reinterpret_cast<const volatile uint8_t *>(&pio->rxf[stateMachine]) + RX_FIFO_TOP_BYTE;
		rx.dreq = pio_get_dreq(pio, stateMachine, false);
		rx.baudRate = static_cast<uint32_t>(baudRate);

		return true;
	}

	/// @brief Set up a PIO UART that sends
	/// @param txConfig The state machine to send with, and the pin to send on. Its PIO's GPIO base must already let
	/// it reach the pin
	/// @param baudRate The baud rate
	/// @param tx Receives the UART's TX info. Only changed if the UART is set up
	/// @return True if set up,false if the state machine can't be set up (see setupStateMachine)
	static bool setup_TX(const RP2xxx_PIO_UART_Config_t &txConfig, const UART_Master::BaudRate baudRate, RP2xxx_UART_TX_t &tx)
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

		const float cyclesPerSecond = static_cast<float>(TX_CYCLES_PER_BIT) * static_cast<float>(static_cast<uint32_t>(baudRate));
		sm_config_set_clkdiv(&config, static_cast<float>(clock_get_hz(clk_sys)) / cyclesPerSecond);

		pio_sm_init(pio, stateMachine, offset, &config);
		pio_sm_set_enabled(pio, stateMachine, true);

		tx.address = &pio->txf[stateMachine];
		tx.statusAddress = &pio->fstat;
		tx.fullMask = static_cast<uint32_t>(1) << (PIO_FSTAT_TXFULL_LSB + stateMachine);
		tx.baudRate = static_cast<uint32_t>(baudRate);

		return true;
	}

	/// @brief Set up a PIO UART that receives and sends
	/// @param rxConfig The state machine to receive with, and the pin to receive on
	/// @param txConfig The state machine to send with, and the pin to send on
	/// @param baudRate The baud rate
	/// @param uart Receives the UART's info. Only changed if the UART is set up
	/// @return True if set up,false if either state machine can't be set up (see setupStateMachine)
	static bool setup_RX_TX(const RP2xxx_PIO_UART_Config_t &rxConfig, const RP2xxx_PIO_UART_Config_t &txConfig,
							 const UART_Master::BaudRate baudRate, RP2xxx_UART_RX_TX_t &uart)
	{
		return setup_RX(rxConfig, baudRate, uart.rx) &&
			   setup_TX(txConfig, baudRate, uart.tx);
	}

	/// @brief Tear down a PIO UART that receives, tearing down its state machine
	/// @param rxConfig The config given to setup_RX
	static void teardown_RX(const RP2xxx_PIO_UART_Config_t &rxConfig)
	{
		teardownStateMachine(rxConfig);
	}

	/// @brief Tear down a PIO UART that sends, tearing down its state machine
	/// @param txConfig The config given to setup_TX
	static void teardown_TX(const RP2xxx_PIO_UART_Config_t &txConfig)
	{
		teardownStateMachine(txConfig);
	}

	/// @brief Tear down a PIO UART that receives and sends, tearing down its state machines
	/// @param rxConfig The receiving config given to setup_RX_TX
	/// @param txConfig The sending config given to setup_RX_TX
	static void teardown_RX_TX(const RP2xxx_PIO_UART_Config_t &rxConfig, const RP2xxx_PIO_UART_Config_t &txConfig)
	{
		teardown_RX(rxConfig);
		teardown_TX(txConfig);
	}
};