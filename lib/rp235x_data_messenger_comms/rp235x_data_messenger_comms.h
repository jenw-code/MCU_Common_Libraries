#pragma once

#include "data_messenger.h"
#include "rp235x_uart.h"
#include "rp235x_uart_dma.h"

/// @brief A Data_Messenger's comms on the RP235x: it knows how to send and receive with DMA on hardware or PIO
/// @brief UARTs, and how to signal and watch the FL lines. The messenger decides when
/// @tparam UART_RX_Config The UART that receives: RP235x_UART_Config or RP235x_PIO_UART_Config
/// @tparam UART_TX_Config The UART that sends: RP235x_UART_Config or RP235x_PIO_UART_Config
/// @tparam TWO_STEP_PAYLOAD_IN_PSRAM Whether the two-step payload buffer is reserved in PSRAM, which must be fitted,
/// rather than the RP235x's internal RAM
template <typename UART_RX_Config, typename UART_TX_Config, bool TWO_STEP_PAYLOAD_IN_PSRAM = false>
class RP235x_Data_Messenger_Comms final
{
public:
	// Note: declared before the private section, as the config it holds must be a complete type there

	/// @brief The link's UARTs, DMA channels and FL pins
	struct UART_Config final
	{
		UART_RX_Config uartRX = {};
		UART_TX_Config uartTX = {};
		uint8_t dmaRXChannel = 0; // 0 to NUM_DMA_CHANNELS - 1
		uint8_t dmaTXChannel = 0; // 0 to NUM_DMA_CHANNELS - 1
		uint8_t rx_fl_pin = 0;	  // GPIO for the receiving FL line
		uint8_t tx_fl_pin = 0;	  // GPIO for the sending FL line
	};

private:
	const UART_Config _uartConfig;

	RP235x_UART_DMA_RX_Transfer _dmaRX = {};
	RP235x_UART_DMA_TX _dmaTX = {};

	// The setup_RX overloads let the RX config's type pick the helper (hardware or PIO UART) that sets it up

	/// @brief Set up a hardware UART that receives
	/// @param rxConfig The UART to receive with, its pin and baud rate
	/// @param rx Receives the UART's RX info. Only changed if the UART is set up
	/// @return True if set up, false if the UART doesn't exist
	static bool setup_RX(const RP235x_UART_Config &rxConfig, RP235x_UART_RX &rx)
	{
		return RP235x_UART::setup_RX(rxConfig, rx);
	}

	/// @brief Set up a PIO UART that receives
	/// @param rxConfig The state machine to receive with, its pin and baud rate
	/// @param rx Receives the UART's RX info. Only changed if the UART is set up
	/// @return True if set up, false if the state machine can't be set up
	static bool setup_RX(const RP235x_PIO_UART_Config &rxConfig, RP235x_UART_RX &rx)
	{
		return RP235x_PIO_UART::setup_RX(rxConfig, rx);
	}

	// The setup_TX overloads let the TX config's type pick the helper (hardware or PIO UART) that sets it up

	/// @brief Set up a hardware UART that sends
	/// @param txConfig The UART to send with, its pin and baud rate
	/// @param tx Receives the UART's TX info. Only changed if the UART is set up
	/// @return True if set up, false if the UART doesn't exist
	static bool setup_TX(const RP235x_UART_Config &txConfig, RP235x_UART_TX &tx)
	{
		return RP235x_UART::setup_TX(txConfig, tx);
	}

	/// @brief Set up a PIO UART that sends
	/// @param txConfig The state machine to send with, its pin and baud rate
	/// @param tx Receives the UART's TX info. Only changed if the UART is set up
	/// @return True if set up, false if the state machine can't be set up
	static bool setup_TX(const RP235x_PIO_UART_Config &txConfig, RP235x_UART_TX &tx)
	{
		return RP235x_PIO_UART::setup_TX(txConfig, tx);
	}

	// The waitForTXIdle overloads let the TX config's type pick how to wait (hardware or PIO UART)

	/// @brief Wait until a hardware UART has sent every byte, up to the end of the last stop bit. Call it once
	/// @brief nothing more will be written to the TX FIFO, as the UART is briefly idle between bytes written slower
	/// @brief than it sends them
	/// @param txConfig The config given to setup_TX
	static void waitForTXIdle(const RP235x_UART_Config &txConfig)
	{
		uart_tx_wait_blocking(uart_get_instance(txConfig.uartIndex));
	}

	/// @brief Wait until a PIO UART has sent every byte, up to the end of the last stop bit. Call it once nothing
	/// @brief more will be written to the TX FIFO, as the FIFO is briefly empty between bytes written slower than
	/// @brief they're sent
	/// @param txConfig The config given to setup_TX
	static void waitForTXIdle(const RP235x_PIO_UART_Config &txConfig)
	{
		const PIO pio = pio_get_instance(txConfig.pioIndex);
		while (!pio_sm_is_tx_fifo_empty(pio, txConfig.stateMachine))
			tight_loop_contents();

		// The state machine takes the last byte from the FIFO as the stop bit of the byte before it starts, so
		// that stop bit, then the last byte (start bit, 8 data bits, stop bit), are still to be sent
		constexpr uint32_t MICROS_PER_SECOND = 1000000;
		constexpr uint32_t BITS_STILL_TO_SEND = 11;
		const uint32_t baudRate = static_cast<uint32_t>(txConfig.baudRate);

		// The +1 covers the fraction of a microsecond the integer division drops
		const uint32_t sendMicros = ((MICROS_PER_SECOND * BITS_STILL_TO_SEND) / baudRate) + 1;
		delayMicroseconds(sendMicros);
	}

	// The clearRXFIFO overloads let the RX config's type pick how to clear it (hardware or PIO UART)

	/// @brief Discard any bytes waiting in a hardware UART's RX FIFO
	/// @param rxConfig The config given to setup_RX
	static void clearRXFIFO(const RP235x_UART_Config &rxConfig)
	{
		uart_inst_t *const uart = uart_get_instance(rxConfig.uartIndex);
		while (uart_is_readable(uart))
			uart_getc(uart);
	}

	/// @brief Discard any bytes waiting in a PIO UART's RX FIFO
	/// @param rxConfig The config given to setup_RX
	static void clearRXFIFO(const RP235x_PIO_UART_Config &rxConfig)
	{
		const PIO pio = pio_get_instance(rxConfig.pioIndex);
		while (!pio_sm_is_rx_fifo_empty(pio, rxConfig.stateMachine))
			pio_sm_get(pio, rxConfig.stateMachine);
	}

public:
	/// @brief Create the comms for a link. Nothing is set up until setup is called
	/// @param uartConfig The link's UARTs, DMA channels and FL pins
	explicit RP235x_Data_Messenger_Comms(const UART_Config &uartConfig) : _uartConfig(uartConfig)
	{
	}

	/// @brief Set up the FL pins, then the UARTs and DMAs that send and receive
	/// @return True if set up, false if PSRAM is needed but not fitted, or a UART or a DMA channel can't be set up
	bool setup()
	{
		// The two-step payload buffer is reserved in PSRAM, so it must be fitted
		if constexpr (TWO_STEP_PAYLOAD_IN_PSRAM)
		{
			if (rp2040.getPSRAMSize() == 0)
				return false;
		}

		// Set to not sending before it becomes an output, so the receiver never sees a frame start that isn't one
		stopSendFrame();
		pinMode(_uartConfig.tx_fl_pin, OUTPUT);

		pinMode(_uartConfig.rx_fl_pin, INPUT);

		// Sending: the UART, then the DMA that feeds it
		RP235x_UART_TX uartTX = {};
		if (!setup_TX(_uartConfig.uartTX, uartTX))
			return false;

		if (!RP235x_UART_DMA::setup_TX({.channel = _uartConfig.dmaTXChannel}, uartTX, _dmaTX))
			return false;

		// Receiving: the UART, then the DMA that empties it
		RP235x_UART_RX uartRX = {};
		if (!setup_RX(_uartConfig.uartRX, uartRX))
			return false;

		if (!RP235x_UART_DMA::setup_RX({.channel = _uartConfig.dmaRXChannel}, uartRX, _dmaRX))
			return false;

		return true;
	}

	/// @brief Attach the receiving FL line's interrupt, on both edges
	/// @param handler Called on each change, with context
	/// @param context Passed to handler (e.g. the messenger the interrupt belongs to)
	void attachReceiveFrameInterrupt(void (*handler)(void *), void *context)
	{
		attachInterruptParam(_uartConfig.rx_fl_pin, handler, CHANGE, context);
	}

	/// @brief Reserve the memory a two-step payload is received into
	/// @param size Number of bytes to reserve
	/// @return The memory, or nullptr if it couldn't be reserved
	uint8_t *reservePayloadBuffer(const uint32_t size)
	{
		if constexpr (TWO_STEP_PAYLOAD_IN_PSRAM)
		{
			// pmalloc only exists properly on boards with PSRAM; elsewhere it's a deprecated stub that always fails
#ifdef RP2350_PSRAM_CS
			return static_cast<uint8_t *>(pmalloc(size));
#else
			static_assert(!TWO_STEP_PAYLOAD_IN_PSRAM, "This board has no PSRAM for the two-step payload buffer");
			return nullptr;
#endif
		}
		else
			return static_cast<uint8_t *>(malloc(size));
	}

	/// @brief Release the memory reserved by reservePayloadBuffer
	/// @param buffer The memory, as returned by reservePayloadBuffer
	void releasePayloadBuffer(uint8_t *buffer)
	{
		free(buffer);
	}

	/// @brief Set the sending FL line to tell the receiver a frame is about to be sent
	void startSendFrame()
	{
		digitalWrite(_uartConfig.tx_fl_pin, Data_Messenger_Common::FRAME_SENDING_STATUS);
	}

	/// @brief Set the sending FL line to tell the receiver the frame has been sent
	void stopSendFrame()
	{
		digitalWrite(_uartConfig.tx_fl_pin, Data_Messenger_Common::FRAME_NOT_SENDING_STATUS);
	}

	/// @brief Whether the other end is sending a frame, as shown by the receiving FL line
	/// @return True while a frame is being sent
	bool frameSending()
	{
		return (digitalRead(_uartConfig.rx_fl_pin) == Data_Messenger_Common::FRAME_SENDING_STATUS);
	}

	/// @brief Whether the other end isn't sending a frame, as shown by the receiving FL line
	/// @return True while no frame is being sent
	bool frameNotSending()
	{
		return (digitalRead(_uartConfig.rx_fl_pin) == Data_Messenger_Common::FRAME_NOT_SENDING_STATUS);
	}

	/// @brief Start the DMA sending some bytes. They must stay unchanged until waitForTXIdle returns
	/// @param bytes The bytes
	/// @param size Number of bytes
	/// @return True, as starting a DMA transfer doesn't report failure
	bool sendBytes(const uint8_t *bytes, const uint32_t size)
	{
		dma_channel_transfer_from_buffer_now(_dmaTX.channel, bytes, size);
		return true;
	}

	/// @brief Wait until every byte given to sendBytes has been sent
	void waitForTXIdle()
	{
		// The UART is briefly idle between bytes the DMA writes, so it's only checked once the DMA has finished
		while (dma_channel_is_busy(_dmaTX.channel))
			tight_loop_contents();

		waitForTXIdle(_uartConfig.uartTX);
	}

	/// @brief Start the DMA receiving bytes into a buffer, in the background
	/// @param bytes Where the bytes are received into
	/// @param size The most bytes to receive
	void startReceiveBytes(uint8_t *bytes, const uint32_t size)
	{
		dma_channel_transfer_to_buffer_now(_dmaRX.channel, bytes, size);
	}

	/// @brief Discard any bytes waiting in the receiving UART's RX FIFO
	void clearRXFIFO()
	{
		clearRXFIFO(_uartConfig.uartRX);
	}

	/// @brief Stop the DMA receiving the bytes started by startReceiveBytes
	void stopReceiveBytes()
	{
		dma_channel_abort(_dmaRX.channel);
	}
};
