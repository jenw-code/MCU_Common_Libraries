#pragma once

#include <Arduino.h>
#include "common.debug.h"
#include "common.parse.h"

/// @brief What every Data_Messenger and its comms share
class Data_Messenger_Common final
{
public:
	// A utility, so it's never instantiated
	Data_Messenger_Common() = delete;

	// FL is active-low. Each core's own type for LOW and HIGH is kept, as they differ between cores
	static constexpr decltype(LOW) FRAME_SENDING_STATUS = LOW;
	static constexpr decltype(HIGH) FRAME_NOT_SENDING_STATUS = HIGH;
};

/// @brief Used to organise transmitting and receiving data from one mcu to another over UART (using DMA). It
/// @brief decides when everything happens; its comms knows how to do it on its MCU
/// @tparam Comms The MCU's comms (e.g. RP235x_Data_Messenger_Comms). It must provide: setup, sendBytes,
/// @brief waitForTXIdle, startReceiveBytes, stopReceiveBytes, clearRXFIFO, startSendFrame, stopSendFrame, frameSending,
/// @brief attachReceiveFrameInterrupt, reservePayloadBuffer and releasePayloadBuffer
/// @tparam Send_Message_Type The enum of message types this end sends: enum class with a uint8_t underlying
/// @brief type, and MIN and MAX values aliasing its first and last types
/// @tparam Receive_Message_Type The enum of message types this end receives, in the same form. Received types
/// @brief outside MIN to MAX are reported as UnknownMessageType
template <typename Comms, typename Send_Message_Type, typename Receive_Message_Type>
class Data_Messenger
{
public:
	/// @brief The errors a messenger reports
	enum class Error : uint8_t
	{
		None,					// No error since the last report
		MessageDropped,			// A message arrived before the last one was handled
		InvalidStartByte,		// A frame didn't start with a header
		UnknownMessageType,		// The message's type is outside the receive message types' MIN to MAX
		PayloadMaxSizeExceeded, // The header announced more than the largest payload that can be received
		InvalidCRC,				// The message failed its CRC check
	};

	// Called with each error, from checkForMessages
	typedef void (*On_Error_Callback)(const Error error);

	// Called with each valid message received. payload is nullptr when payloadSize is 0
	typedef void (*On_Receive_Callback)(const Receive_Message_Type messageType, const uint8_t *payload,
										const uint32_t payloadSize);

private:
	static constexpr uint8_t DMA_BUFFER_SIZE = 64;
	static constexpr uint8_t HEADER_SIZE = 6; // HEADER_START_BYTE, message type, BUFFER_SIZE (4 bytes);
	static constexpr uint8_t CRC32_SIZE = 4;

	// Header layout
	static constexpr uint8_t HEADER_START_BYTE_INDEX = 0;
	static constexpr uint8_t MESSAGE_TYPE_INDEX = 1;
	static constexpr uint8_t PAYLOAD_SIZE_INDEX = 2;

	// The payload size and the CRC are each packed by packUInt32
	static_assert(HEADER_SIZE == PAYLOAD_SIZE_INDEX + sizeof(uint32_t), "The payload size must end the header");
	static_assert(CRC32_SIZE == sizeof(uint32_t), "The CRC must be a uint32_t");

	// The message type is sent as one header byte
	static_assert(sizeof(Send_Message_Type) == sizeof(uint8_t), "Send message types must be a uint8_t enum");
	static_assert(sizeof(Receive_Message_Type) == sizeof(uint8_t), "Receive message types must be a uint8_t enum");

	// Calc max buffer size that allows us to send the entire message in one step
	static constexpr uint8_t MAX_ONE_STEP_PAYLOAD_SIZE = DMA_BUFFER_SIZE - (HEADER_SIZE + CRC32_SIZE);

	// The max payload size that can be received
	static constexpr uint16_t MAX_TWO_STEP_PAYLOAD_SIZE = 20 * 1024;

	static constexpr uint8_t HEADER_START_BYTE = 0b10101010;

	static constexpr uint16_t STEP_TRANSITION_DELAY_MICROS = 1000;

	/// @brief The frame the receiving FL line's next edges belong to
	enum class Expected_Frame : uint8_t
	{
		NewMessage,		// The first frame of a message: a whole one-step message, or a two-step header
		TwoStepPayload, // The payload frame that follows a two-step header
	};

	/// @brief Permanent received message buffers
	struct RX_Message_Buffers final
	{
		uint8_t oneStepBuffer[DMA_BUFFER_SIZE] = {}; // Permanent memory, reused for every message
		uint8_t *twoStepPayloadBuffer = nullptr;	 // Permanent memory, reserved once and reused for every message
	};

	/// @brief Permanent sending message buffer
	struct TX_Message_Buffer final
	{
		uint8_t buffer[DMA_BUFFER_SIZE] = {}; // Permanent memory, reused for every message
	};

	Comms &_comms;

	volatile bool _messageReceived = false;

	// Set by the receiving FL line's interrupt, and reported by checkForMessages
	volatile Error _lastError = Error::None;

	On_Receive_Callback _receiveCallback = nullptr;
	On_Error_Callback _errorCallback = nullptr;

	RX_Message_Buffers _receivedMessage = {};
	TX_Message_Buffer _messageToSend = {};

	uint8_t _frameBuffer[DMA_BUFFER_SIZE] = {};

	// Only used by the receiving FL line's interrupt, so it isn't volatile. If anything else needs it, make it volatile
	Expected_Frame _expectedFrame = Expected_Frame::NewMessage;

	/// @brief Calculate the standard CRC-32 (as used by zlib and Ethernet) of some bytes, optionally continuing
	/// @brief from the CRC of the bytes before them, so a message received in pieces can be checked piece by piece
	/// @brief e.g. the ASCII bytes "123456789" give 0xCBF43926
	/// @param bytes The bytes
	/// @param size Number of bytes
	/// @param prevCRC The CRC returned for the bytes before these, or 0 if these are the first bytes
	/// @return The CRC of every byte so far
	static uint32_t calcCRC32(const uint8_t *const bytes, const size_t size, const uint32_t prevCRC = 0)
	{
		// The polynomial 0x04C11DB7 reversed, as each byte is processed least significant bit first
		constexpr uint32_t POLYNOMIAL = 0xEDB88320;

		// The register starts inverted, so leading zero bytes still change the CRC, and
		// ends inverted, as the standard specifies
		constexpr uint32_t INVERT = 0xFFFFFFFF;

		// Undo the previous result's final inversion, to carry on from its register value
		uint32_t crc = prevCRC ^ INVERT;

		for (size_t iByte = 0; iByte < size; ++iByte)
		{
			crc ^= bytes[iByte];

			for (uint8_t iBit = 0; iBit < 8; ++iBit)
			{
				const bool lowBitSet = ((crc & 1) != 0);
				crc >>= 1;

				if (lowBitSet)
					crc ^= POLYNOMIAL;
			}
		}

		return crc ^ INVERT;
	}

	/// @brief Pack a uint32_t into bytes, little-endian
	/// @brief e.g. 0x12345678 gives 0x78, 0x56, 0x34, 0x12
	/// @param bytes Where the 4 bytes go, least significant first
	/// @param value The value
	static void packUInt32(uint8_t *bytes, const uint32_t value)
	{
		bytes[0] = static_cast<uint8_t>(value);
		bytes[1] = static_cast<uint8_t>(value >> 8);
		bytes[2] = static_cast<uint8_t>(value >> 16);
		bytes[3] = static_cast<uint8_t>(value >> 24);
	}

	/// @brief Unpack a little-endian uint32_t from bytes
	/// @brief e.g. 0x78, 0x56, 0x34, 0x12 gives 0x12345678
	/// @param bytes The 4 bytes, least significant first
	/// @return The value
	static uint32_t unpackUInt32(const uint8_t *bytes)
	{
		return static_cast<uint32_t>(bytes[0]) |
			   (static_cast<uint32_t>(bytes[1]) << 8) |
			   (static_cast<uint32_t>(bytes[2]) << 16) |
			   (static_cast<uint32_t>(bytes[3]) << 24);
	}

	/// @brief Report an error through the error callback, if there is one
	/// @param error The error
	void triggerErrorCallback(const Error error)
	{
		if (_errorCallback != nullptr)
			_errorCallback(error);
	}

	/// @brief Send a message (header and payload) small enough for the receiver's DMA buffer as one frame.
	/// @brief A CRC is also appended to the end of the message to help validate the data was sent successfully
	/// @param messageType The message's type
	/// @param payload The payload's bytes
	/// @param payloadSize Number of payload bytes, up to MAX_ONE_STEP_PAYLOAD_SIZE
	/// @return True if sending started
	bool sendOneStep(const Send_Message_Type messageType, const uint8_t *payload, const uint32_t payloadSize)
	{
		// Built in permanent memory, not a local array: the DMA is still reading it after
		// this function returns, when a local array's memory would already be reused
		uint8_t *bytes = _messageToSend.buffer;
		bytes[HEADER_START_BYTE_INDEX] = HEADER_START_BYTE;
		bytes[MESSAGE_TYPE_INDEX] = static_cast<uint8_t>(messageType);
		packUInt32(&bytes[PAYLOAD_SIZE_INDEX], payloadSize);

		// memcpy is undefined for a nullptr source, even when copying nothing
		if (payloadSize > 0)
			memcpy(&bytes[HEADER_SIZE], payload, payloadSize);

		const uint32_t headerAndPayloadSize = HEADER_SIZE + payloadSize;

		// Calc and append the crc32
		const uint32_t crc32 = calcCRC32(bytes, headerAndPayloadSize);
		packUInt32(&bytes[headerAndPayloadSize], crc32);

		return _comms.sendBytes(bytes, headerAndPayloadSize + CRC32_SIZE);
	}

	/// @brief Send a message too big for the receiver's DMA buffer as two frames: the header, then the payload.
	/// @brief A CRC is also appended to the end of the message to help validate the data was sent successfully
	/// @param messageType The message's type
	/// @param payload The payload's bytes. They must stay unchanged until the comms' waitForTXIdle returns
	/// @param payloadSize Number of payload bytes, more than MAX_ONE_STEP_PAYLOAD_SIZE
	/// @return True if every part was sent or started
	bool sendTwoStep(const Send_Message_Type messageType, const uint8_t *payload, const uint32_t payloadSize)
	{
		// Built in permanent memory, not a local array: the CRC placed in it below is still being read by
		// the DMA after this function returns, when a local array's memory would already be reused
		uint8_t *header = _messageToSend.buffer;
		header[HEADER_START_BYTE_INDEX] = HEADER_START_BYTE;
		header[MESSAGE_TYPE_INDEX] = static_cast<uint8_t>(messageType);
		packUInt32(&header[PAYLOAD_SIZE_INDEX], payloadSize);

		if (!_comms.sendBytes(header, HEADER_SIZE))
			return false;

		_comms.waitForTXIdle();
		_comms.stopSendFrame();
		delayMicroseconds(STEP_TRANSITION_DELAY_MICROS);
		_comms.startSendFrame();

		if (!_comms.sendBytes(payload, payloadSize))
			return false;

		// The CRC covers the header and the payload, as one message
		const uint32_t headerCRC32 = calcCRC32(header, HEADER_SIZE);
		const uint32_t crc32 = calcCRC32(payload, payloadSize, headerCRC32);

		// Placed after the header, so the header is left intact for its CRC above
		uint8_t *crc32Bytes = &_messageToSend.buffer[HEADER_SIZE];
		packUInt32(crc32Bytes, crc32);

		// The payload's transfer must finish before the CRC's starts on the same DMA channel
		_comms.waitForTXIdle();
		return _comms.sendBytes(crc32Bytes, CRC32_SIZE);
	}

	/// @brief Handle a frame starting or ending on the receiving FL line. Called from the line's interrupt, so it
	/// @brief must be quick
	void handleReceiveFrameChange()
	{
		if (_comms.frameSending())
		{
			// Step 1: a frame is starting. A two-step payload's receive was started when its header frame ended
			if (_expectedFrame == Expected_Frame::NewMessage)
				_comms.startReceiveBytes(_frameBuffer, DMA_BUFFER_SIZE);
		}
		else
		{
			// The frame has ended, but its receive is still waiting for bytes if the frame was shorter than the
			// size it was started with, so it's stopped before anything else can land in the buffer
			_comms.stopReceiveBytes();

			// The line is idle, so anything left in the RX FIFO is from this frame and would otherwise be received
			// as the next frame's first bytes (e.g. bytes a full buffer had no room for)
			_comms.clearRXFIFO();

			if (_expectedFrame == Expected_Frame::NewMessage)
			{
				// Step 2: a new message's frame has been received. It's only taken if the last message has been
				// handled, and if it starts with a header (the payload frame of a dropped two-step message doesn't)
				if (_messageReceived)
					_lastError = Error::MessageDropped;

				else if (_frameBuffer[HEADER_START_BYTE_INDEX] != HEADER_START_BYTE)
					_lastError = Error::InvalidStartByte;

				else
				{
					memcpy(_receivedMessage.oneStepBuffer, _frameBuffer, DMA_BUFFER_SIZE);

					const uint32_t payloadSize = unpackUInt32(&_receivedMessage.oneStepBuffer[PAYLOAD_SIZE_INDEX]);
					if (payloadSize <= MAX_ONE_STEP_PAYLOAD_SIZE)
						_messageReceived = true;

					else if (payloadSize <= MAX_TWO_STEP_PAYLOAD_SIZE)
					{
						// Step 3: the frame was a two-step header. The payload's receive starts now, so it's ready
						// before the payload frame starts
						_comms.startReceiveBytes(_receivedMessage.twoStepPayloadBuffer, payloadSize + CRC32_SIZE);
						_expectedFrame = Expected_Frame::TwoStepPayload;
					}
					else
						_lastError = Error::PayloadMaxSizeExceeded;
				}
			}
			else
			{
				// Step 4: a two-step payload has been received
				_messageReceived = true;
				_expectedFrame = Expected_Frame::NewMessage;
			}
		}
	}

	/// @brief The receiving FL line's interrupt. Passes each change to the messenger it belongs to
	/// @param messenger The messenger, as given to the comms' attachReceiveFrameInterrupt
	static void receiveFrameInterrupt(void *messenger)
	{
		static_cast<Data_Messenger *>(messenger)->handleReceiveFrameChange();
	}

public:
	/// @brief Create a messenger that sends and receives through a comms
	/// @param comms The comms, created with its MCU's config. It must outlive the messenger
	explicit Data_Messenger(Comms &comms) : _comms(comms)
	{
	}

	~Data_Messenger()
	{
		if (_receivedMessage.twoStepPayloadBuffer != nullptr)
		{
			_comms.releasePayloadBuffer(_receivedMessage.twoStepPayloadBuffer);
			_receivedMessage.twoStepPayloadBuffer = nullptr;
		}
	}

	/// @brief Set up the messenger and its comms
	/// @param receiveCallback Called with each valid message received. Required
	/// @param errorCallback Called with each error. Optional: nullptr if errors aren't wanted
	/// @return True if set up, false if there's no receive callback, the comms couldn't be set up, or the payload
	/// 		buffer couldn't be reserved
	bool setup(const On_Receive_Callback receiveCallback, const On_Error_Callback errorCallback = nullptr)
	{
		if (receiveCallback == nullptr)
			return false;

		// The comms is set up first, as it checks everything the buffer below relies on (e.g. PSRAM being fitted)
		if (!_comms.setup())
			return false;

		// The CRC follows the payload into the same buffer
		_receivedMessage.twoStepPayloadBuffer = _comms.reservePayloadBuffer(MAX_TWO_STEP_PAYLOAD_SIZE + CRC32_SIZE);
		if (_receivedMessage.twoStepPayloadBuffer == nullptr)
			return false;

		// Anything received while setting up isn't part of a frame
		_comms.clearRXFIFO();

		_receiveCallback = receiveCallback;
		_errorCallback = errorCallback;

		// Attached once everything the interrupt uses is set up, so the first frame can be handled
		_comms.attachReceiveFrameInterrupt(receiveFrameInterrupt, this);

		return true;
	}

	/// @brief Send a message. Returns once every byte has been sent, so the payload can be changed straight away
	/// @param messageType The message's type
	/// @param payload The payload's bytes
	/// @param payloadSize Number of payload bytes
	/// @return True if sent, false if a part of it couldn't be sent
	bool send(const Send_Message_Type messageType, const uint8_t *payload, const uint32_t payloadSize)
	{
		if (payloadSize > MAX_TWO_STEP_PAYLOAD_SIZE)
			return false;

		logValue(F("Messenger sending. Message type = "), static_cast<uint8_t>(messageType));
		logValue(F("Messenger sending. Payload size = "), payloadSize);

		_comms.startSendFrame();

		// Decide whether it will be sent in one or two steps - this is done purely by payload size
		if (payloadSize <= MAX_ONE_STEP_PAYLOAD_SIZE)
		{
			if (!sendOneStep(messageType, payload, payloadSize))
			{
				_comms.stopSendFrame();
				return false;
			}
		}
		else if (!sendTwoStep(messageType, payload, payloadSize))
		{
			_comms.stopSendFrame();
			return false;
		}

		// The frame only ends once its last byte has been sent
		_comms.waitForTXIdle();

		_comms.stopSendFrame();
		return true;
	}

	/// @brief Check for a received message. If one has arrived and is valid, the receive callback is called with it.
	/// @brief Errors, including an invalid message, are reported through the error callback
	void checkForMessages()
	{
		// Errors found by the interrupt are reported here, as the callback can't be called from an interrupt.
		// Note: Errors relate to frames that were rejected, not a message being held, so checking message received still happens
		if (_lastError != Error::None)
		{
			triggerErrorCallback(_lastError);
			_lastError = Error::None;
		}

		if (!_messageReceived)
			return;

		const uint8_t *header = _receivedMessage.oneStepBuffer;
		const uint32_t payloadSize = unpackUInt32(&header[PAYLOAD_SIZE_INDEX]);

		// A one-step payload and its CRC follow the header; a two-step payload and its CRC were received into
		// their own buffer
		const uint8_t *payload = nullptr;
		const uint8_t *crc32Bytes = nullptr;

		if (payloadSize == 0)
			crc32Bytes = &header[HEADER_SIZE];

		else if (payloadSize <= MAX_ONE_STEP_PAYLOAD_SIZE)
		{
			payload = &header[HEADER_SIZE];
			crc32Bytes = &payload[payloadSize];
		}
		else
		{
			payload = _receivedMessage.twoStepPayloadBuffer;
			crc32Bytes = &payload[payloadSize];
		}

		// The CRC covers the header and the payload, as one message
		const uint32_t headerCRC32 = calcCRC32(header, HEADER_SIZE);
		const uint32_t crc32 = calcCRC32(payload, payloadSize, headerCRC32);

		if (crc32 == unpackUInt32(crc32Bytes))
		{
			// The sender may have been built with message types this end doesn't know
			Receive_Message_Type messageType = Receive_Message_Type::MIN;
			if (parseEnum(header[MESSAGE_TYPE_INDEX], messageType))
			{
				logValue(F("Messenger received. Message type = "), header[MESSAGE_TYPE_INDEX]);
				logValue(F("Messenger received. Payload size = "), payloadSize);

				_receiveCallback(messageType, payload, payloadSize);
			}
			else
				triggerErrorCallback(Error::UnknownMessageType);
		}
		else
			triggerErrorCallback(Error::InvalidCRC);

		// The buffers are free for the next message
		_messageReceived = false;
	}
};
