#pragma once

#include <Arduino.h>
#include "common.debug.h"
#include "common.data.packing.h"
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

/// @brief The standard CRC-32 lookup table: one entry for each value of a byte. Each entry is what processing its
/// @brief byte value's 8 bits, one at a time, does to the CRC register, so a whole byte can be processed in one step
/// @brief e.g. entry 1 is 0x77073096
struct CRC32_Table final
{
	static constexpr uint16_t SIZE = 256;
	uint32_t entries[SIZE] = {};

	/// @brief Populate the table, at compile time when the table is constexpr
	constexpr CRC32_Table()
	{
		// The polynomial 0x04C11DB7 reversed, as each byte is processed least significant bit first
		constexpr uint32_t POLYNOMIAL = 0xEDB88320;

		for (uint16_t byteValue = 0; byteValue < SIZE; ++byteValue)
		{
			uint32_t crc = byteValue;

			for (uint8_t iBit = 0; iBit < 8; ++iBit)
			{
				const bool lowBitSet = ((crc & 1) != 0);
				crc >>= 1;

				if (lowBitSet)
					crc ^= POLYNOMIAL;
			}

			entries[byteValue] = crc;
		}
	}
};

/// @brief Used to organise transmitting and receiving data from one mcu to another over a serial link. It
/// @brief decides when everything happens; its comms knows how to do it on its MCU
/// @tparam Comms The MCU's comms (e.g. RP235x_Data_Messenger_Comms): the MCU-specific part that carries out what
/// 		the messenger decides. It sends and receives bytes over its serial link in the background, signals and watches
///			the FL lines, and reserves the payload buffer, so the messenger itself works unchanged on any MCU
/// @tparam Send_Message_Type The enum of message types this end sends: enum class with a uint8_t underlying
/// 		type, and MIN and MAX values aliasing its first and last types
/// @tparam Receive_Message_Type The enum of message types this end receives, in the same form. Received types
/// 		outside MIN to MAX are reported as UnknownMessageType
/// @tparam MAX_TWO_STEP_PAYLOAD_SIZE The largest payload this end sends or receives (bytes). It must be larger than
/// 		the largest one-step payload. The receive buffer is reserved at this size. The default is 1 KB
template <typename Comms, typename Send_Message_Type, typename Receive_Message_Type,
		  uint32_t MAX_TWO_STEP_PAYLOAD_SIZE = 1024>
class Data_Messenger
{
public:
	/// @brief The errors a messenger reports
	enum class Error : uint8_t
	{
		None = 0,				// No error since the last report
		SendAsyncMessageFail,	// Async message failed to send
		ReceivedMessageDropped, // A message arrived before the last one was handled
		InvalidStartByte,		// A frame didn't start with a header
		UnknownMessageType,		// The message's type is outside the receive message types' MIN to MAX
		PayloadMaxSizeExceeded, // The header announced more than the largest payload that can be received
		InvalidCRC,				// The message failed its CRC check
	};

	// Called with each error, from process
	typedef void (*On_Error_Callback)(const Error error);

	// Called with each valid message received. payload is nullptr when payloadSize is 0
	typedef void (*On_Receive_Callback)(const Receive_Message_Type messageType, const uint8_t *payload,
										const uint32_t payloadSize);

	// Called with each async sent message. payload is nullptr when payloadSize is 0
	typedef void (*On_Sent_Callback)(const Send_Message_Type messageType, const uint8_t *payload,
									 const uint32_t payloadSize);

private:
	static constexpr uint8_t FRAME_BUFFER_SIZE = 64;
	static constexpr uint8_t HEADER_SIZE = 7; // HEADER_START_BYTE, message type, payload type, payload size (4 bytes);
	static constexpr uint8_t CRC32_SIZE = 4;

	// Header layout
	static constexpr uint8_t HEADER_START_BYTE_INDEX = 0;
	static constexpr uint8_t MESSAGE_TYPE_INDEX = 1;
	static constexpr uint8_t PAYLOAD_TYPE_INDEX = 2;
	static constexpr uint8_t PAYLOAD_SIZE_INDEX = 3;

	// The payload size and the CRC are each packed as a uint32_t
	static_assert(HEADER_SIZE == PAYLOAD_SIZE_INDEX + sizeof(uint32_t), "The payload size must end the header");
	static_assert(CRC32_SIZE == sizeof(uint32_t), "The CRC must be a uint32_t");

	// The message type is sent as one header byte
	static_assert(sizeof(Send_Message_Type) == sizeof(uint8_t), "Send message types must be a uint8_t enum");
	static_assert(sizeof(Receive_Message_Type) == sizeof(uint8_t), "Receive message types must be a uint8_t enum");

	// Calc max buffer size that allows us to send the entire message in one step
	static constexpr uint8_t MAX_ONE_STEP_PAYLOAD_SIZE = FRAME_BUFFER_SIZE - (HEADER_SIZE + CRC32_SIZE);

	// A payload too big for one step is sent in two, so the two-step limit must be above the one-step one
	static_assert(MAX_TWO_STEP_PAYLOAD_SIZE > MAX_ONE_STEP_PAYLOAD_SIZE, "The largest two-step payload must be larger than the largest one-step payload");

	static constexpr uint8_t HEADER_START_BYTE = 0b10101010;

	static constexpr uint16_t STEP_TRANSITION_DELAY_MICROS = 1000;

	// Built at compile time, so it's held in flash
	static constexpr CRC32_Table CRC32_TABLE = {};

	// Both ends of a link share this table, so a wrong one would still pass every test between them. It's
	// checked against the standard's published entries instead
	static_assert(CRC32_TABLE.entries[1] == 0x77073096, "The CRC-32 table doesn't match the standard");
	static_assert(CRC32_TABLE.entries[255] == 0x2D02EF8D, "The CRC-32 table doesn't match the standard");

	/// @brief The frame the receiving FL line's next edges belong to
	enum class Expected_Frame : uint8_t
	{
		NewMessage,		// The first frame of a message: a whole one-step message, or a two-step header
		TwoStepPayload, // The payload frame that follows a two-step header
	};

	/// @brief Permanent received message buffers
	struct RX_Message_Buffers final
	{
		uint8_t oneStepBuffer[FRAME_BUFFER_SIZE] = {}; // Permanent memory, reused for every message
		uint8_t *twoStepPayloadBuffer = nullptr;	   // Permanent memory, reserved once and reused for every message
	};

	/// @brief Permanent sending message buffer
	struct TX_Message_Buffer final
	{
		uint8_t buffer[FRAME_BUFFER_SIZE] = {}; // Permanent memory, reused for every message
	};

	/// @brief Async message being sent
	struct TX_Message_Async final
	{
		bool twoStep = false;
		Send_Message_Type messageType = Send_Message_Type::MIN;
		const uint8_t *payload = nullptr;
		uint32_t payloadSize = 0;
		uint32_t headerCRC32 = 0;
		On_Sent_Callback callback = nullptr;
	};

	/// @brief How a payload's bytes are laid out
	enum class Payload_Type : uint8_t
	{
		Raw = 0,	  // The sender's own layout
		Parameterised // A list of parameters, each a value type followed by its value
	};

	/// @brief The type of a parameter's value, sent before the value in a parameterised payload
	enum class Parameter_Value_Type : uint8_t
	{
		Bool = 0, // 1 byte
		Uint8,	  // 1 byte
		Uint16,	  // 2 bytes
		Uint32,	  // 4 bytes
		Float	  // 4 bytes
	};

	Comms &_comms;

	volatile bool _messageReceived = false;

	TX_Message_Async _messageSendingAsync = {};

	// Set by the receiving FL line's interrupt, and reported by process
	volatile Error _lastError = Error::None;

	On_Receive_Callback _receiveCallback = nullptr;
	On_Error_Callback _errorCallback = nullptr;

	RX_Message_Buffers _receivedMessage = {};
	TX_Message_Buffer _messageToSend = {};

	uint8_t _frameBuffer[FRAME_BUFFER_SIZE] = {};

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
		// The register starts inverted, so leading zero bytes still change the CRC, and
		// ends inverted, as the standard specifies
		constexpr uint32_t INVERT = 0xFFFFFFFF;

		// Undo the previous result's final inversion, to carry on from its register value
		uint32_t crc = prevCRC ^ INVERT;

		for (size_t iByte = 0; iByte < size; ++iByte)
		{
			// The register's low byte, combined with the next byte, picks the entry that applies all 8 bits at once
			const uint8_t tableIndex = static_cast<uint8_t>(crc ^ bytes[iByte]);
			crc = (crc >> 8) ^ CRC32_TABLE.entries[tableIndex];
		}

		return crc ^ INVERT;
	}

	/// @brief Report an error through the error callback, if there is one
	/// @param error The error
	void triggerErrorCallback(const Error error)
	{
		if (_errorCallback != nullptr)
			_errorCallback(error);
	}

	/// @brief Report an async message as sent through its callback, if there is one
	/// @param sentMessage The message that has been sent
	void triggerSentCallback(const TX_Message_Async &sentMessage)
	{
		if (sentMessage.callback != nullptr)
			sentMessage.callback(sentMessage.messageType, sentMessage.payload, sentMessage.payloadSize);
	}

	/// @brief Send a message (header and payload) small enough for the receiver's frame buffer as one frame.
	/// @brief A CRC is also appended to the end of the message to help validate the data was sent successfully
	/// @param messageType The message's type
	/// @param payloadType How the payload's bytes are laid out
	/// @param payload The payload's bytes
	/// @param payloadSize Number of payload bytes, up to MAX_ONE_STEP_PAYLOAD_SIZE
	/// @param callback Called from process once an async send has been sent, or nullptr if the caller waits for it
	/// @return True if sending started
	bool sendOneStep(const Send_Message_Type messageType,
					 const Payload_Type payloadType, const uint8_t *payload, const uint32_t payloadSize,
					 const On_Sent_Callback callback)
	{
		// Built in permanent memory, not a local array: the comms is still sending from it after
		// this function returns, when a local array's memory would already be reused
		uint8_t *bytes = _messageToSend.buffer;
		bytes[HEADER_START_BYTE_INDEX] = HEADER_START_BYTE;
		bytes[MESSAGE_TYPE_INDEX] = static_cast<uint8_t>(messageType);
		bytes[PAYLOAD_TYPE_INDEX] = static_cast<uint8_t>(payloadType);
		packValue(&bytes[PAYLOAD_SIZE_INDEX], payloadSize);

		// memcpy is undefined for a nullptr source, even when copying nothing
		if (payloadSize > 0)
			memcpy(&bytes[HEADER_SIZE], payload, payloadSize);

		const uint32_t headerAndPayloadSize = HEADER_SIZE + payloadSize;

		// Calc and append the crc32
		const uint32_t crc32 = calcCRC32(bytes, headerAndPayloadSize);
		packValue(&bytes[headerAndPayloadSize], crc32);

		if (!_comms.sendBytes(bytes, headerAndPayloadSize + CRC32_SIZE))
			return false;

		if (callback != nullptr)
		{
			_messageSendingAsync.twoStep = false;
			_messageSendingAsync.messageType = messageType;
			_messageSendingAsync.payload = payload;
			_messageSendingAsync.payloadSize = payloadSize;
			_messageSendingAsync.callback = callback;
		}

		return true;
	}

	/// @brief Send a two-step message's CRC, which ends the message. Call it once the payload has been sent, as the
	/// @brief comms sends one buffer at a time
	/// @param payload The payload's bytes, as sent
	/// @param payloadSize Number of payload bytes
	/// @param headerCRC32 The header's CRC, which the message's CRC carries on from
	/// @return True if sending started
	bool sendTwoStepCRC(const uint8_t *payload, const uint32_t payloadSize, const uint32_t headerCRC32)
	{
		// The CRC covers the header and the payload, as one message
		const uint32_t crc32 = calcCRC32(payload, payloadSize, headerCRC32);

		// Placed after the header, so the header is left intact for its CRC above
		uint8_t *crc32Bytes = &_messageToSend.buffer[HEADER_SIZE];
		packValue(crc32Bytes, crc32);

		// The comms sends one buffer at a time, so the payload must finish before the CRC starts
		_comms.waitForTXIdle();
		return _comms.sendBytes(crc32Bytes, CRC32_SIZE);
	}

	/// @brief Send a message too big for the receiver's frame buffer as two frames: the header, then the payload.
	/// @brief A CRC is also appended to the end of the message to help validate the data was sent successfully
	/// @param messageType The message's type
	/// @param payloadType How the payload's bytes are laid out
	/// @param payload The payload's bytes. They must stay unchanged until the whole message has been sent
	/// @param payloadSize Number of payload bytes, more than MAX_ONE_STEP_PAYLOAD_SIZE
	/// @param callback Called from process once an async send has been sent, or nullptr if the caller waits for it.
	/// 		For an async send, the CRC is left for process to send once the payload has been sent
	/// @return True if every part was sent or started
	bool sendTwoStep(const Send_Message_Type messageType,
					 const Payload_Type payloadType, const uint8_t *payload, const uint32_t payloadSize,
					 const On_Sent_Callback callback)
	{
		// Built in permanent memory, not a local array: the comms is still sending the CRC placed in it below
		// after this function returns, when a local array's memory would already be reused
		uint8_t *header = _messageToSend.buffer;
		header[HEADER_START_BYTE_INDEX] = HEADER_START_BYTE;
		header[MESSAGE_TYPE_INDEX] = static_cast<uint8_t>(messageType);
		header[PAYLOAD_TYPE_INDEX] = static_cast<uint8_t>(payloadType);
		packValue(&header[PAYLOAD_SIZE_INDEX], payloadSize);

		if (!_comms.sendBytes(header, HEADER_SIZE))
			return false;

		_comms.waitForTXIdle();
		_comms.stopSendFrame();
		delayMicroseconds(STEP_TRANSITION_DELAY_MICROS);
		_comms.startSendFrame();

		if (!_comms.sendBytes(payload, payloadSize))
			return false;

		const uint32_t headerCRC32 = calcCRC32(header, HEADER_SIZE);

		if (callback != nullptr)
		{
			_messageSendingAsync.twoStep = true;
			_messageSendingAsync.messageType = messageType;
			_messageSendingAsync.payload = payload;
			_messageSendingAsync.payloadSize = payloadSize;
			_messageSendingAsync.headerCRC32 = headerCRC32;
			_messageSendingAsync.callback = callback;
		}
		else if (!sendTwoStepCRC(payload, payloadSize, headerCRC32))
			return false;

		return true;
	}

	/// @brief Send a message, in one step or two by its payload size
	/// @param messageType The message's type
	/// @param payloadType How the payload's bytes are laid out
	/// @param payload The payload's bytes
	/// @param payloadSize Number of payload bytes, up to MAX_TWO_STEP_PAYLOAD_SIZE
	/// @param callback Called from process once an async send has been sent, or nullptr to return only once every
	/// 		byte has been sent
	/// @return True if sent, or for an async send, started. False if not (e.g. an async send is still in progress)
	bool send(const Send_Message_Type messageType,
			  const Payload_Type payloadType, const uint8_t *payload, const uint32_t payloadSize,
			  const On_Sent_Callback callback)
	{
		if (payloadSize > MAX_TWO_STEP_PAYLOAD_SIZE)
			return false;

		// If there is a message being sent then reject this one..
		if (_messageSendingAsync.callback != nullptr)
			return false;

		logValue(F("Messenger sending. Message type = "), static_cast<uint8_t>(messageType));
		logValue(F("Messenger sending. Payload size = "), payloadSize);

		_comms.startSendFrame();

		// Decide whether it will be sent in one or two steps - this is done purely by payload size
		if (payloadSize <= MAX_ONE_STEP_PAYLOAD_SIZE)
		{
			if (!sendOneStep(messageType, payloadType, payload, payloadSize, callback))
			{
				_comms.stopSendFrame();
				return false;
			}
		}
		else if (!sendTwoStep(messageType, payloadType, payload, payloadSize, callback))
		{
			_comms.stopSendFrame();
			return false;
		}

		// If this is being sent asynchronously then record message and don't wait..
		if (callback == nullptr)
		{
			// The frame only ends once its last byte has been sent
			_comms.waitForTXIdle();
			_comms.stopSendFrame();
		}

		return true;
	}

	/// @brief Handle a frame starting or ending on the receiving FL line.
	/// @brief Called from the line's interrupt, so it must be quick
	void handleReceiveFrameChange()
	{
		if (_comms.frameSending())
		{
			// Step 1: a frame is starting. A two-step payload's receive was started when its header frame ended
			if (_expectedFrame == Expected_Frame::NewMessage)
				_comms.startReceiveBytes(_frameBuffer, FRAME_BUFFER_SIZE);
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
					_lastError = Error::ReceivedMessageDropped;

				else if (_frameBuffer[HEADER_START_BYTE_INDEX] != HEADER_START_BYTE)
					_lastError = Error::InvalidStartByte;

				else
				{
					memcpy(_receivedMessage.oneStepBuffer, _frameBuffer, FRAME_BUFFER_SIZE);

					const uint32_t payloadSize = unpackValue<uint32_t>(&_receivedMessage.oneStepBuffer[PAYLOAD_SIZE_INDEX]);
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
	explicit Data_Messenger(Comms &comms) : _comms(comms) {}

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
	/// @return True if sent, false if not (e.g. the payload is too big)
	bool send(const Send_Message_Type messageType, const uint8_t *payload, const uint32_t payloadSize)
	{
		return send(messageType, Payload_Type::Raw, payload, payloadSize, nullptr);
	}

	/// @brief Send a message in the background. Returns once sending has started; process finishes it
	/// @param messageType The message's type
	/// @param payload The payload's bytes. They must stay unchanged until callback is called
	/// @param payloadSize Number of payload bytes
	/// @param callback Called from process once every byte has been sent. Required
	/// @return True if sending started, false if not (e.g. an async send is still in progress)
	bool sendAsync(const Send_Message_Type messageType, const uint8_t *payload,
				   const uint32_t payloadSize, const On_Sent_Callback callback)
	{
		return (callback != nullptr) && send(messageType, Payload_Type::Raw, payload, payloadSize, callback);
	}

	/// @brief Process internal events i.e. check for a received message, check if an async send has completed.
	/// @brief Errors, including an invalid message, are reported through the error callback
	void process()
	{
		// Errors found by the interrupt are reported here, as the callback can't be called from an interrupt.
		// Note: Errors relate to frames that were rejected, not a message being held, so checking message received still happens
		if (_lastError != Error::None)
		{
			triggerErrorCallback(_lastError);
			_lastError = Error::None;
		}

		// Check if there's an async message that has finished sending its payload.
		// If there is, the final thing to do for a two-step message is to send its CRC32
		if ((_messageSendingAsync.callback != nullptr) && !_comms.sendingBytes())
		{
			if (_messageSendingAsync.twoStep && !sendTwoStepCRC(_messageSendingAsync.payload,
																_messageSendingAsync.payloadSize,
																_messageSendingAsync.headerCRC32))
			{
				_comms.stopSendFrame();

				_messageSendingAsync = {};
				triggerErrorCallback(Error::SendAsyncMessageFail);
			}
			else
			{
				_comms.waitForTXIdle();
				_comms.stopSendFrame();

				// Cleared before its callback, so the callback can start the next send
				const TX_Message_Async sentMessage = _messageSendingAsync;
				_messageSendingAsync = {};
				triggerSentCallback(sentMessage);
			}
		}

		if (!_messageReceived)
			return;

		const uint8_t *header = _receivedMessage.oneStepBuffer;
		const uint32_t payloadSize = unpackValue<uint32_t>(&header[PAYLOAD_SIZE_INDEX]);

		// A one-step payload and its CRC follow the header; a two-step payload
		// and its CRC were received into their own buffer
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

		if (crc32 == unpackValue<uint32_t>(crc32Bytes))
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
