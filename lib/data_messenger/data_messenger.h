#pragma once

#include <Arduino.h>
#include <type_traits>
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

/// @brief Used to organise transmitting and receiving data from one MCU to another over a serial link. It
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
		None = 0,				  // No error since the last report
		SendAsyncMessageFail,	  // Async message failed to send
		ReceivedMessageDropped,	  // A message arrived before the last one was handled
		InvalidStartByte,		  // A frame didn't start with a header
		UnknownMessageType,		  // The message's type is outside the receive message types' MIN to MAX
		PayloadMaxSizeExceeded,	  // The header announced more than the largest payload that can be received
		InvalidCRC,				  // The message failed its CRC check
		UnknownPayloadType,		  // The message's payload type is neither parameters nor a payload
		PayloadCallbackUndefined, // A payload arrived, but no payload callback was set up
		InvalidParameters,		  // The message's parameters couldn't be parsed
	};

	// Called with each error, from process
	typedef void (*On_Error_Callback)(const Error error);

	// Called with each async sent message
	typedef void (*On_Sent_Callback)(const Send_Message_Type messageType);

	/// @brief One parameter of a parameterised message. Each is built from a plain value, so an array of
	/// @brief parameters can be written as plain values, e.g. { count, enabled, ratio }
	class Parameter final
	{
	private:
		// The messenger packs and unpacks parameters, so it reads them directly
		friend class Data_Messenger;

		/// @brief The type of a parameter's value, sent before the value in a parameterised payload
		enum class Value_Type : uint8_t
		{
			Unknown = 0, // Not set yet, e.g. a received parameter before it's decoded
			Bool,
			Uint8,
			Uint16,
			Uint32,
			Float
		};

		/// @brief Get the value type a value is sent as
		/// @brief e.g. uint16_t gives Uint16
		/// @tparam T The value's type: bool, uint8_t, uint16_t, uint32_t or float
		/// @return The value type
		template <typename T>
		static constexpr Value_Type toValueType()
		{
			if constexpr (std::is_same_v<T, bool>)
				return Value_Type::Bool;

			else if constexpr (std::is_same_v<T, uint8_t>)
				return Value_Type::Uint8;

			else if constexpr (std::is_same_v<T, uint16_t>)
				return Value_Type::Uint16;

			else if constexpr (std::is_same_v<T, uint32_t>)
				return Value_Type::Uint32;

			else if constexpr (std::is_same_v<T, float>)
				return Value_Type::Float;

			else
				static_assert(sizeof(T) == 0, "Unsupported parameter type");
		}

		/// @brief A parameter's value, as any of the value types. Sized by the compiler for the largest
		union Value
		{
			bool boolValue;
			uint8_t uint8Value;
			uint16_t uint16Value;
			uint32_t uint32Value;
			float floatValue;

			// The largest member is zeroed, so every byte has a known value before a value is set
			Value() : uint32Value(0) {}

			// One per value type, so a Parameter can set the member that matches its value type
			Value(const bool value) : boolValue(value) {}
			Value(const uint8_t value) : uint8Value(value) {}
			Value(const uint16_t value) : uint16Value(value) {}
			Value(const uint32_t value) : uint32Value(value) {}
			Value(const float value) : floatValue(value) {}
		};

		Value_Type _valueType;
		Value _value;

		/// @brief Create an empty parameter, for the messenger to set once it decodes a received value
		Parameter() : _valueType(Value_Type::Unknown), _value() {}

		/// @brief Set the parameter's value, with the value type that matches it
		/// @tparam T The value's type: bool, uint8_t, uint16_t, uint32_t or float
		/// @param value The value
		template <typename T>
		void set(const T value)
		{
			_valueType = toValueType<T>();
			_value = Value(value);
		}

		/// @brief Get the packed size of the parameter's value
		/// @brief e.g. a uint16_t parameter gives 2
		/// @return The value's packed size (bytes), or 0 if the value type isn't known
		uint8_t getValueSize() const
		{
			switch (_valueType)
			{
			case Value_Type::Bool:
				return sizeof(uint8_t); // A bool is packed as one byte
			case Value_Type::Uint8:
				return sizeof(uint8_t);
			case Value_Type::Uint16:
				return sizeof(uint16_t);
			case Value_Type::Uint32:
				return sizeof(uint32_t);
			case Value_Type::Float:
				return sizeof(float);
			default:
				return 0;
			}
		}

	public:
		/// @brief Create a parameter from a value, with the value type that matches it
		/// @tparam T The value's type: bool, uint8_t, uint16_t, uint32_t or float
		/// @param value The value
		template <typename T>
		Parameter(const T value) : _valueType(toValueType<T>()), _value(value) {}

		/// @brief Get the parameter's value
		/// @tparam T The parameter's type: bool, uint8_t, uint16_t, uint32_t or float
		/// @param value Receives the value. Only changed if T is the parameter's type
		/// @return True if T is the parameter's type
		template <typename T>
		bool getValue(T &value) const
		{
			// T must be the same type as the parameter value
			if (_valueType == toValueType<T>())
			{
				if constexpr (std::is_same_v<T, bool>)
					value = _value.boolValue;

				else if constexpr (std::is_same_v<T, uint8_t>)
					value = _value.uint8Value;

				else if constexpr (std::is_same_v<T, uint16_t>)
					value = _value.uint16Value;

				else if constexpr (std::is_same_v<T, uint32_t>)
					value = _value.uint32Value;

				else if constexpr (std::is_same_v<T, float>)
					value = _value.floatValue;

				return true;
			}

			return false;
		}
	};

	// Called with each valid message received. parameters is nullptr when parameterCount is 0, and the parameters
	// are only valid during the call
	typedef void (*On_Receive_Callback)(const Receive_Message_Type messageType, const Parameter *parameters,
										const uint8_t parameterCount);

	// Called with each valid payload received. payload is nullptr when payloadSize is 0
	typedef void (*On_Receive_Payload_Callback)(const Receive_Message_Type messageType, const uint8_t *payload,
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

	// Parameter layout
	static constexpr uint8_t PARAMETER_VALUE_TYPE_INDEX = 0;
	static constexpr uint8_t PARAMETER_VALUE_INDEX = PARAMETER_VALUE_TYPE_INDEX + sizeof(typename Parameter::Value_Type);

	static constexpr uint8_t MAX_PARAMETER_COUNT = 8;
	static constexpr uint8_t MAX_PARAMETER_SIZE = PARAMETER_VALUE_INDEX + sizeof(typename Parameter::Value);

	// Parameterised messages are always sent in one step
	static_assert(MAX_PARAMETER_COUNT * MAX_PARAMETER_SIZE <= MAX_ONE_STEP_PAYLOAD_SIZE,
				  "The maximum count of parameters must fit within a one-step payload");

	Comms &_comms;

	volatile bool _messageReceived = false;

	TX_Message_Async _messageSendingAsync = {};

	// Set by the receiving FL line's interrupt, and reported by process
	volatile Error _lastError = Error::None;

	On_Receive_Callback _receiveCallback = nullptr;
	On_Receive_Payload_Callback _receivePayloadCallback = nullptr;
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
			sentMessage.callback(sentMessage.messageType);
	}

	/// @brief Pack parameters into bytes, in the order given, each as its value type followed by its value
	/// @brief e.g. the uint8_t 5 then the bool true give 0x02, 0x05, 0x01, 0x01
	/// @param bytes Where the parameters go. There must be room for every parameter's value type and value
	/// @param parameters The parameters
	/// @param parameterCount Number of parameters, up to MAX_PARAMETER_COUNT
	/// @return The packed parameters' size (bytes)
	static uint32_t packParameters(uint8_t *const bytes, const Parameter *const parameters, const uint8_t parameterCount)
	{
		uint32_t packedSize = 0;

		for (uint8_t parameterIndex = 0; parameterIndex < parameterCount; ++parameterIndex)
		{
			const Parameter &parameter = parameters[parameterIndex];
			uint8_t *parameterBytes = &bytes[packedSize];

			parameterBytes[PARAMETER_VALUE_TYPE_INDEX] = static_cast<uint8_t>(parameter._valueType);

			uint8_t *valueBytes = &parameterBytes[PARAMETER_VALUE_INDEX];
			switch (parameter._valueType)
			{
			case Parameter::Value_Type::Unknown:
				// Nothing to pack - this should never be reached
				break;
			case Parameter::Value_Type::Bool:
				packValue(valueBytes, parameter._value.boolValue);
				break;
			case Parameter::Value_Type::Uint8:
				packValue(valueBytes, parameter._value.uint8Value);
				break;
			case Parameter::Value_Type::Uint16:
				packValue(valueBytes, parameter._value.uint16Value);
				break;
			case Parameter::Value_Type::Uint32:
				packValue(valueBytes, parameter._value.uint32Value);
				break;
			case Parameter::Value_Type::Float:
				packValue(valueBytes, parameter._value.floatValue);
				break;
			}

			packedSize += PARAMETER_VALUE_INDEX + parameter.getValueSize();
		}

		return packedSize;
	}

	/// @brief Unpack a received payload of parameters, each a value type followed by its value
	/// @brief e.g. 0x02, 0x05, 0x01, 0x01 gives a uint8_t parameter of 5 then a bool parameter of true
	/// @param payload The payload
	/// @param payloadSize The payload's size (bytes)
	/// @param parameters Receives the parameters. There must be room for MAX_PARAMETER_COUNT
	/// @param parameterCount Receives the number of parameters. Only changed if the payload is parsed
	/// @return True if every value type is known, there are no more than MAX_PARAMETER_COUNT parameters, and the
	/// 		last one ends exactly at the payload's end
	static bool unpackParameters(const uint8_t *const payload, const uint32_t payloadSize,
								 Parameter *const parameters, uint8_t &parameterCount)
	{
		// A value is unpacked before its size is known, so the last one can read past the payload's end. The CRC
		// always follows the payload, so the read stays within the message
		static_assert(sizeof(typename Parameter::Value) <= CRC32_SIZE, "A value must fit within the CRC");

		uint8_t foundCount = 0;
		uint32_t payloadByteIndex = 0;

		while ((payloadByteIndex < payloadSize) && (foundCount < MAX_PARAMETER_COUNT))
		{
			const uint8_t *parameterBytes = &payload[payloadByteIndex];
			const uint8_t *valueBytes = &parameterBytes[PARAMETER_VALUE_INDEX];
			Parameter &parameter = parameters[foundCount];

			switch (static_cast<typename Parameter::Value_Type>(parameterBytes[PARAMETER_VALUE_TYPE_INDEX]))
			{
			case Parameter::Value_Type::Bool:
				parameter.set(unpackValue<bool>(valueBytes));
				break;
			case Parameter::Value_Type::Uint8:
				parameter.set(unpackValue<uint8_t>(valueBytes));
				break;
			case Parameter::Value_Type::Uint16:
				parameter.set(unpackValue<uint16_t>(valueBytes));
				break;
			case Parameter::Value_Type::Uint32:
				parameter.set(unpackValue<uint32_t>(valueBytes));
				break;
			case Parameter::Value_Type::Float:
				parameter.set(unpackValue<float>(valueBytes));
				break;
			default:
				return false; // Unknown, or not a value type
			}

			payloadByteIndex += PARAMETER_VALUE_INDEX + parameter.getValueSize();
			++foundCount;
		}

		if (payloadByteIndex != payloadSize)
			return false;

		parameterCount = foundCount;
		return true;
	}

	/// @brief Send a message (header and payload) small enough for the receiver's frame buffer as one frame.
	/// @brief A CRC is also appended to the end of the message to help validate the data was sent successfully
	/// @param messageType The message's type
	/// @param payloadType How the payload's bytes are laid out
	/// @param payload The payload's bytes, or nullptr when sending parameters
	/// @param payloadSize Number of payload bytes, up to MAX_ONE_STEP_PAYLOAD_SIZE, or 0 when sending parameters
	/// @param parameters The parameters, or nullptr when sending a payload
	/// @param parameterCount Number of parameters, up to MAX_PARAMETER_COUNT, or 0 when sending a payload
	/// @param callback Called from process once an async send has been sent, or nullptr if the caller waits for it
	/// @return True if sending started
	bool sendOneStep(const Send_Message_Type messageType, const Payload_Type payloadType,
					 const uint8_t *payload, const uint32_t payloadSize,
					 const Parameter *const parameters, const uint8_t parameterCount,
					 const On_Sent_Callback callback)
	{
		// Built in permanent memory, not a local array: the comms is still sending from it after
		// this function returns, when a local array's memory would already be reused
		uint8_t *bytes = _messageToSend.buffer;
		bytes[HEADER_START_BYTE_INDEX] = HEADER_START_BYTE;
		bytes[MESSAGE_TYPE_INDEX] = static_cast<uint8_t>(messageType);
		bytes[PAYLOAD_TYPE_INDEX] = static_cast<uint8_t>(payloadType);

		// The payload size goes in the header once it's known, as parameters only have a size once packed
		uint32_t framePayloadSize = 0;

		if (payloadType == Payload_Type::Parameterised)
			framePayloadSize = packParameters(&bytes[HEADER_SIZE], parameters, parameterCount);

		else
		{
			// memcpy is undefined for a nullptr source, even when copying nothing
			if (payloadSize > 0)
				memcpy(&bytes[HEADER_SIZE], payload, payloadSize);

			framePayloadSize = payloadSize;
		}

		packValue(&bytes[PAYLOAD_SIZE_INDEX], framePayloadSize);

		const uint32_t headerAndPayloadSize = HEADER_SIZE + framePayloadSize;

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

	/// @brief Send a message, in one step or two by its payload size. Parameters always fit in one step
	/// @param messageType The message's type
	/// @param payload The payload's bytes, or nullptr when sending parameters
	/// @param payloadSize Number of payload bytes, up to MAX_TWO_STEP_PAYLOAD_SIZE, or 0 when sending parameters
	/// @param parameters The parameters, or nullptr when sending a payload
	/// @param parameterCount Number of parameters, up to MAX_PARAMETER_COUNT, or 0 when sending a payload
	/// @param callback Called from process once an async send has been sent, or nullptr to return only once every
	/// 		byte has been sent
	/// @return True if sent, or for an async send, started. False if not (e.g. an async send is still in progress)
	bool send(const Send_Message_Type messageType,
			  const uint8_t *payload, const uint32_t payloadSize,
			  const Parameter *const parameters, const uint8_t parameterCount,
			  const On_Sent_Callback callback)
	{
		if (payloadSize > MAX_TWO_STEP_PAYLOAD_SIZE)
			return false;

		// If there is a message being sent then reject this one..
		if (_messageSendingAsync.callback != nullptr)
			return false;

		logValue(F("Messenger sending. Message type = "), static_cast<uint8_t>(messageType));

		const Payload_Type payloadType = (payloadSize > 0) ? Payload_Type::Raw : Payload_Type::Parameterised;

		if (payloadType == Payload_Type::Parameterised)
			logValue(F("Messenger sending. Parameter count = "), parameterCount);
		else
			logValue(F("Messenger sending. Payload size = "), payloadSize);

		_comms.startSendFrame();

		// Decide whether it will be sent in one or two steps. Parameters should always fit in one step
		if ((payloadType == Payload_Type::Parameterised) || (payloadSize <= MAX_ONE_STEP_PAYLOAD_SIZE))
		{
			if (!sendOneStep(messageType, payloadType, payload, payloadSize, parameters, parameterCount, callback))
			{
				_comms.stopSendFrame();
				return false;
			}
		}
		// If we have parameters (which shouldn't be possible at this point) return false.
		// We only support parameters in one step
		else if ((parameterCount > 0) || !sendTwoStep(messageType, payloadType, payload, payloadSize, callback))
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

	/// @brief Finish an async send once its payload has gone: send a two-step message's CRC, end the frame, and
	/// @brief report it sent
	void processSent()
	{
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
	}

	/// @brief Validate a received message and pass it to the callback for its payload type. Anything invalid is
	/// @brief reported through the error callback
	void processReceived()
	{
		if (!_messageReceived)
			return;

		const uint8_t *header = _receivedMessage.oneStepBuffer;
		const uint32_t payloadSize = unpackValue<uint32_t>(&header[PAYLOAD_SIZE_INDEX]);

		// Get the payload (this will depend if it's one-step or two-step)
		const uint8_t *payload = (payloadSize <= MAX_ONE_STEP_PAYLOAD_SIZE)
									 ? &header[HEADER_SIZE]					  // One-step
									 : _receivedMessage.twoStepPayloadBuffer; // Two-step

		// Get the CRC. This is the last value in the message (at the end of the payload)
		const uint8_t *crc32Bytes = &payload[payloadSize];

		// Calculate the CRC from the data received and see if it matches the CRC in the message
		const uint32_t headerCRC32 = calcCRC32(header, HEADER_SIZE);
		const uint32_t crc32 = calcCRC32(payload, payloadSize, headerCRC32);

		if (crc32 == unpackValue<uint32_t>(crc32Bytes))
		{
			Receive_Message_Type messageType = Receive_Message_Type::MIN;
			if (parseEnum(header[MESSAGE_TYPE_INDEX], messageType))
			{
				logValue(F("Messenger received. Message type = "), header[MESSAGE_TYPE_INDEX]);
				logValue(F("Messenger received. Payload size = "), payloadSize);

				// Unpack (and validate) message data based on payload type
				switch (static_cast<Payload_Type>(header[PAYLOAD_TYPE_INDEX]))
				{
				case Payload_Type::Parameterised:
					if (payloadSize == 0)
						_receiveCallback(messageType, nullptr, 0);
					else
					{
						Parameter parameters[MAX_PARAMETER_COUNT];
						uint8_t parameterCount;

						if (unpackParameters(payload, payloadSize, parameters, parameterCount))
							_receiveCallback(messageType, parameters, parameterCount);
						else
							triggerErrorCallback(Error::InvalidParameters);
					}
					break;

				case Payload_Type::Raw:
					if (_receivePayloadCallback != nullptr)
						_receivePayloadCallback(messageType, payload, payloadSize);
					else
						triggerErrorCallback(Error::PayloadCallbackUndefined);
					break;

				default:
					triggerErrorCallback(Error::UnknownPayloadType);
					break;
				}
			}
			else
				triggerErrorCallback(Error::UnknownMessageType);
		}
		else
			triggerErrorCallback(Error::InvalidCRC);

		// The buffers are free for the next message
		_messageReceived = false;
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
	/// @param receivePayloadCallback Called with each valid payload received.
	///        Optional: nullptr if payloads aren't expected
	/// @param errorCallback Called with each error. Optional: nullptr if errors aren't wanted
	/// @return True if set up, false if there's no receive callback, the comms couldn't be set up, or the payload
	/// 		buffer couldn't be reserved
	bool setup(const On_Receive_Callback receiveCallback,
			   const On_Receive_Payload_Callback receivePayloadCallback = nullptr,
			   const On_Error_Callback errorCallback = nullptr)
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
		_receivePayloadCallback = receivePayloadCallback;
		_errorCallback = errorCallback;

		// Attached once everything the interrupt uses is set up, so the first frame can be handled
		_comms.attachReceiveFrameInterrupt(receiveFrameInterrupt, this);

		return true;
	}

	/// @brief Send a message with no parameters. Returns once every byte has been sent
	/// @param messageType The message's type
	/// @return True if sent
	bool send(const Send_Message_Type messageType)
	{
		return send(messageType, nullptr, 0, nullptr, 0, nullptr);
	}

	/// @brief Send a message with parameters. Returns once every byte has been sent
	/// @tparam PARAMETER_COUNT Number of parameters, from 1 up to MAX_PARAMETER_COUNT. Inferred from the array
	/// @param messageType The message's type
	/// @param parameters The parameters, e.g. { count, enabled, ratio }
	/// @return True if sent
	template <uint8_t PARAMETER_COUNT>
	bool send(const Send_Message_Type messageType, const Parameter (&parameters)[PARAMETER_COUNT])
	{
		static_assert(PARAMETER_COUNT <= MAX_PARAMETER_COUNT, "Too many parameters for one message");

		return send(messageType, nullptr, 0, parameters, PARAMETER_COUNT, nullptr);
	}

	/// @brief Send a message with no parameters in the background. Returns once sending has started; process
	/// @brief finishes it
	/// @param messageType The message's type
	/// @param callback Called from process once every byte has been sent. Required
	/// @return True if sending started, false if not (e.g. an async send is still in progress)
	bool sendAsync(const Send_Message_Type messageType, const On_Sent_Callback callback)
	{
		return (callback != nullptr) && send(messageType, nullptr, 0, nullptr, 0, callback);
	}

	/// @brief Send a message with parameters in the background. Returns once sending has started; process
	/// @brief finishes it. The parameters are packed before this returns, so they can be changed straight away
	/// @tparam PARAMETER_COUNT Number of parameters, from 1 up to MAX_PARAMETER_COUNT. Inferred from the array
	/// @param messageType The message's type
	/// @param parameters The parameters, e.g. { count, enabled, ratio }
	/// @param callback Called from process once every byte has been sent. Required
	/// @return True if sending started, false if not (e.g. an async send is still in progress)
	template <uint8_t PARAMETER_COUNT>
	bool sendAsync(const Send_Message_Type messageType, const Parameter (&parameters)[PARAMETER_COUNT],
				   const On_Sent_Callback callback)
	{
		static_assert(PARAMETER_COUNT <= MAX_PARAMETER_COUNT, "Too many parameters for one message");

		return (callback != nullptr) && send(messageType, nullptr, 0, parameters, PARAMETER_COUNT, callback);
	}

	/// @brief Send a payload. Returns once every byte has been sent, so the payload can be changed straight away
	/// @param messageType The message's type
	/// @param payload The payload's bytes
	/// @param payloadSize Number of payload bytes, from 1
	/// @return True if sent, false if not (e.g. the payload is empty or too big)
	bool sendPayload(const Send_Message_Type messageType, const uint8_t *payload, const uint32_t payloadSize)
	{
		return (payloadSize > 0) && send(messageType, payload, payloadSize, nullptr, 0, nullptr);
	}

	/// @brief Send a payload in the background. Returns once sending has started; process finishes it
	/// @param messageType The message's type
	/// @param payload The payload's bytes. They must stay unchanged until callback is called
	/// @param payloadSize Number of payload bytes, from 1
	/// @param callback Called from process once every byte has been sent. Required
	/// @return True if sending started, false if not (e.g. the payload is empty, or an async send is still in
	/// 		progress)
	bool sendPayloadAsync(const Send_Message_Type messageType, const uint8_t *payload,
						  const uint32_t payloadSize, const On_Sent_Callback callback)
	{
		return (payloadSize > 0) && (callback != nullptr) &&
			   send(messageType, payload, payloadSize, nullptr, 0, callback);
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

		processSent();

		processReceived();
	}
};
