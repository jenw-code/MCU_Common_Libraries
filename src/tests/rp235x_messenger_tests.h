#pragma once

// The Data_Messenger test pack. Run it with testMessenger(), from the one .cpp that includes this header (it
// defines the test's global objects)

#include <Arduino.h>
#include "common.debug.h"

#include "data_messenger.h"
#include "rp235x_data_messenger_comms.h"

//---------------------------------------- MESSENGER UNDER TEST ------------------------------------
// The only MCU-specific part. To test another MCU's comms, replace this section

// The two-step payload buffer is in internal RAM, as the Pico 2 has no PSRAM
using CommsClass = RP235x_Data_Messenger_Comms<RP235x_UART_Config, RP235x_UART_Config, false>;

/// @brief The test's message types. The link is looped back, so each end sends and receives the same types
enum class Test_Message_Type : uint8_t
{
	SendOverMaxPayload,
	MessageTypeOnly,
	OneStepSmall,
	OneStepLimit,
	TwoStepLowerLimit,
	TwoStepLarge,
	TwoStepLimit,
	SeveralInARow,
	OneStepDropped,
	TwoStepDropped,
	AsyncOneStep,
	AsyncTwoStep,
	AsyncInProgress,
	ParametersEveryType,
	ParametersMost,
	ParametersAsync,
	RelayIn,
	RelayOut,
	SendTimeMeasurement,
	MIN = SendOverMaxPayload,
	MAX = SendTimeMeasurement,
};

// The largest payload the messenger under test accepts (its MAX_TWO_STEP_PAYLOAD_SIZE)
constexpr uint32_t MAX_PAYLOAD_SIZE = 20 * 1024;

using MessengerClass = Data_Messenger<CommsClass, Test_Message_Type, Test_Message_Type, MAX_PAYLOAD_SIZE>;

// Looped back: TX (4) wired to RX (5), and the sending FL line (6) wired to the receiving one (7). TX and RX are
// also UART 1's pins, so the same wiring serves the hardware UART links
constexpr CommsClass::UART_Config COMMS_CONFIG = {
	.uartRX = {
		.uartIndex = 1,
		.pin = 5,
		.baudRate = UART_Master::BaudRate::Baud2000000,
	},
	.uartTX = {
		.uartIndex = 1,
		.pin = 4,
		.baudRate = UART_Master::BaudRate::Baud2000000,
	},
	.dmaRXChannel = 0,
	.dmaTXChannel = 1,
	.rx_fl_pin = 7,
	.tx_fl_pin = 6,
};

CommsClass commsUnderTest(COMMS_CONFIG);
MessengerClass messengerUnderTest(commsUnderTest);

//------------------------------------ MESSENGER TEST PAYLOAD --------------------------------------

// The largest payload sent in one step: the 64-byte frame, less the 7-byte header and the 4-byte CRC
constexpr uint32_t MAX_ONE_STEP_PAYLOAD_SIZE = 64 - (7 + 4);

// The most parameters one message can hold (the messenger's MAX_PARAMETER_COUNT)
constexpr uint8_t MAX_PARAMETER_COUNT = 8;

// Global, as the largest payload is far too big for the stack
uint8_t testPayload[MAX_PAYLOAD_SIZE] = {};

// The application's own copy of a received message, sent on from here so the messenger's receive buffer is
// free for the next message straight away
uint8_t relayPayload[MAX_PAYLOAD_SIZE] = {};

/// @brief The value of each payload byte: its index, so a shifted, missing or corrupted byte shows up
/// @param index The byte's position in the payload
/// @return The byte's value
uint8_t patternByte(const uint32_t index)
{
	return static_cast<uint8_t>(index);
}

/// @brief Fill the test payload with the pattern
/// @param size Number of payload bytes, up to MAX_PAYLOAD_SIZE
void fillTestPayload(const uint32_t size)
{
	for (uint32_t index = 0; index < size; ++index)
		testPayload[index] = patternByte(index);
}

/// @brief Check a received payload follows the pattern
/// @param payload The payload's bytes
/// @param size Number of payload bytes
/// @return True if every byte matches the pattern
bool payloadMatchesPattern(const uint8_t *payload, const uint32_t size)
{
	for (uint32_t index = 0; index < size; ++index)
	{
		if (payload[index] != patternByte(index))
			return false;
	}

	return true;
}

//----------------------------------- MESSENGER CALLBACKS ------------------------------------------

/// @brief What the callbacks have seen since the record was last reset
struct Callback_Record final
{
	uint32_t messageCount = 0;
	Test_Message_Type messageType = Test_Message_Type::MIN;
	uint32_t payloadSize = 0;
	bool payloadValid = false; // The payload matched the pattern, or was nullptr when its size was 0
	uint8_t parameterCount = 0;
	bool parametersValid = false; // Matched the parameters sent, or were nullptr when there were none
	uint32_t errorCount = 0;
	MessengerClass::Error lastError = MessengerClass::Error::None;
	uint32_t sentCount = 0;
	Test_Message_Type sentMessageType = Test_Message_Type::MIN;
	uint32_t sentAtMicros = 0;	  // When the sent callback was called, for timing a whole async send
	bool relayCopied = false;	  // A RelayIn payload was copied into relayPayload
	uint32_t relayCopyMicros = 0; // How long the copy took
};

Callback_Record callbackRecord = {};

// The parameters the test under way sent, for the receive callback to check against
const MessengerClass::Parameter *sentParameters = nullptr;
uint8_t sentParameterCount = 0;

/// @brief Check two parameters both hold a T, with the same value
/// @tparam T The value's type: bool, uint8_t, uint16_t, uint32_t or float
/// @param received The parameter received
/// @param sent The parameter sent
/// @return True if both hold a T, with the same value
template <typename T>
bool parametersMatchAs(const MessengerClass::Parameter &received, const MessengerClass::Parameter &sent)
{
	T receivedValue = {};
	T sentValue = {};
	return received.getValue(receivedValue) && sent.getValue(sentValue) && (receivedValue == sentValue);
}

/// @brief Check a received parameter matches the one sent
/// @param received The parameter received
/// @param sent The parameter sent
/// @return True if they hold the same type and value
bool parameterMatches(const MessengerClass::Parameter &received, const MessengerClass::Parameter &sent)
{
	// A parameter's value type isn't public, so each type is tried
	return parametersMatchAs<bool>(received, sent) ||
		   parametersMatchAs<uint8_t>(received, sent) ||
		   parametersMatchAs<uint16_t>(received, sent) ||
		   parametersMatchAs<uint32_t>(received, sent) ||
		   parametersMatchAs<float>(received, sent);
}

/// @brief Check received parameters match the ones the test sent
/// @param parameters The parameters received
/// @param parameterCount Number of parameters received
/// @return True if there are as many as were sent, and each matches
bool parametersMatchSent(const MessengerClass::Parameter *parameters, const uint8_t parameterCount)
{
	if (parameterCount != sentParameterCount)
		return false;

	for (uint8_t index = 0; index < parameterCount; ++index)
	{
		if (!parameterMatches(parameters[index], sentParameters[index]))
			return false;
	}

	return true;
}

/// @brief Record and print each message received
/// @param messageType The message's type
/// @param parameters The parameters, or nullptr when parameterCount is 0
/// @param parameterCount Number of parameters
void handleMessengerReceive(const Test_Message_Type messageType, const MessengerClass::Parameter *parameters,
							const uint8_t parameterCount)
{
	++callbackRecord.messageCount;
	callbackRecord.messageType = messageType;
	callbackRecord.parameterCount = parameterCount;

	if (parameterCount == 0)
		callbackRecord.parametersValid = (parameters == nullptr);
	else
		callbackRecord.parametersValid = parametersMatchSent(parameters, parameterCount);

	logValue("Messenger received. Message type = ", static_cast<uint8_t>(messageType));
	logValue("Messenger received. Parameter count = ", parameterCount);
	logValue("Messenger received. Parameters valid = ", callbackRecord.parametersValid);
}

/// @brief Record and print each payload received
/// @param messageType The message's type
/// @param payload The payload's bytes, or nullptr when payloadSize is 0
/// @param payloadSize Number of payload bytes
void handleMessengerReceivePayload(const Test_Message_Type messageType, const uint8_t *payload,
								   const uint32_t payloadSize)
{
	++callbackRecord.messageCount;
	callbackRecord.messageType = messageType;
	callbackRecord.payloadSize = payloadSize;

	if (payloadSize == 0)
		callbackRecord.payloadValid = (payload == nullptr);
	else
		callbackRecord.payloadValid = payloadMatchesPattern(payload, payloadSize);

	// Only RelayIn is copied, so the relayed message coming back over the loop isn't relayed again
	if (messageType == Test_Message_Type::RelayIn)
	{
		const uint32_t startMicros = micros();
		memcpy(relayPayload, payload, payloadSize);
		callbackRecord.relayCopyMicros = micros() - startMicros;
		callbackRecord.relayCopied = true;
	}

	logValue("Messenger received. Message type = ", static_cast<uint8_t>(messageType));
	logValue("Messenger received. Payload size = ", payloadSize);
	logValue("Messenger received. Payload valid = ", callbackRecord.payloadValid);
}

/// @brief Record and print each error
/// @param error The error
void handleMessengerError(const MessengerClass::Error error)
{
	++callbackRecord.errorCount;
	callbackRecord.lastError = error;

	logValue("Messenger error = ", static_cast<uint8_t>(error));
}

/// @brief Record and print each async message sent
/// @param messageType The message's type
void handleMessengerSent(const Test_Message_Type messageType)
{
	// Taken first, so the rest of the callback isn't included in a measured send
	callbackRecord.sentAtMicros = micros();

	++callbackRecord.sentCount;
	callbackRecord.sentMessageType = messageType;

	logValue("Messenger sent. Message type = ", static_cast<uint8_t>(messageType));
}

//---------------------------------------- TEST HELPERS --------------------------------------------

/// @brief Send a message with a patterned payload, and check it's received exactly once, intact, with no errors
/// @param messenger The messenger, looped back so it receives what it sends
/// @param messageType The message's type
/// @param payloadSize Number of payload bytes, from 1 up to MAX_PAYLOAD_SIZE
/// @return True if the message was sent and received as expected
bool sendAndCheck(MessengerClass &messenger, const Test_Message_Type messageType, const uint32_t payloadSize)
{
	callbackRecord = {};
	fillTestPayload(payloadSize);

	const uint8_t *payload = (payloadSize > 0) ? testPayload : nullptr;

	const bool sent = messenger.sendPayload(messageType, payload, payloadSize);

	// send returns once every byte has been sent, so the message has been received by now
	messenger.process();

	const bool receivedOnce = (callbackRecord.messageCount == 1);
	const bool messageTypeMatches = (callbackRecord.messageType == messageType);
	const bool sizeMatches = (callbackRecord.payloadSize == payloadSize);
	const bool noErrors = (callbackRecord.errorCount == 0);

	return sent && receivedOnce && messageTypeMatches && sizeMatches && callbackRecord.payloadValid && noErrors;
}

/// @brief Whether the async message under test has been both sent and received
/// @return True once the sent and receive callbacks have both been called
bool asyncMessageSentAndReceived()
{
	return (callbackRecord.sentCount > 0) && (callbackRecord.messageCount > 0);
}

/// @brief Call process until the async message under test has been both sent and received, or the time runs out
/// @param messenger The messenger, looped back so it receives what it sends
void processUntilAsyncMessageDone(MessengerClass &messenger)
{
	// A send that never finishes fails its test, rather than hanging the test pack
	constexpr uint32_t TIMEOUT_MILLIS = 1000;

	const uint32_t startMillis = millis();
	while (!asyncMessageSentAndReceived() && ((millis() - startMillis) < TIMEOUT_MILLIS))
		messenger.process();
}

/// @brief Send an async message with a patterned payload, and check it's reported sent and received exactly once,
/// @brief intact, with no errors
/// @param messenger The messenger, looped back so it receives what it sends
/// @param messageType The message's type
/// @param payloadSize Number of payload bytes, from 1 up to MAX_PAYLOAD_SIZE
/// @return True if the message was sent and received as expected
bool sendAsyncAndCheck(MessengerClass &messenger, const Test_Message_Type messageType, const uint32_t payloadSize)
{
	callbackRecord = {};
	fillTestPayload(payloadSize);

	const uint8_t *payload = (payloadSize > 0) ? testPayload : nullptr;

	const bool sendStarted = messenger.sendPayloadAsync(messageType, payload, payloadSize, handleMessengerSent);

	// sendAsync returns once sending has started, so process finishes it
	processUntilAsyncMessageDone(messenger);

	const bool sentOnce = (callbackRecord.sentCount == 1);
	const bool sentMatches = (callbackRecord.sentMessageType == messageType);

	const bool receivedOnce = (callbackRecord.messageCount == 1);
	const bool messageTypeMatches = (callbackRecord.messageType == messageType);
	const bool sizeMatches = (callbackRecord.payloadSize == payloadSize);
	const bool noErrors = (callbackRecord.errorCount == 0);

	return sendStarted && sentOnce && sentMatches &&
		   receivedOnce && messageTypeMatches && sizeMatches && callbackRecord.payloadValid && noErrors;
}

uint32_t testsRun = 0;
uint32_t testsPassed = 0;

/// @brief Run one test and print its result
/// @param name The test's name
/// @param test The test
/// @param messenger The messenger the test uses
void runTest(const char *name, bool (*test)(MessengerClass &), MessengerClass &messenger)
{
	logStr("");
	logStr(name);

	const bool passed = test(messenger);

	++testsRun;
	if (passed)
	{
		++testsPassed;
		logStr("PASS");
	}
	else
		logStr("FAIL");
}

//-------------------------------- TESTS: SETUP AND SEND VALIDATION --------------------------------
// Run before the messenger is set up: each is rejected before any hardware is touched

/// @brief Test 1: setup without a receive callback fails
/// @param messenger The messenger, not yet set up
/// @return True if setup returned false
bool testSetupWithoutReceiveCallback(MessengerClass &messenger)
{
	const bool setUp = messenger.setup(nullptr, handleMessengerReceivePayload, handleMessengerError);
	return !setUp;
}

/// @brief Test 3: sending a payload over the largest size fails
/// @param messenger The messenger
/// @return True if send returned false
bool testSendOverMaxPayload(MessengerClass &messenger)
{
	// The size is rejected before the payload is read, so the buffer only needs to exist
	const bool sent = messenger.sendPayload(Test_Message_Type::SendOverMaxPayload, testPayload, MAX_PAYLOAD_SIZE + 1);
	return !sent;
}

//---------------------------------- TESTS: NORMAL MESSAGES ----------------------------------------

/// @brief Test 4: a message type with no parameters
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if received as expected
bool testMessageTypeOnly(MessengerClass &messenger)
{
	callbackRecord = {};

	const bool sent = messenger.send(Test_Message_Type::MessageTypeOnly);

	// send returns once every byte has been sent, so the message has been received by now
	messenger.process();

	const bool receivedOnce = (callbackRecord.messageCount == 1);
	const bool messageTypeMatches = (callbackRecord.messageType == Test_Message_Type::MessageTypeOnly);
	const bool noParameters = (callbackRecord.parameterCount == 0) && callbackRecord.parametersValid;
	const bool noErrors = (callbackRecord.errorCount == 0);

	return sent && receivedOnce && messageTypeMatches && noParameters && noErrors;
}

/// @brief Test 5: a small one-step payload
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if received as expected
bool testOneStepSmall(MessengerClass &messenger)
{
	return sendAndCheck(messenger, Test_Message_Type::OneStepSmall, 10);
}

/// @brief Test 6: the largest one-step payload
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if received as expected
bool testOneStepLimit(MessengerClass &messenger)
{
	return sendAndCheck(messenger, Test_Message_Type::OneStepLimit, MAX_ONE_STEP_PAYLOAD_SIZE);
}

/// @brief Test 7: the smallest two-step payload
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if received as expected
bool testTwoStepLowerLimit(MessengerClass &messenger)
{
	return sendAndCheck(messenger, Test_Message_Type::TwoStepLowerLimit, MAX_ONE_STEP_PAYLOAD_SIZE + 1);
}

/// @brief Test 8: a large two-step payload
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if received as expected
bool testTwoStepLarge(MessengerClass &messenger)
{
	return sendAndCheck(messenger, Test_Message_Type::TwoStepLarge, 1000);
}

/// @brief Test 9: the largest payload
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if received as expected
bool testTwoStepLimit(MessengerClass &messenger)
{
	return sendAndCheck(messenger, Test_Message_Type::TwoStepLimit, MAX_PAYLOAD_SIZE);
}

/// @brief Test 10: several messages in a row, one-step and two-step mixed
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if every message was received as expected
bool testSeveralInARow(MessengerClass &messenger)
{
	constexpr uint32_t PAYLOAD_SIZES[] = {10, MAX_ONE_STEP_PAYLOAD_SIZE, MAX_ONE_STEP_PAYLOAD_SIZE + 1, 1000, 10};

	bool allPassed = true;
	for (const uint32_t payloadSize : PAYLOAD_SIZES)
	{
		if (!sendAndCheck(messenger, Test_Message_Type::SeveralInARow, payloadSize))
			allPassed = false;
	}

	return allPassed;
}

//---------------------------------- TESTS: DROPS AND RECOVERING SYNC ------------------------------
// Each sends a second message before checkForMessages has handled the first, so the second is dropped

/// @brief Test 14: a one-step message arriving before the last one is handled is dropped, and the first is still
/// @brief delivered
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if the first message was delivered and the drop reported
bool testOneStepDropped(MessengerClass &messenger)
{
	callbackRecord = {};
	fillTestPayload(10);

	const bool firstSent = messenger.sendPayload(Test_Message_Type::OneStepDropped, testPayload, 10);
	const bool secondSent = messenger.sendPayload(Test_Message_Type::OneStepDropped, testPayload, 10);

	messenger.process();

	const bool firstDelivered = (callbackRecord.messageCount == 1) && callbackRecord.payloadValid;
	const bool dropReported = (callbackRecord.errorCount == 1) &&
							  (callbackRecord.lastError == MessengerClass::Error::ReceivedMessageDropped);

	return firstSent && secondSent && firstDelivered && dropReported;
}

/// @brief Test 15: a two-step message arriving before the last one is handled is dropped, the first is still
/// @brief delivered, and the next message is received normally (the messenger stayed in sync)
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if the first message was delivered, an error reported, and the next message received
bool testTwoStepDropped(MessengerClass &messenger)
{
	callbackRecord = {};
	fillTestPayload(1000);

	const bool firstSent = messenger.sendPayload(Test_Message_Type::TwoStepDropped, testPayload, 10);
	const bool secondSent = messenger.sendPayload(Test_Message_Type::TwoStepDropped, testPayload, 1000);

	messenger.process();

	// Note: the first message is still waiting to be handled when both the header frame and the payload frame
	// end, so both are dropped as MessageDropped (the start byte is never checked)
	const bool firstDelivered = (callbackRecord.messageCount == 1) && callbackRecord.payloadValid;
	const bool dropReported = (callbackRecord.errorCount == 1) &&
							  (callbackRecord.lastError == MessengerClass::Error::ReceivedMessageDropped);

	const bool stillInSync = sendAndCheck(messenger, Test_Message_Type::TwoStepDropped, 10);

	return firstSent && secondSent && firstDelivered && dropReported && stillInSync;
}

//---------------------------------------- TESTS: UNKNOWN MESSAGE TYPES ----------------------------

/// @brief Test 16: a message whose type is outside the receive message types' MIN to MAX is rejected, as if the
/// @brief sender had been built with message types this end doesn't know
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if the message wasn't delivered and UnknownMessageType was reported
bool testUnknownMessageType(MessengerClass &messenger)
{
	callbackRecord = {};
	fillTestPayload(10);

	// One past the last known type. Any uint8_t converts safely to a uint8_t enum, so this is well defined
	const uint8_t unknownValue = static_cast<uint8_t>(Test_Message_Type::MAX) + 1;
	const Test_Message_Type unknownType = static_cast<Test_Message_Type>(unknownValue);

	const bool sent = messenger.sendPayload(unknownType, testPayload, 10);
	messenger.process();

	const bool notDelivered = (callbackRecord.messageCount == 0);
	const bool unknownReported = (callbackRecord.errorCount == 1) &&
								 (callbackRecord.lastError == MessengerClass::Error::UnknownMessageType);

	return sent && notDelivered && unknownReported;
}

//---------------------------------------- TESTS: ASYNC SENDS --------------------------------------

/// @brief Test 17: an async one-step message
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if sent and received as expected
bool testAsyncOneStep(MessengerClass &messenger)
{
	return sendAsyncAndCheck(messenger, Test_Message_Type::AsyncOneStep, 10);
}

/// @brief Test 18: an async two-step message, at the largest payload
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if sent and received as expected
bool testAsyncTwoStep(MessengerClass &messenger)
{
	return sendAsyncAndCheck(messenger, Test_Message_Type::AsyncTwoStep, MAX_PAYLOAD_SIZE);
}

/// @brief Test 19: sends tried while an async message is still sending are rejected, and the async message is
/// @brief still sent and received
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if both sends were rejected, and the async message was sent and received as expected
bool testAsyncInProgress(MessengerClass &messenger)
{
	callbackRecord = {};
	fillTestPayload(MAX_PAYLOAD_SIZE);

	const bool firstStarted = messenger.sendPayloadAsync(Test_Message_Type::AsyncInProgress, testPayload, MAX_PAYLOAD_SIZE,
												  handleMessengerSent);

	// Checked straight away, so the sends below are known to be tried while the first is really still sending
	const bool stillSending = commsUnderTest.sendingBytes();

	const bool sendRejected = !messenger.sendPayload(Test_Message_Type::AsyncInProgress, testPayload, 10);
	const bool sendAsyncRejected = !messenger.sendPayloadAsync(Test_Message_Type::AsyncInProgress, testPayload, 10,
														handleMessengerSent);

	processUntilAsyncMessageDone(messenger);

	const bool firstSentOnce = (callbackRecord.sentCount == 1);
	const bool firstReceivedOnce = (callbackRecord.messageCount == 1) &&
								   (callbackRecord.payloadSize == MAX_PAYLOAD_SIZE) && callbackRecord.payloadValid;
	const bool noErrors = (callbackRecord.errorCount == 0);

	return firstStarted && stillSending && sendRejected && sendAsyncRejected &&
		   firstSentOnce && firstReceivedOnce && noErrors;
}

//---------------------------------------- TESTS: RELAY --------------------------------------------

/// @brief Test 20: a large message is received, copied in the receive callback, and the copy sent on async, as
/// @brief the RP does with an image from the vision MCU to the Wi-Fi unit. Prints how long core 0 is blocked
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if the message was received and copied, and the copy was sent async and received intact
bool testRelay(MessengerClass &messenger)
{
	callbackRecord = {};
	fillTestPayload(MAX_PAYLOAD_SIZE);

	// Plays the vision MCU's message arriving
	const bool inSent = messenger.sendPayload(Test_Message_Type::RelayIn, testPayload, MAX_PAYLOAD_SIZE);
	messenger.process();

	const bool inReceived = (callbackRecord.messageCount == 1) && callbackRecord.payloadValid;
	const bool inCopied = callbackRecord.relayCopied;
	const bool inNoErrors = (callbackRecord.errorCount == 0);
	const uint32_t copyMicros = callbackRecord.relayCopyMicros;

	// Cleared, so only the relayed message is counted from here
	callbackRecord = {};

	const uint32_t startMicros = micros();
	const bool outStarted = messenger.sendPayloadAsync(Test_Message_Type::RelayOut, relayPayload, MAX_PAYLOAD_SIZE,
												handleMessengerSent);
	const uint32_t sendAsyncMicros = micros() - startMicros;

	// The proof it's async: sendAsync has returned while the payload is still being sent
	const bool stillSending = commsUnderTest.sendingBytes();

	processUntilAsyncMessageDone(messenger);

	const bool outSentOnce = (callbackRecord.sentCount == 1);
	const bool outReceivedOnce = (callbackRecord.messageCount == 1) &&
								 (callbackRecord.messageType == Test_Message_Type::RelayOut) &&
								 (callbackRecord.payloadSize == MAX_PAYLOAD_SIZE) && callbackRecord.payloadValid;
	const bool outNoErrors = (callbackRecord.errorCount == 0);

	// Printed with Serial directly, so they show with APP_DEBUG at 0
	Serial.print("Relay copy time: ");
	Serial.print(copyMicros);
	Serial.println(" us");
	Serial.print("Relay sendAsync time: ");
	Serial.print(sendAsyncMicros);
	Serial.println(" us");
	Serial.print("Relay still sending when sendAsync returned: ");
	Serial.println(stillSending);

	return inSent && inReceived && inCopied && inNoErrors &&
		   outStarted && stillSending && outSentOnce && outReceivedOnce && outNoErrors;
}

//---------------------------------------- TESTS: PARAMETERS ---------------------------------------

/// @brief Send a message with parameters, and check it's received exactly once, intact, with no errors
/// @tparam PARAMETER_COUNT Number of parameters. Inferred from the array
/// @param messenger The messenger, looped back so it receives what it sends
/// @param messageType The message's type
/// @param parameters The parameters
/// @return True if the message was sent and received as expected
template <uint8_t PARAMETER_COUNT>
bool sendParametersAndCheck(MessengerClass &messenger, const Test_Message_Type messageType,
							const MessengerClass::Parameter (&parameters)[PARAMETER_COUNT])
{
	callbackRecord = {};
	sentParameters = parameters;
	sentParameterCount = PARAMETER_COUNT;

	const bool sent = messenger.send(messageType, parameters);

	// send returns once every byte has been sent, so the message has been received by now
	messenger.process();

	const bool receivedOnce = (callbackRecord.messageCount == 1);
	const bool messageTypeMatches = (callbackRecord.messageType == messageType);
	const bool parametersMatch = (callbackRecord.parameterCount == PARAMETER_COUNT) && callbackRecord.parametersValid;
	const bool noErrors = (callbackRecord.errorCount == 0);

	return sent && receivedOnce && messageTypeMatches && parametersMatch && noErrors;
}

/// @brief Test 21: one parameter of every value type, each at a value that uses its whole size, and a negative
/// @brief float so its sign bit is sent
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if received as expected
bool testParametersEveryType(MessengerClass &messenger)
{
	const MessengerClass::Parameter parameters[] = {true, uint8_t(200), uint16_t(60000), uint32_t(4000000000),
													-3.25f};

	return sendParametersAndCheck(messenger, Test_Message_Type::ParametersEveryType, parameters);
}

/// @brief Test 22: the most parameters a message can hold, each of the largest value type
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if received as expected
bool testParametersMost(MessengerClass &messenger)
{
	// Sized by MAX_PARAMETER_COUNT, so a different count fails to compile
	const MessengerClass::Parameter parameters[MAX_PARAMETER_COUNT] = {
		uint32_t(0x01020304), uint32_t(0x05060708), uint32_t(0x090A0B0C), uint32_t(0x0D0E0F10),
		uint32_t(0x11121314), uint32_t(0x15161718), uint32_t(0x191A1B1C), uint32_t(0x1D1E1F20)};

	return sendParametersAndCheck(messenger, Test_Message_Type::ParametersMost, parameters);
}

/// @brief Test 23: an async message with parameters
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if sent and received as expected
bool testParametersAsync(MessengerClass &messenger)
{
	// The array is sized by it, so a different count fails to compile
	constexpr uint8_t PARAMETER_COUNT = 3;
	const MessengerClass::Parameter parameters[PARAMETER_COUNT] = {uint8_t(5), true, 0.22f};

	callbackRecord = {};
	sentParameters = parameters;
	sentParameterCount = PARAMETER_COUNT;

	const bool sendStarted = messenger.sendAsync(Test_Message_Type::ParametersAsync, parameters,
												 handleMessengerSent);

	// sendAsync returns once sending has started, so process finishes it
	processUntilAsyncMessageDone(messenger);

	const bool sentOnce = (callbackRecord.sentCount == 1);
	const bool sentMatches = (callbackRecord.sentMessageType == Test_Message_Type::ParametersAsync);

	const bool receivedOnce = (callbackRecord.messageCount == 1);
	const bool messageTypeMatches = (callbackRecord.messageType == Test_Message_Type::ParametersAsync);
	const bool parametersMatch = (callbackRecord.parameterCount == PARAMETER_COUNT) && callbackRecord.parametersValid;
	const bool noErrors = (callbackRecord.errorCount == 0);

	return sendStarted && sentOnce && sentMatches &&
		   receivedOnce && messageTypeMatches && parametersMatch && noErrors;
}

//-------------------------------------------- PERFORMANCE -----------------------------------------

/// @brief How long sending one parameterised message took
struct Parameter_Send_Times final
{
	uint32_t sendMicros = 0;			// How long send took to return
	uint32_t sendAsyncMicros = 0;		// How long sendAsync took to return
	uint32_t sendAsyncToSentMicros = 0; // From calling sendAsync to its sent callback
};

/// @brief Measure how long send and sendAsync take for a message with parameters
/// @tparam PARAMETER_COUNT Number of parameters. Inferred from the array
/// @param messenger The messenger, set up and looped back
/// @param parameters The parameters
/// @return The times
template <uint8_t PARAMETER_COUNT>
Parameter_Send_Times measureParameterSendTimes(MessengerClass &messenger,
											   const MessengerClass::Parameter (&parameters)[PARAMETER_COUNT])
{
	Parameter_Send_Times times;

	// Kept current, so the receive callback checks against these parameters
	sentParameters = parameters;
	sentParameterCount = PARAMETER_COUNT;

	const uint32_t sendStartMicros = micros();
	messenger.send(Test_Message_Type::SendTimeMeasurement, parameters);
	times.sendMicros = micros() - sendStartMicros;

	// Handled before the next send, so the next message isn't dropped
	messenger.process();

	callbackRecord = {};

	const uint32_t sendAsyncStartMicros = micros();
	messenger.sendAsync(Test_Message_Type::SendTimeMeasurement, parameters, handleMessengerSent);
	times.sendAsyncMicros = micros() - sendAsyncStartMicros;

	// Finished before the next send, as a send is rejected while an async one is in progress
	processUntilAsyncMessageDone(messenger);
	times.sendAsyncToSentMicros = callbackRecord.sentAtMicros - sendAsyncStartMicros;

	return times;
}

/// @brief Measure how long send and sendAsync take for a message with no parameters
/// @param messenger The messenger, set up and looped back
/// @return The times
Parameter_Send_Times measureNoParameterSendTimes(MessengerClass &messenger)
{
	Parameter_Send_Times times;

	// Kept current, so the receive callback checks against no parameters
	sentParameters = nullptr;
	sentParameterCount = 0;

	const uint32_t sendStartMicros = micros();
	messenger.send(Test_Message_Type::SendTimeMeasurement);
	times.sendMicros = micros() - sendStartMicros;

	// Handled before the next send, so the next message isn't dropped
	messenger.process();

	callbackRecord = {};

	const uint32_t sendAsyncStartMicros = micros();
	messenger.sendAsync(Test_Message_Type::SendTimeMeasurement, handleMessengerSent);
	times.sendAsyncMicros = micros() - sendAsyncStartMicros;

	// Finished before the next send, as a send is rejected while an async one is in progress
	processUntilAsyncMessageDone(messenger);
	times.sendAsyncToSentMicros = callbackRecord.sentAtMicros - sendAsyncStartMicros;

	return times;
}

/// @brief Print the times for one parameterised message
/// @param parameterCount Number of parameters the message had
/// @param times The times
void printParameterSendTimes(const uint8_t parameterCount, const Parameter_Send_Times &times)
{
	Serial.print(parameterCount);
	Serial.print(" parameters: send ");
	Serial.print(times.sendMicros);
	Serial.print(" us, sendAsync ");
	Serial.print(times.sendAsyncMicros);
	Serial.print(" us, sendAsync to sent ");
	Serial.print(times.sendAsyncToSentMicros);
	Serial.println(" us");
}

/// @brief Measure how long send and sendAsync take to return for one-step and two-step messages, and print the
/// @brief results. Printed with Serial directly, so it works with APP_DEBUG at 0, which keeps the messenger's
/// @brief logging out of the times
/// @param messenger The messenger, set up and looped back
void measureSendTimes(MessengerClass &messenger)
{
	constexpr uint32_t PAYLOAD_SIZES[] = {10, MAX_ONE_STEP_PAYLOAD_SIZE, MAX_ONE_STEP_PAYLOAD_SIZE + 1, 1000,
										  MAX_PAYLOAD_SIZE};
	constexpr uint8_t MEASUREMENT_COUNT = sizeof(PAYLOAD_SIZES) / sizeof(PAYLOAD_SIZES[0]);

	uint32_t sendMicros[MEASUREMENT_COUNT] = {};
	uint32_t sendAsyncMicros[MEASUREMENT_COUNT] = {};
	uint32_t sendAsyncToSentMicros[MEASUREMENT_COUNT] = {};

	for (uint8_t index = 0; index < MEASUREMENT_COUNT; ++index)
	{
		fillTestPayload(PAYLOAD_SIZES[index]);

		const uint32_t sendStartMicros = micros();
		messenger.sendPayload(Test_Message_Type::SendTimeMeasurement, testPayload, PAYLOAD_SIZES[index]);
		sendMicros[index] = micros() - sendStartMicros;

		// Handled before the next send, so the next message isn't dropped
		messenger.process();

		callbackRecord = {};

		const uint32_t sendAsyncStartMicros = micros();
		messenger.sendPayloadAsync(Test_Message_Type::SendTimeMeasurement, testPayload, PAYLOAD_SIZES[index],
							handleMessengerSent);
		sendAsyncMicros[index] = micros() - sendAsyncStartMicros;

		// Finished before the next send, as a send is rejected while an async one is in progress
		processUntilAsyncMessageDone(messenger);
		sendAsyncToSentMicros[index] = callbackRecord.sentAtMicros - sendAsyncStartMicros;
	}

	// The smallest parameterised message, one parameter, and the largest
	const MessengerClass::Parameter oneParameter[] = {uint8_t(5)};
	const MessengerClass::Parameter mostParameters[MAX_PARAMETER_COUNT] = {
		uint32_t(0x01020304), uint32_t(0x05060708), uint32_t(0x090A0B0C), uint32_t(0x0D0E0F10),
		uint32_t(0x11121314), uint32_t(0x15161718), uint32_t(0x191A1B1C), uint32_t(0x1D1E1F20)};

	const Parameter_Send_Times noParameterTimes = measureNoParameterSendTimes(messenger);
	const Parameter_Send_Times oneParameterTimes = measureParameterSendTimes(messenger, oneParameter);
	const Parameter_Send_Times mostParameterTimes = measureParameterSendTimes(messenger, mostParameters);

	// Printed after all the sends, so printing can't affect any of the times
	Serial.println();
	Serial.println("---- Send times ----");
	for (uint8_t index = 0; index < MEASUREMENT_COUNT; ++index)
	{
		Serial.print(PAYLOAD_SIZES[index]);
		Serial.print(" byte payload: send ");
		Serial.print(sendMicros[index]);
		Serial.print(" us, sendAsync ");
		Serial.print(sendAsyncMicros[index]);
		Serial.print(" us, sendAsync to sent ");
		Serial.print(sendAsyncToSentMicros[index]);
		Serial.println(" us");
	}

	printParameterSendTimes(0, noParameterTimes);
	printParameterSendTimes(1, oneParameterTimes);
	printParameterSendTimes(MAX_PARAMETER_COUNT, mostParameterTimes);
}

void testMessenger()
{
	logStr("");
	logStr("---- Setup and send validation ----");
	runTest("Test 1: setup without a receive callback", testSetupWithoutReceiveCallback, messengerUnderTest);
	runTest("Test 3: send over the largest payload", testSendOverMaxPayload, messengerUnderTest);

	const uint32_t setupStartMicros = micros();
	const bool messengerSetup = messengerUnderTest.setup(handleMessengerReceive, handleMessengerReceivePayload,
														 handleMessengerError);
	const uint32_t setupMicros = micros() - setupStartMicros;
	// Printed with Serial directly, so they show with APP_DEBUG at 0
	if (messengerSetup)
		Serial.println("Messenger setup succeeded");
	else
		Serial.println("Messenger setup FAILED: no tests run");

	Serial.print("Setup time: ");
	Serial.print(setupMicros);
	Serial.println(" us");

	if (messengerSetup)
	{
		logStr("");
		logStr("---- Normal messages ----");
		runTest("Test 4: message type only", testMessageTypeOnly, messengerUnderTest);
		runTest("Test 5: one-step, small", testOneStepSmall, messengerUnderTest);
		runTest("Test 6: one-step, largest", testOneStepLimit, messengerUnderTest);
		runTest("Test 7: two-step, smallest", testTwoStepLowerLimit, messengerUnderTest);
		runTest("Test 8: two-step, large", testTwoStepLarge, messengerUnderTest);
		runTest("Test 9: two-step, largest", testTwoStepLimit, messengerUnderTest);
		runTest("Test 10: several in a row", testSeveralInARow, messengerUnderTest);

		logStr("");
		logStr("---- Drops and recovering sync ----");
		runTest("Test 14: one-step dropped", testOneStepDropped, messengerUnderTest);
		runTest("Test 15: two-step dropped", testTwoStepDropped, messengerUnderTest);

		logStr("");
		logStr("---- Unknown message types ----");
		runTest("Test 16: unknown message type", testUnknownMessageType, messengerUnderTest);

		logStr("");
		logStr("---- Async sends ----");
		runTest("Test 17: async one-step", testAsyncOneStep, messengerUnderTest);
		runTest("Test 18: async two-step, largest", testAsyncTwoStep, messengerUnderTest);
		runTest("Test 19: sends rejected while async in progress", testAsyncInProgress, messengerUnderTest);

		logStr("");
		logStr("---- Relay ----");
		runTest("Test 20: relay a large message", testRelay, messengerUnderTest);

		logStr("");
		logStr("---- Parameters ----");
		runTest("Test 21: every value type", testParametersEveryType, messengerUnderTest);
		runTest("Test 22: the most parameters", testParametersMost, messengerUnderTest);
		runTest("Test 23: async with parameters", testParametersAsync, messengerUnderTest);

		measureSendTimes(messengerUnderTest);
	}

	logStr("");
	logValue("Tests run = ", testsRun);
	logValue("Tests passed = ", testsPassed);
}
