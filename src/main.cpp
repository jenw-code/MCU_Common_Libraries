#include <Arduino.h>
#include "common.debug.h"

#include "data_messenger.h"
#include "rp235x_data_messenger_comms.h"
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

//---------------------------------------- MESSENGER UNDER TEST ------------------------------------
// The only MCU-specific part. To test another MCU's comms, replace this section

// TEST - temporary: the Pico 2 has no PSRAM, so setup should fail
using CommsClass = RP235x_Data_Messenger_Comms<RP235x_PIO_UART_Config, RP235x_UART_Config, false>;

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
	SendTimeMeasurement,
	MIN = SendOverMaxPayload,
	MAX = SendTimeMeasurement,
};

using MessengerClass = Data_Messenger<CommsClass, Test_Message_Type, Test_Message_Type>;

// Looped back: TX (4) wired to RX (5), and the sending FL line (6) wired to the receiving one (7). TX and RX are
// also UART 1's pins, so the same wiring serves the hardware UART links
constexpr CommsClass::UART_Config COMMS_CONFIG = {
	.uartRX = {
		.pioIndex = 0,
		.stateMachine = 0,
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

// The largest payload Data_Messenger accepts (its MAX_TWO_STEP_PAYLOAD_SIZE)
constexpr uint32_t MAX_PAYLOAD_SIZE = 20 * 1024;

// The largest payload sent in one step: the 64-byte frame, less the 6-byte header and the 4-byte CRC
constexpr uint32_t MAX_ONE_STEP_PAYLOAD_SIZE = 64 - (6 + 4);

// Global, as the largest payload is far too big for the stack
uint8_t testPayload[MAX_PAYLOAD_SIZE] = {};

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
	uint32_t errorCount = 0;
	MessengerClass::Error lastError = MessengerClass::Error::None;
};

Callback_Record callbackRecord = {};

/// @brief Record and print each message received
/// @param messageType The message's type
/// @param payload The payload's bytes, or nullptr when payloadSize is 0
/// @param payloadSize Number of payload bytes
void handleMessengerReceive(const Test_Message_Type messageType, const uint8_t *payload, const uint32_t payloadSize)
{
	++callbackRecord.messageCount;
	callbackRecord.messageType = messageType;
	callbackRecord.payloadSize = payloadSize;

	if (payloadSize == 0)
		callbackRecord.payloadValid = (payload == nullptr);
	else
		callbackRecord.payloadValid = payloadMatchesPattern(payload, payloadSize);

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

//---------------------------------------- TEST HELPERS --------------------------------------------

/// @brief Send a message with a patterned payload, and check it's received exactly once, intact, with no errors
/// @param messenger The messenger, looped back so it receives what it sends
/// @param messageType The message's type
/// @param payloadSize Number of payload bytes, up to MAX_PAYLOAD_SIZE. 0 sends a message type only
/// @return True if the message was sent and received as expected
bool sendAndCheck(MessengerClass &messenger, const Test_Message_Type messageType, const uint32_t payloadSize)
{
	callbackRecord = {};
	fillTestPayload(payloadSize);

	const uint8_t *payload = nullptr;
	if (payloadSize > 0)
		payload = testPayload;

	const bool sent = messenger.send(messageType, payload, payloadSize);

	// send returns once every byte has been sent, so the message has been received by now
	messenger.checkForMessages();

	const bool receivedOnce = (callbackRecord.messageCount == 1);
	const bool messageTypeMatches = (callbackRecord.messageType == messageType);
	const bool sizeMatches = (callbackRecord.payloadSize == payloadSize);
	const bool noErrors = (callbackRecord.errorCount == 0);

	return sent && receivedOnce && messageTypeMatches && sizeMatches && callbackRecord.payloadValid && noErrors;
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
	const bool setUp = messenger.setup(nullptr, handleMessengerError);
	return !setUp;
}

/// @brief Test 3: sending a payload over the largest size fails
/// @param messenger The messenger
/// @return True if send returned false
bool testSendOverMaxPayload(MessengerClass &messenger)
{
	// The size is rejected before the payload is read, so the buffer only needs to exist
	const bool sent = messenger.send(Test_Message_Type::SendOverMaxPayload, testPayload, MAX_PAYLOAD_SIZE + 1);
	return !sent;
}

//---------------------------------- TESTS: NORMAL MESSAGES ----------------------------------------

/// @brief Test 4: a message type with no payload
/// @param messenger The messenger, looped back so it receives what it sends
/// @return True if received as expected
bool testMessageTypeOnly(MessengerClass &messenger)
{
	return sendAndCheck(messenger, Test_Message_Type::MessageTypeOnly, 0);
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
	constexpr uint32_t PAYLOAD_SIZES[] = {0, 10, MAX_ONE_STEP_PAYLOAD_SIZE, MAX_ONE_STEP_PAYLOAD_SIZE + 1, 1000, 10};

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

	const bool firstSent = messenger.send(Test_Message_Type::OneStepDropped, testPayload, 10);
	const bool secondSent = messenger.send(Test_Message_Type::OneStepDropped, testPayload, 10);

	messenger.checkForMessages();

	const bool firstDelivered = (callbackRecord.messageCount == 1) && callbackRecord.payloadValid;
	const bool dropReported = (callbackRecord.errorCount == 1) &&
							  (callbackRecord.lastError == MessengerClass::Error::MessageDropped);

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

	const bool firstSent = messenger.send(Test_Message_Type::TwoStepDropped, testPayload, 10);
	const bool secondSent = messenger.send(Test_Message_Type::TwoStepDropped, testPayload, 1000);

	messenger.checkForMessages();

	// Note: the first message is still waiting to be handled when both the header frame and the payload frame
	// end, so both are dropped as MessageDropped (the start byte is never checked)
	const bool firstDelivered = (callbackRecord.messageCount == 1) && callbackRecord.payloadValid;
	const bool dropReported = (callbackRecord.errorCount == 1) &&
							  (callbackRecord.lastError == MessengerClass::Error::MessageDropped);

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

	const bool sent = messenger.send(unknownType, testPayload, 10);
	messenger.checkForMessages();

	const bool notDelivered = (callbackRecord.messageCount == 0);
	const bool unknownReported = (callbackRecord.errorCount == 1) &&
								 (callbackRecord.lastError == MessengerClass::Error::UnknownMessageType);

	return sent && notDelivered && unknownReported;
}

//-------------------------------------------- PERFORMANCE -----------------------------------------

/// @brief Measure how long send takes for one-step and two-step messages, and print the results. Printed with
/// @brief Serial directly, so it works with APP_DEBUG at 0, which keeps the messenger's logging out of the times
/// @param messenger The messenger, set up and looped back
void measureSendTimes(MessengerClass &messenger)
{
	constexpr uint32_t PAYLOAD_SIZES[] = {0, 10, MAX_ONE_STEP_PAYLOAD_SIZE, MAX_ONE_STEP_PAYLOAD_SIZE + 1, 1000,
										  MAX_PAYLOAD_SIZE};
	constexpr uint8_t MEASUREMENT_COUNT = sizeof(PAYLOAD_SIZES) / sizeof(PAYLOAD_SIZES[0]);

	uint32_t sendMicros[MEASUREMENT_COUNT] = {};

	for (uint8_t index = 0; index < MEASUREMENT_COUNT; ++index)
	{
		fillTestPayload(PAYLOAD_SIZES[index]);

		const uint32_t startMicros = micros();
		messenger.send(Test_Message_Type::SendTimeMeasurement, testPayload, PAYLOAD_SIZES[index]);
		sendMicros[index] = micros() - startMicros;

		// Handled before the next send, so the next message isn't dropped
		messenger.checkForMessages();
	}

	// Printed after all the sends, so printing can't affect any of the times
	Serial.println();
	Serial.println("---- Send times ----");
	for (uint8_t index = 0; index < MEASUREMENT_COUNT; ++index)
	{
		Serial.print(PAYLOAD_SIZES[index]);
		Serial.print(" byte payload: ");
		Serial.print(sendMicros[index]);
		Serial.println(" us");
	}
}

void testMessenger()
{
	logStr("");
	logStr("---- Setup and send validation ----");
	runTest("Test 1: setup without a receive callback", testSetupWithoutReceiveCallback, messengerUnderTest);
	runTest("Test 3: send over the largest payload", testSendOverMaxPayload, messengerUnderTest);

	const uint32_t setupStartMicros = micros();
	const bool messengerSetup = messengerUnderTest.setup(handleMessengerReceive, handleMessengerError);
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

		measureSendTimes(messengerUnderTest);
	}

	logStr("");
	logValue("Tests run = ", testsRun);
	logValue("Tests passed = ", testsPassed);
}

//----------------------------------------------- SETUP --------------------------------------------

void setup()
{
	// Not dependent on APP_DEBUG, as measureSendTimes prints its results with Serial directly
	Serial.begin(115200);
	while (!Serial)
		delay(10);

	logStr("Starting..");

	// testMessenger();

	setupLidar();
}

//---------------------------------------- LOOP ----------------------------------------------------

void loop()
{
	if (lidar1 != nullptr)
	{
		lidar1->checkDistance();
	}

	// Well under the auto-checker's 200 ms interval, so checkDistance runs each time its timer asks
	delay(20);
}
