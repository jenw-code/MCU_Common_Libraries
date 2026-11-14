#include <Arduino.h>
#include "common.debug.h"

#include "tests/rp235x_lidar_tests.h"
#include "tests/rp235x_messenger_tests.h"
#include "tests/rp235x_uart_tests.h"

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

	// testUART();
}

//---------------------------------------- LOOP ----------------------------------------------------

void loop()
{
	checkLidar();
}
