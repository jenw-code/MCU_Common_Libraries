/*

Functionality
-------------

*/
#include <Arduino.h>
#include <pins_arduino.h>
#include "common.debug.h"
#include "common.battery.h"
#include "common.battery.info.h"
#include "common.errors.h"
#include "common.versioning.h"
#include "config.h" // Includes pin mapping

#include "bmm150_i2c.h"

BMM150_I2C bmm;

void setup()
{
    Wire.setSCL(D4);
    Wire.setSDA(D5);

    bmm.begin();
}

//---------------------------------------- LOOP ----------------------------------------------------

void loop()
{

}