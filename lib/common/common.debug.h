#pragma once

#include <Arduino.h>

#define APP_DEBUG 0 // 0 = No debugging, 1 = Debugging

#if APP_DEBUG
#define logStr(s) Serial.println(s)
#define logValue(s, v)     \
	{                      \
		Serial.print(s);   \
		Serial.println(v); \
	}
#define logValueBIN(s, v)     \
	{                         \
		Serial.print(s);      \
		Serial.print(v, BIN); \
		Serial.println();     \
	}
#define logError(category, code)                      \
	{                                                 \
		Serial.print(F("Error: Category = "));        \
		Serial.print(static_cast<uint8_t>(category)); \
		Serial.print(F(", Code = "));                 \
		Serial.println(static_cast<uint8_t>(code));   \
	}

#else
#define logStr(s)
#define logValue(s, v)
#define logValueBIN(s, v)
#define logError(category, code)
#endif