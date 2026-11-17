#pragma once

#include <Arduino.h>

#define APP_DEBUG 0 // 0 = No debugging, 1 = Debugging

/// @brief Print a line, when debugging
/// @tparam Text The line's type (e.g. a string, or F() for a string held in flash)
/// @param text The line
template <typename Text>
void logStr(const Text &text)
{
	if constexpr (APP_DEBUG)
		Serial.println(text);
}

/// @brief Print a label followed by a value, when debugging
/// @tparam Label The label's type (e.g. a string, or F() for a string held in flash)
/// @tparam Value The value's type: anything Serial can print
/// @param label The label
/// @param value The value
template <typename Label, typename Value>
void logValue(const Label &label, const Value &value)
{
	if constexpr (APP_DEBUG)
	{
		Serial.print(label);
		Serial.println(value);
	}
}

/// @brief Print a label followed by a value in binary, when debugging
/// @tparam Label The label's type (e.g. a string, or F() for a string held in flash)
/// @tparam Value The value's type: an integer
/// @param label The label
/// @param value The value
template <typename Label, typename Value>
void logValueBIN(const Label &label, const Value &value)
{
	if constexpr (APP_DEBUG)
	{
		Serial.print(label);
		Serial.print(value, BIN);
		Serial.println();
	}
}

/// @brief Print an error's category and code, when debugging
/// @tparam Category The category's type: a uint8_t enum
/// @tparam Code The code's type: a uint8_t enum
/// @param category The error's category
/// @param code The error's code
template <typename Category, typename Code>
void logError(const Category category, const Code code)
{
	if constexpr (APP_DEBUG)
	{
		Serial.print(F("Error: Category = "));
		Serial.print(static_cast<uint8_t>(category));
		Serial.print(F(", Code = "));
		Serial.println(static_cast<uint8_t>(code));
	}
}