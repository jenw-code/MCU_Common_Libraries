#pragma once

#include <Arduino.h>

inline constexpr uint32_t INT_POW(uint16_t num, uint8_t pow)
{
	return (pow == 0) ? 1 : num * INT_POW(num, pow - 1);
}

inline constexpr unsigned long SECONDS_TO_MILLIS(const uint16_t seconds)
{
	return static_cast<uint64_t>(seconds) * 1000;
}

inline constexpr unsigned long MINUTES_TO_MILLIS(const uint8_t minutes)
{
	return SECONDS_TO_MILLIS(static_cast<uint16_t>(minutes) * 60);
}

