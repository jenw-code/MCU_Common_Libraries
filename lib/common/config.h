#pragma once

#include "pins/pins_v1.h"

namespace CONFIG
{
	namespace BATTERY_LEVEL_LED
	{
		inline constexpr uint16_t ON_MILLIS = 1000;
		inline constexpr uint16_t OFF_MILLIS = 500;
	}

	namespace CRITICAL_BATTERY_LEVEL_LED
	{
		inline constexpr uint16_t ON_MILLIS = 100;
		inline constexpr uint16_t OFF_MILLIS = 100;
		inline constexpr uint8_t SEQUENCE_COUNT = 2;
		inline constexpr uint16_t SEQUENCE_OFF_MILLIS = 3000;
	}
};
