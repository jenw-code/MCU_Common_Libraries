#pragma once

#include <Arduino.h>

/// @brief Parse a byte as an enum value, checking it's one of the enum's values
/// @tparam Enum A uint8_t enum class with MIN and MAX values aliasing its first and last values
/// @param value The byte
/// @param result Receives the enum value. Only changed if the byte maps to the enum
/// @return True if the byte is within the enum's MIN to MAX
template <typename Enum>
bool parseEnum(const uint8_t value, Enum &result)
{
	static_assert(sizeof(Enum) == sizeof(uint8_t), "The enum must be a uint8_t enum");

	// Any uint8_t converts safely to a uint8_t enum, so the conversion is made first and the range checked after
	const Enum candidate = static_cast<Enum>(value);
	if ((candidate >= Enum::MIN) && (candidate <= Enum::MAX))
	{
		result = candidate;
		return true;
	}

	return false;
}
