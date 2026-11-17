#pragma once

#include <Arduino.h>
#include <type_traits>

// A float is packed and unpacked as a uint32_t's bit pattern
static_assert(sizeof(float) == sizeof(uint32_t), "A float must be the same size as a uint32_t");

/// @brief Pack a value into bytes, little-endian. A float is packed as its bit pattern
/// @brief e.g. the uint16_t 0x1234 gives 0x34, 0x12
/// @tparam T The value's type: bool, uint8_t, uint16_t, uint32_t or float
/// @param bytes Where the value's bytes go, least significant first. There must be room for sizeof(T) bytes
/// @param value The value
template <typename T>
void packValue(uint8_t *const bytes, const T value)
{
	if constexpr ((std::is_same_v<T, bool>) ||
				  (std::is_same_v<T, uint8_t>))
		bytes[0] = static_cast<uint8_t>(value);

	else if constexpr (std::is_same_v<T, uint16_t>)
	{
		bytes[0] = static_cast<uint8_t>(value);
		bytes[1] = static_cast<uint8_t>(value >> 8);
	}
	else if constexpr (std::is_same_v<T, uint32_t>)
	{
		bytes[0] = static_cast<uint8_t>(value);
		bytes[1] = static_cast<uint8_t>(value >> 8);
		bytes[2] = static_cast<uint8_t>(value >> 16);
		bytes[3] = static_cast<uint8_t>(value >> 24);
	}
	else if constexpr (std::is_same_v<T, float>)
	{
		uint32_t uint32Value;
		memcpy(&uint32Value, &value, sizeof(float));
		packValue(bytes, uint32Value);
	}
	else
		static_assert(sizeof(T) == 0, "Unsupported type for packValue");
}

/// @brief Unpack a little-endian value from bytes. A float is unpacked from its bit pattern
/// @brief e.g. 0x34, 0x12 gives the uint16_t 0x1234
/// @tparam T The value's type: bool, uint8_t, uint16_t, uint32_t or float
/// @param bytes The value's bytes, least significant first
/// @return The value
template <typename T>
T unpackValue(const uint8_t *const bytes)
{
	if constexpr ((std::is_same_v<T, bool>) ||
				  (std::is_same_v<T, uint8_t>))
		return static_cast<T>(bytes[0]);

	else if constexpr (std::is_same_v<T, uint16_t>)
	{
		return static_cast<uint16_t>(bytes[0]) |
			   (static_cast<uint16_t>(bytes[1]) << 8);
	}
	else if constexpr (std::is_same_v<T, uint32_t>)
	{
		return static_cast<uint32_t>(bytes[0]) |
			   (static_cast<uint32_t>(bytes[1]) << 8) |
			   (static_cast<uint32_t>(bytes[2]) << 16) |
			   (static_cast<uint32_t>(bytes[3]) << 24);
	}
	else if constexpr (std::is_same_v<T, float>)
	{
		const uint32_t uint32Value = unpackValue<uint32_t>(bytes);
		float floatValue;
		memcpy(&floatValue, &uint32Value, sizeof(float));
		return floatValue;
	}
	else
		static_assert(sizeof(T) == 0, "Unsupported type for unpackValue");
}