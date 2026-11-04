#pragma once

#include "common.math.h"

namespace BATTERY_CONFIG
{
	inline constexpr unsigned long LEVEL_AUTO_OFF_MILLIS = SECONDS_TO_MILLIS(20); // How soon to auto turn off level display

	inline constexpr unsigned long HEARTBEAT_MILLIS = MINUTES_TO_MILLIS(10); // How often to check that the IC is okay
	inline constexpr unsigned long RETRY_MILLIS = 500;						 // How often to re-try initialising IC in case there's an error

	namespace LEVEL_MIN_PERCENT
	{
		inline constexpr uint8_t VERYGOOD = 78;
		inline constexpr uint8_t GOOD = 56;
		inline constexpr uint8_t POOR = 34;
		inline constexpr uint8_t VERYPOOR = 12; // A value below this is treated as critical
	}

}

//------------------------------------- BATTERY MANAGEMENT -----------------------------------------

enum class BatteryLevel : uint8_t
{
	Unknown = 0,
	Critical = 1,
	VeryPoor = 2,
	Poor = 3,
	Good = 4,
	VeryGood = 5,

	MIN = Unknown,
	MAX = VeryGood
};

enum class BatteryError : uint8_t
{
	None = 0,
	Management_IC_Setup_Fail = 1,
	Voltage_Dropping_When_Charging = 2, // Most likely there's no battery connected, or battery charge extremely low
	CellPercent_Invalid_Low = 3,		// Most likely there's no battery connected, or battery charge extremely low
	CellPercent_Invalid_High = 4,		// Most likely there's no battery connected
	Low_Voltage = 5,
	High_Voltage = 6,

	MIN = None,
	MAX = High_Voltage
};

// Container for battery status
struct BatteryStatus final
{
	BatteryLevel level = BatteryLevel::Unknown; // Battery level enum
	BatteryError error = BatteryError::None;	// Battery errorcode enum
};

// Container for battery management state
struct BatteryManagementState final
{
	BatteryStatus batteryStatus; // Status of the battery
	bool isCharging = false;	 // True if the device is charging
	bool inStandby = false;		 // True if the device is in standby (i.e. plugged in but not charging)

	bool usbPluggedIn() const
	{
		return isCharging || inStandby;
	}
};

//---------------------------------- BATTERY MANAGEMENT SETTINGS -----------------------------------

enum class BatteryLevelDisplay : uint8_t
{
	AutoOff = 0,   // Default value to auto hide good battery levels
	AlwaysOn = 1,  // Continuously show battery level
	AlwaysOff = 2, // Never show battery level
};

// Container for battery management settings (most of these are set by jumpers on PCB)
struct BatteryManagementSettings final
{
	BatteryLevelDisplay levelDisplay = BatteryLevelDisplay::AutoOff;
};