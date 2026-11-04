#pragma once

#include "common.battery.h"

template <uint16_t MIN_MV_VAL, uint16_t MAX_MV_VAL, uint16_t CAPACITY_MAH_VAL, uint8_t CELL_COUNT_VAL>
struct BatteryInfo final
{
	static_assert(MIN_MV_VAL < MAX_MV_VAL, "MIN_MV must be less than MAX_MV.");
	static_assert(MAX_MV_VAL <= (4500 * CELL_COUNT_VAL), "MAX_MV exceeds safe LiPo limits (>4.5V).");
	static_assert(CAPACITY_MAH_VAL > 0, "CAPACITY_MAH must be greater than 0.");
	static_assert(CELL_COUNT_VAL > 0, "CELL_COUNT must be at least 1.");

	static constexpr uint16_t MIN_MV = MIN_MV_VAL;
	static constexpr uint16_t MAX_MV = MAX_MV_VAL;
	static constexpr uint16_t CAPACITY_MAH = CAPACITY_MAH_VAL;
	static constexpr uint8_t CELL_COUNT = CELL_COUNT_VAL;
	static constexpr uint16_t DESIGN_ENERGY_MWH = (static_cast<uint32_t>(CAPACITY_MAH_VAL) * 37 * CELL_COUNT) / 10;
};

namespace BATTERIES_1S
{
	inline constexpr BatteryInfo<3200, 4200, 3050, 1> MAKERHAWK_3000;
	inline constexpr BatteryInfo<3200, 4200, 3700, 1> LP103395;
	inline constexpr BatteryInfo<3200, 4200, 3700, 1> LP103395_33;
	inline constexpr BatteryInfo<3200, 4200, 2700, 1> LP605575;
	inline constexpr BatteryInfo<3200, 4200, 2300, 1> LP803090;
	inline constexpr BatteryInfo<3200, 4200, 2000, 1> LP585460;
	inline constexpr BatteryInfo<3200, 4200, 2000, 1> LP803465;
	inline constexpr BatteryInfo<3200, 4200, 2000, 1> LP803860;
}