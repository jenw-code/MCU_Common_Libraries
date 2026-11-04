#pragma once

enum class ErrorCategory : uint8_t
{
	None = 0,
	Battery = 1,
	General = 100
};

enum class GeneralError : uint8_t
{
	None = 0
};

struct AppError final
{
	ErrorCategory category = ErrorCategory::None;
	uint8_t code = 0;

	void reset()
	{
		*this = {};
	}
};