#pragma once

#include <Arduino.h>

/// @brief Base class for mock serial sources used in testing: holds the data a mock device sends, and when each
/// @brief byte arrives. The data is added as arrays, each with a pause. An array arrives its pause after the
/// @brief previous one arrived (or after start() for the first), as a real device's data does
class Mock_Serial_Base
{
protected:
	// No default member initializers, as they can't be used inside the enclosing class. Only used in an array
	// initialised with = {}, which zeroes every element
	struct DataArray final
	{
		uint32_t pauseMillis;
		const uint8_t *data;
		size_t size;
	};

private:
	static constexpr uint8_t MAX_DATA_ARRAYS = 16;

	DataArray _dataArrays[MAX_DATA_ARRAYS] = {};
	uint8_t _dataArrayCount = 0;

	uint32_t _startMillis = 0; // When the first data array's pause started
	bool _started = false;

protected:
	// Only created as part of a derived mock
	Mock_Serial_Base() = default;

	/// @brief Start the data arriving. Only the first call starts it, as a real device sends regardless of
	/// @brief anything restarting at the receiving end
	void start()
	{
		if (!_started)
		{
			_startMillis = millis();
			_started = true;
		}
	}

	/// @brief Get how many data arrays have been added
	/// @return Number of data arrays
	uint8_t getArrayCount() const
	{
		return _dataArrayCount;
	}

	/// @brief Get a data array
	/// @param index The data array's position, in the order added. Must be less than getArrayCount()
	/// @return The data array
	const DataArray &getArray(const uint8_t index) const
	{
		return _dataArrays[index];
	}

	/// @brief Calculate how many data arrays have arrived so far
	/// @return Number of data arrays arrived. 0 before start()
	uint8_t calcArraysArrived() const
	{
		if (!_started)
			return 0;

		const uint32_t elapsedMillis = millis() - _startMillis;

		uint32_t arrivalMillis = 0;
		uint8_t arraysArrived = 0;
		for (uint8_t index = 0; index < _dataArrayCount; ++index)
		{
			arrivalMillis += _dataArrays[index].pauseMillis;

			if (elapsedMillis < arrivalMillis)
				break;

			++arraysArrived;
		}

		return arraysArrived;
	}

	/// @brief Calculate how many bytes have arrived so far, across every data array
	/// @return Number of bytes arrived. 0 before start()
	size_t calcBytesArrived() const
	{
		const uint8_t arraysArrived = calcArraysArrived();

		size_t bytesArrived = 0;
		for (uint8_t index = 0; index < arraysArrived; ++index)
			bytesArrived += _dataArrays[index].size;

		return bytesArrived;
	}

	/// @brief Calculate how many bytes the data holds, across every data array
	/// @return Number of bytes
	size_t calcDataSize() const
	{
		size_t dataSize = 0;
		for (uint8_t index = 0; index < _dataArrayCount; ++index)
			dataSize += _dataArrays[index].size;

		return dataSize;
	}

	/// @brief Get a byte of the data, counting across every data array, e.g. position 0 is the first data array's
	/// @brief first byte
	/// @param position The byte's position in the data. Must be less than calcDataSize()
	/// @return The byte
	uint8_t getByte(size_t position) const
	{
		uint8_t index = 0;
		while (position >= _dataArrays[index].size)
		{
			position -= _dataArrays[index].size;
			++index;
		}

		return _dataArrays[index].data[position];
	}

public:
	/// @brief Add a data array to the end of the data
	/// @tparam SIZE The array's size (bytes)
	/// @param pauseMillis The pause before the array arrives, after the previous one arrived (ms)
	/// @param data The bytes that arrive. Must stay valid while the mock is used
	/// @return True if added, false if MAX_DATA_ARRAYS have already been added
	template <size_t SIZE>
	bool addData(const uint32_t pauseMillis, const uint8_t (&data)[SIZE])
	{
		if (_dataArrayCount >= MAX_DATA_ARRAYS)
			return false;

		_dataArrays[_dataArrayCount] = {.pauseMillis = pauseMillis, .data = data, .size = SIZE};
		++_dataArrayCount;
		return true;
	}
};
