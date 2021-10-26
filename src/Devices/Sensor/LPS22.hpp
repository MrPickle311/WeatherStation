#pragma once

#include "I2CSensor.hpp"

namespace Device
{

enum class LPS22_OutputDataBitRate : uint8_t
{
	Rate_1Hz = 0b001,
	Rate_10Hz = 0b010,
	Rate_25Hz = 0b011,
	Rate_50Hz = 0b100,
	Rate_75Hz = 0b101
};

class LPS_22 : public I2C_Sensor
{
	using addr_t = volatile uint8_t;

	constexpr static addr_t dev_adr { 0x5D /*93*/ << 1};

	constexpr static addr_t TEMP_OUT_L { 0x2B };
	constexpr static addr_t TEMP_OUT_H { 0x2C };
	constexpr static addr_t CTRL_REG1  { 0x10 };
	constexpr static addr_t CTRL_REG2  { 0x11 };
	constexpr static addr_t STATUS_REG { 0x27 };

	constexpr static addr_t PRESS_OUT_XL{ 0x28 };
	constexpr static addr_t PRESS_OUT_L { 0x29 };
	constexpr static addr_t PRESS_OUT_H { 0x2A };
public:
	LPS_22(I2C_Bus& bus);
public:
	void enable(const LPS22_OutputDataBitRate rate);
	uint32_t readPressureRaw();
	float readPressureMillibars(uint32_t raw_pressure);
};



}
