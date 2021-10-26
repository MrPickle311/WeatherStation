#include "I2CSensor.hpp"

namespace Device
{

enum class HTS22_OutputDataRate
{
	Rate_1Hz = 0b01 ,
	Rate_7Hz = 0b10 ,
	Rate_12_5Hz = 0b11
};

class HTS22 : public I2C_Sensor
{
	constexpr static uint8_t dev_adr = 95 << 1;

	constexpr static uint8_t CTRL_REG1 = 0x20;
	constexpr static uint8_t TEMP_OUT_L = 0x2A;
	constexpr static uint8_t TEMP_OUT_H = 0x2B;
	constexpr static uint8_t THUMIDITY_OUT_L = 0x28;
	constexpr static uint8_t THUMIDITY_OUT_H = 0x29;

	constexpr static uint8_t H0_rH_x2 = 0x30;
	constexpr static uint8_t H1_rH_x2 = 0x31;
	constexpr static uint8_t H0_T0_OUT_0 = 0x36;
	constexpr static uint8_t H0_T0_OUT_1 = 0x37;
	constexpr static uint8_t H1_T0_OUT_0 = 0x3A;
	constexpr static uint8_t H1_T0_OUT_1 = 0x3B;
private:
	uint16_t concatWord(uint8_t high_byte , uint8_t low_byte);
public:
	HTS22(I2C_Bus& bus);
	void enable(const HTS22_OutputDataRate);
	int16_t getTemperature();
	uint16_t getHumidity();
};

}
