#pragma once

#include "i2c.hpp"

//void resetLPS22()
//{
//	reg_t soft_reset_val = 0x4;
//	reg_t mem_reset_val = 0x80;
//	reg_t auto_incremet_disable_val = 0x10;
//
//	Device::I2C_Bus::get(I2C1).writeToExternRegister(dev_adr, CTRL_REG2, soft_reset_val);
//	Device::I2C_Bus::get(I2C1).writeToExternRegister(dev_adr, CTRL_REG2, mem_reset_val);
//	Device::I2C_Bus::get(I2C1).writeToExternRegister(dev_adr, CTRL_REG2, auto_incremet_disable_val);
//}

float readPressureMillibars(uint32_t raw_pressure)
{
  return (float)raw_pressure / 4096;
}
