#include "as5600.h"
#include "i2c.h"
#include "math_utils.h"

static int i2cWrite(uint8_t dev_addr, uint8_t *pData, uint32_t count)
{
  int status;
  int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;

  status = HAL_I2C_Master_Transmit(&AS5600_I2C_HANDLE, dev_addr, pData, count, i2c_time_out);
  return status;
}

static int i2cRead(uint8_t dev_addr, uint8_t *pData, uint32_t count)
{
  int status;
  int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;

  status = HAL_I2C_Master_Receive(&AS5600_I2C_HANDLE, (dev_addr | 1), pData, count, i2c_time_out);
  return status;
}

uint16_t as5600GetRawAngle(void)
{
  uint16_t raw_angle;
  uint8_t buffer[2] = {0};
  uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;

  i2cWrite(AS5600_ADDR, &raw_angle_register, 1);
  i2cRead(AS5600_ADDR, buffer, 2);
  raw_angle = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
  return raw_angle;
}

float as5600GetAngleWithoutTrack(void)
{
  return as5600GetRawAngle() * 0.08789f * _PI / 180; // 得到弧度制的角度
}
