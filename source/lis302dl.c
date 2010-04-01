/*
 * lis302dl.c
 *
 *  Created on: 24.08.2009
 *      Author: marcus
 */
#include <avr/io.h>
#include <avr/power.h>

#include "lis302dl.h"
#include "i2cmaster.h"

//is least significant bit f the accelerometer (wired on board)
#define LIS302_SDO 0

//general I2C address of the Accelerometer
#define LIS302_I2C_ADDRESS (0x1c + LIS302_SDO)

//some register definitions
#define LIS302_REGISTER_WHO_AM_I 0x0f
#define LIS302_REGISTER_CTRL_REG_1 0x20
#define LIS302_REGISTER_CTRL_REG_2 0x21
#define LIS302_REGISTER_CTRL_REG_3 0x22
#define LIS302_REGISTER_HP_FILTER_SET 0x23
#define LIS302_REGISTER_STATUS 0x27
#define LIS302_REGISTER_OUT_X 0x29
#define LIS302_REGISTER_OUT_Y 0x2b
#define LIS302_REGISTER_OUT_Z 0x2d
#define LIS302_REGISTER_FF_WU_CFG_1 0x30
#define LIS302_REGISTER_FF_WU_SRC_1 0x31
#define LIS302_REGISTER_FF_WU_THS_1 0x32
#define LIS302_REGISTER_FF_WU_DURATION_1 0x33
#define LIS302_REGISTER_FF_WU_CFG_2 0x34
#define LIS302_REGISTER_FF_WU_SRC_2 0x35
#define LIS302_REGISTER_FF_WU_THS_2 0x36
#define LIS302_REGISTER_FF_WU_DURATION_2 0x37
#define LIS302_REGISTER_CLICK_CFG 0x38
#define LIS302_REGISTER_CLICK_SRC 0x39
#define LIS302_REGISTER_THSY_X 0x3b
#define LIS302_REGISTER_THSZ 0x3c
#define LIS302_REGISTER_CLICK_TIME_LIMIT 0x3d
#define LIS302_REGISTER_CLICK_LATENCY 0x3e
#define LIS302_REGISTER_CLICK_WINDOW 0x3f

#define LIS302_WHO_AM_I_VALUE 0x3b

//mask for interrupt 1 config
#define LIS302_INTERUPT1_CONFIG_MASK ~0x07
#define LIS302_INTERUPT2_CONFIG_MASK ~0x38

uint8_t
lis302_read_register(uint8_t register_id);
void
lis302_write_register(uint8_t register_id, uint8_t value);

uint8_t
lis302dl_init(void)
{
  power_twi_enable();
  i2c_init();
  //configure ports
  LIS302_INT1_DDR &= ~(LIS302_INT1_POS);
  LIS302_INT1_PORT &= ~(LIS302_INT1_POS);

  LIS302_INT2_DDR &= ~(LIS302_INT2_POS);
  LIS302_INT2_PORT &= ~(LIS302_INT2_POS);

  uint8_t who_am_i = 0;
  who_am_i = lis302_read_register(LIS302_REGISTER_WHO_AM_I);
  if (LIS302_WHO_AM_I_VALUE != who_am_i)
    {
      return -1;
    }

  //set active & enable axis
  uint8_t ctrl_reg_value = _BV(7) | _BV(6) | _BV(2) | _BV(1) | _BV(0);
  lis302_write_register(LIS302_REGISTER_CTRL_REG_1, ctrl_reg_value);
  uint8_t reg_value = lis302_read_register(LIS302_REGISTER_CTRL_REG_1);
  if (reg_value != ctrl_reg_value)
    {
      return -1;
    }
  else
    {
      return 0;
    }
}

void
lis302dl_selftest(uint8_t selftest)
{
  uint8_t register_value = lis302_read_register(LIS302_REGISTER_CTRL_REG_1);
  if (selftest)
    {
      register_value |= _BV(4) | _BV(3);
    }
  else
    {
      register_value &= ~(_BV(4) | _BV(3));
    }
}

void
lis302dl_full_scale(uint8_t fullscale)
{
  uint8_t register_value = lis302_read_register(LIS302_REGISTER_CTRL_REG_1);
  if (fullscale)
    {
      register_value |= _BV(5);
    }
  else
    {
      register_value &= ~_BV(5);
    }
  lis302_write_register(LIS302_REGISTER_CTRL_REG_1, register_value);
}

void
lis302dl_enable_high_pass_filter(uint8_t enable_filter)
{
  uint8_t register_value = lis302_read_register(LIS302_REGISTER_CTRL_REG_2);
  if (enable_filter)
    {
      register_value |= _BV(4);
    }
  else
    {
      register_value &= ~_BV(4);
    }
  lis302_write_register(LIS302_REGISTER_CTRL_REG_2, register_value);
  //reset the high pass filter
  lis302_read_register(LIS302_REGISTER_HP_FILTER_SET);
}

accel_vect
lis302dl_read_accel(void)
{
  accel_vect return_value;

  /*
   return_value.accel_x = lis302_read_register(LIS302_REGISTER_OUT_X);
   return_value.accel_y = lis302_read_register(LIS302_REGISTER_OUT_Y);
   return_value.accel_z = lis302_read_register(LIS302_REGISTER_OUT_Z);
   */

  i2c_start_wait(I2C_7BIT_WRITE(LIS302_I2C_ADDRESS));
  i2c_write(_BV(7) | LIS302_REGISTER_OUT_X);
  i2c_rep_start(I2C_7BIT_READ(LIS302_I2C_ADDRESS));
  return_value.accel_x = i2c_readAck();
  i2c_readAck();
  return_value.accel_y = i2c_readAck();
  i2c_readAck();
  return_value.accel_z = i2c_readNak();
  i2c_stop();

  return return_value;
}

void
lis302dl_configure_filter(uint8_t filter_bits, uint8_t filter_coeff)
{
  uint8_t register_value = lis302_read_register(LIS302_REGISTER_CTRL_REG_2);

  //mask the input for security
  filter_bits &= 0x1c;
  filter_coeff &= 0x3;

  register_value &= 0xe0; //mask the filter settings
  register_value |= filter_bits | filter_coeff;

  lis302_write_register(LIS302_REGISTER_CTRL_REG_2, register_value);
}

uint8_t
lis302dl_read_status()
{
  return lis302_read_register(LIS302_REGISTER_STATUS);
}

void
lis302dl_configure_interrupt1(uint8_t interrupt_source)
{
  uint8_t interrupt_config = lis302_read_register(LIS302_REGISTER_CTRL_REG_3);
  interrupt_config &= LIS302_INTERUPT1_CONFIG_MASK;
  interrupt_config |= interrupt_source;
  lis302_write_register(LIS302_REGISTER_CTRL_REG_3, interrupt_config);
}

void
lis302dl_configure_interrupt2(uint8_t interrupt_source)
{
  uint8_t interrupt_config = lis302_read_register(LIS302_REGISTER_CTRL_REG_3);
  interrupt_config &= LIS302_INTERUPT1_CONFIG_MASK;
  interrupt_config |= interrupt_source << 3;
  lis302_write_register(LIS302_REGISTER_CTRL_REG_3, interrupt_config);
}

void
lis302dl_enable_click(uint8_t click_enabled)
{
  //TODO should this be configurable?
  //we jsut enable any cli
  if (click_enabled)
    {
      lis302_write_register(LIS302_REGISTER_CLICK_CFG, 0x7f);
    }
  else
    {
      lis302_write_register(LIS302_REGISTER_CLICK_CFG, 0x0);
    }
}

void
lis302_click_configure(uint8_t click_treshhold, uint8_t click_time_limit,
    uint8_t click_latency, uint8_t click_window)
{
  click_treshhold &= 0xf;
  lis302_write_register(LIS302_REGISTER_THSY_X, click_treshhold
      | (click_treshhold << 4));
  lis302_write_register(LIS302_REGISTER_THSZ, click_treshhold);
  lis302_write_register(LIS302_REGISTER_CLICK_TIME_LIMIT, click_time_limit);
  lis302_write_register(LIS302_REGISTER_CLICK_LATENCY, click_latency);
  lis302_write_register(LIS302_REGISTER_CLICK_WINDOW, click_window);
}

uint8_t
lis302_read_clicksource(void)
{
  return lis302_read_register(LIS302_REGISTER_CLICK_SRC);
}

uint8_t
lis302dl_read_who_am_i()
{
  return lis302_read_register(LIS302_REGISTER_WHO_AM_I);
}

void
lis302_write_register(uint8_t register_address, uint8_t value)
{
  i2c_start_wait(I2C_7BIT_WRITE(LIS302_I2C_ADDRESS));
  i2c_write(register_address);
  i2c_write(value);
  i2c_stop();
}

uint8_t
lis302_read_register(uint8_t register_address)
{
  uint8_t result = -1;
  i2c_start_wait(I2C_7BIT_WRITE(LIS302_I2C_ADDRESS));
  i2c_write(register_address);
  i2c_rep_start(I2C_7BIT_READ(LIS302_I2C_ADDRESS));
  result = i2c_readNak();
  i2c_stop();
  return result;
}
