/*
 * lis302dl.h
 *
 *  Created on: 24.08.2009
 *      Author: marcus
 */

#ifndef LIS302DL_H_
#define LIS302DL_H_

#include <inttypes.h>

#define LIS302_INT1_PORT PORTC
#define LIS302_INT1_DDR DDRC
#define LIS302_INT1_PIN PINC
#define LIS302_INT1_POS _BV(3)

#define LIS302_INT2_PORT PORTC
#define LIS302_INT2_DDR DDRC
#define LIS302_INT2_PIN PINC
#define LIS302_INT2_POS _BV(2)

#define LIS302_STATUS_ZYXOR _BV(7)
#define LIS302_STATUS_ZOR _BV(6)
#define LIS302_STATUS_YOR _BV(5)
#define LIS302_STATUS_XOR _BV(4)
#define LIS302_STATUS_ZYXDA _BV(3)
#define LIS302_STATUS_ZDA _BV(2)
#define LIS302_STATUS_YDA _BV(1)
#define LIS302_STATUS_XDA _BV(0)

#define LIS302_FDS_ENABLE _BV(4)
#define LIS302_HP_FF_WU1 _BV(2)
#define LIS302_HP_FF_WU2 _BV(3)

#define LIS302_INT_NONE 0
#define LIS302_INT_FF_WU_1 1
#define LIS302_INT_FF_WU_2 2
#define LIS302_INT_FF_WU_ANY 3
#define LIS302_INT_DATA_RDY 6
#define LIS302_INT_CLICK 7

typedef struct _accel_vect
{
  int8_t accel_x;
  int8_t accel_y;
  int8_t accel_z;
} accel_vect;

uint8_t
lis302dl_init(void);

void
lis302dl_selftest(uint8_t selftest);

void
lis302dl_full_scale(uint8_t fullscale);

accel_vect
lis302dl_read_accel(void);

void
lis302dl_configure_filter(uint8_t filter_bits, uint8_t filter_coeff);

void
lis302dl_configure_interrupt1(uint8_t interrupt_source);

void
lis302dl_configure_interrupt2(uint8_t interrupt_source);

uint8_t
lis302dl_read_who_am_i();

uint8_t
lis302dl_read_status();

uint8_t
lis302_read_clicksource(void);

void
lis302_click_configure(uint8_t click_treshhold, uint8_t click_time_limit,
    uint8_t click_latency, uint8_t click_window);

void
lis302dl_enable_click(uint8_t click_enabled);

#endif /* LIS302DL_H_ */
