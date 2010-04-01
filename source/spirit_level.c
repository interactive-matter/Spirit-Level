/*
 * spirit_level.c
 *
 *  Created on: 17.11.2009
 *      Author: marcus
 */
#include <stdint.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <math.h>
#include <avr/delay.h>

#include "spirit_level.h"
#include "lis302dl.h"
#include "display.h"
#include "kalman.h"
#include "state.h"
#include "watchdog.h"

//our state variables
volatile int8_t state_calculate = 0;
volatile int8_t state_spirit_level_active = 0;

//the axis definitions (numerical names)
#define NO_AXIS 0
#define X_AXIS 1
#define Y_AXIS 2
#define Z_AXIS 3
//which axis is measured
volatile uint8_t measurement_axis = Z_AXIS;

//how many degrees are still standing?
#define NO_MOVE_TRESHOLD 5
#define SLEEP_AFTER_TICKS 30

//The mask for the accelerator interrupts
#define ACC_INTERRUPT_MASK (_BV(2) | _BV(3))

//click configuration
#define AC_CLICK_TRESH 10 //0.5g
#define ACC_CLICK_TIME 0xff //0.5ms
#define ACC_CLICK_LATENCY 0xff //1ms
#define ACC_CLICK_WINDOW 0xff //1ms
//the displayed angle
volatile int16_t angle_int = 0;
double angle_float = 0;
double correction_x = 0.0;
double correction_y = 0.0;
double correction_z = 0.0;
uint8_t calibration_wait = 0;
//which axis is to be calibrated
uint8_t calibration_axis = -1;
//how long do we wait after the calibration request to settle
#define CALIBRATION_DELAY 10

//our three kalman states for the vectors
kalman_state x_state;
kalman_state y_state;
kalman_state z_state;

#define PROCESS_NOISE 0.1
#define SENSOR_NOISE 64.0
#define INITIAL_Q 100.0

void
start_level_timer();
void
spirit_level_watchdog_callback(void);
void
handle_click(void);

void
spirit_level_init()
{
  //we assume the display is already running
  display_show_number(1);
  //now start & configure the lis302dl
  lis302dl_init();
  display_show_number(2);
  uint8_t hello = lis302dl_read_who_am_i();
  display_show_number(hello);
  _delay_ms(10);
  lis302dl_full_scale(0);
  lis302dl_configure_filter(0, 0);
  //configure click interrupt
  lis302dl_configure_interrupt1(LIS302_INT_CLICK);
  lis302dl_enable_click(-1);
  lis302_click_configure(AC_CLICK_TRESH, ACC_CLICK_TIME, ACC_CLICK_LATENCY,
      ACC_CLICK_WINDOW);
  //enable the interrupt inputs
  DDRC |= ACC_INTERRUPT_MASK;
  PCICR |= PCIE1;

  // register our states
  state_calculate = state_register_task(&spirit_level_calculate);
  state_activate(state_calculate);
  state_spirit_level_active = state_register_state();
  state_activate(state_spirit_level_active);

  x_state = kalman_init(PROCESS_NOISE, SENSOR_NOISE, INITIAL_Q, 0);
  y_state = kalman_init(PROCESS_NOISE, SENSOR_NOISE, INITIAL_Q, 0);
  z_state = kalman_init(PROCESS_NOISE, SENSOR_NOISE, INITIAL_Q, 0);

  start_level_timer();

  //we want to be informed if there is no movement for a certain period of time
  watchdog_init(*spirit_level_watchdog_callback);
}

void
spirit_level_calculate()
{
  if (state_is_active(state_calculate))
    {
      accel_vect accel = lis302dl_read_accel();

      kalman_update(&x_state, (double) accel.accel_x);
      kalman_update(&y_state, (double) accel.accel_y);
      kalman_update(&z_state, (double) accel.accel_z);

      double current_angle = 0.0;
      if (measurement_axis == X_AXIS)
        {
          current_angle = atan2(y_state.x, -(z_state.x)) * 180.0 / M_PI;
        }
      else if (measurement_axis == Y_AXIS)
        {
          current_angle = atan2(x_state.x, -(z_state.x)) * 180.0 / M_PI;
        }
      else if (measurement_axis == Z_AXIS)
        {
          current_angle = atan2(x_state.x, -(y_state.x)) * 180.0 / M_PI;
        }

      if (measurement_axis == X_AXIS)
        {
          angle_int = round(current_angle - correction_x);
        }
      else if (measurement_axis == Y_AXIS)
        {
          angle_int = round(current_angle - correction_x);
        }
      else if (measurement_axis == Z_AXIS)
        {
          angle_int = round(current_angle - correction_x);
        }
      display_show_number(angle_int);

      //do we have to perform a calibration?
      if (calibration_axis != NO_AXIS)
        {
          if (calibration_wait == 0)
            {
              if (calibration_axis == X_AXIS)
                {
                  correction_x = current_angle;
                }
              else if (calibration_axis == Y_AXIS)
                {
                  correction_y = current_angle;
                }
              else if (calibration_axis == Z_AXIS)
                {
                  correction_z = current_angle;
                }
              calibration_axis=NO_AXIS;
            } else {
              calibration_wait--;
            }
          state_deactivate(state_calculate);
        }

    }
  //TODO just a test
  handle_click();
}

//TODO static?
int16_t old_angle = 0;
uint8_t no_move_since = 0;
void
spirit_level_watchdog_callback(void)
{
  //first of all compute the last movement
  if (abs(old_angle - angle_int) < NO_MOVE_TRESHOLD)
    {
      if (no_move_since < SLEEP_AFTER_TICKS)
        {
          no_move_since++;
        }
    }
  else
    {
      no_move_since = 0;
    }
  //then we check if we have to switch the display on or off
  if ((no_move_since == SLEEP_AFTER_TICKS) && state_is_active(
      state_spirit_level_active))
    {
      state_deactivate(state_spirit_level_active);
      display_disable();
    }
  else if ((no_move_since == 0)
      && (!state_is_active(state_spirit_level_active)))
    {
      state_activate(state_spirit_level_active);
      display_enable();
    }
  old_angle = angle_int;
}

void
handle_click(void)
{
  uint8_t click_source = lis302_read_clicksource();
  //todo generalize
  if (click_source & _BV(0))
    {
      //x single click
      measurement_axis = Z_AXIS;
    }
  else if (click_source & _BV(1))
    {
      //x double click
      calibration_axis = Z_AXIS;
      calibration_wait = CALIBRATION_DELAY;
    }
  else if (click_source & _BV(2))
    {
      //y single click
      measurement_axis = Y_AXIS;
    }
  else if (click_source & _BV(3))
    {
      //y double click
      calibration_axis = Y_AXIS;
      calibration_wait = CALIBRATION_DELAY;
    }
  else if (click_source & _BV(4))
    {
      //z single click
      measurement_axis = X_AXIS;
    }
  else if (click_source & _BV(5))
    {
      //z double click
      calibration_axis = X_AXIS;
      calibration_wait = CALIBRATION_DELAY;
    }

}

void
acc_int2(void)
{

}

void
start_level_timer(void)
{
  power_timer2_enable();
  TCCR2A = 0;
  //the cpu runs on 8Mhz the lis302 delivers data at 400HZ
  //by that we have new data every 20000step -so it is ore than safe
  //to get data only each 1024th step
  TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
  TIMSK2 = _BV(TOIE2);
  ASSR = 0;
}

ISR(TIMER2_OVF_vect)
{
  if (!state_is_active(state_calculate))
    {
      //is there new DATA?
      if (lis302dl_read_status() & LIS302_STATUS_ZYXDA)
        {
          state_activate(state_calculate);
        }
    }
}

//the PCI1 interrupt for detecting accelerometer interrupts
ISR(PCINT1_vect)
{
  uint8_t interrupts = PINC & ACC_INTERRUPT_MASK;
  if (interrupts & _BV(2))
    {
      acc_int2();
    }
  if (interrupts & _BV(3))
    {
      handle_click();
    }
}
