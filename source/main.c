/*
 * main.c
 *
 *  Created on: 16.11.2009
 *      Author: marcus
 */
//TODO why does it pretend that 1Mhz is the real clock_FREQ
#define F_CPU 8000000UL

#include <stdint.h>
#include <avr/delay.h>
#include <avr/power.h>
#include <avr/interrupt.h>

#include "display.h"
#include "spirit_level.h"
#include "watchdog.h"

int
main(void)
{
  //you never know if there is a watchdog active - lets reset it!
  //TODO needed?
  //get_mcusr();
  power_all_disable();


  display_init();

  display_enable();

  //enable interrupts
  sei();

  //a simple self test
  for (int16_t i = 0; i < 10; i++)
    {
      //TOOD why does the inversion not work?
      int16_t value = /*-1*i**/(i * 111);
      display_show_number(value);
      _delay_ms(10);
    }

  spirit_level_init();

  while (1)
    {
      state_process();
    }
}
