/*
 * watchdog.c
 *
 *  Created on: 20.12.2009
 *      Author: marcus
 */

#include <stdint.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#include "watchdog.h"

#define WATCHDOG_TIME WDTO_500MS

uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
watchdog_callback watchdog_callback_function = NULL;

//TODO automatically executed
void
get_mcusr(void)
__attribute__((naked))
__attribute__((section(".init3")));
void
get_mcusr(void)
{
  mcusr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

void
watchdog_init(watchdog_callback callback)
{
  cli();
  //first of all register the callback_function
  watchdog_callback_function = callback;

  WDTCSR =_BV(WDCE) | _BV(WDE);
  WDTCSR = _BV(WDIE) | WDTO_1S;
  sei();
}

ISR(WDT_vect)
{
  cli();
  //reset the watchdog to avoid resets!
  //wdt_reset();
  //WDTCSR |= _BV(WDIE);
  //TODO must we ensure that the system is not reset?
  //TODO do this really first
  //first call the handling function
  if (watchdog_callback_function != NULL)
    {
      (*watchdog_callback_function)();
    }
  sei();
}

