/*
 * state.c
 *
 *  Created on: 22.11.2009
 *      Author: marcus
 */
#include <stdio.h>
#include <avr/sfr_defs.h>

#include "state.h"

uint8_t volatile state = 0;
uint8_t status_step = 0;
uint8_t registered_tasks = 0;

state_callback state_callbacks[8] =
  { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };

uint8_t
state_register_task(state_callback callback)
{
  if ((registered_tasks < 7) && (callback != NULL))
    {
      state_callbacks[registered_tasks] = callback;
      uint8_t result = _BV(registered_tasks);
      registered_tasks++;
      return result;
    }
  else
    {
      return 0xff;
    }
}

uint8_t
state_register_state()
{
  uint8_t result = registered_tasks;
  registered_tasks++;
  return _BV(result);
}

void
state_activate(uint8_t state_number)
{
  state |= state_number;
}

void
state_deactivate(uint8_t state_number)
{
  state &= ~(state_number);
}

uint8_t
state_is_active(uint8_t state_number)
{
  return state & state_number;
}

void
state_process(void)
{
  //advance one task and ensure that you zero after 7
  status_step = (status_step + 1) & 7;
  if (state & _BV(status_step))
    {
      if (state_callbacks[status_step] != NULL)
        {
          //we can deactivate the state - perhaps it wants to be activated later
          //state_deactivate(status_step);
          //TODO - how do we deal with that?
          //and call the function of the state
          state_callbacks[status_step]();
        }
    }
}
