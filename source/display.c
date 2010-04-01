/*
 * display.c
 *
 *  Created on: 16.11.2009
 *      Author: marcus
 */
#include <stdint.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

#include "display.h"

#define DIGIT1 _BV(0)
#define DIGIT2 _BV(1)
#define DIGIT3 _BV(2)
#define DIGIT4 _BV(1)

#define SEGMENT1 _BV(0)
#define SEGMENT2 _BV(1)
#define SEGMENT3 _BV(2)
#define SEGMENT4 _BV(3)
#define SEGMENT5 _BV(4)
#define SEGMENT6 _BV(5)
#define SEGMENT7 _BV(6)
#define SEGMENT8 _BV(7)

#define DISPLAY_PORT1 PORT_C
#define DISPLAY_PORT2 PORT_B

//  _8
// |3 |7
//  _2
// |5 |6
//  _4 .1
static uint8_t numbers[] =
  { SEGMENT3 | SEGMENT4 | SEGMENT5 | SEGMENT6 | SEGMENT7 | SEGMENT8, //0
      SEGMENT6 | SEGMENT7, //1
      SEGMENT8 | SEGMENT7 | SEGMENT2 | SEGMENT5 | SEGMENT4, //2
      SEGMENT8 | SEGMENT7 | SEGMENT2 | SEGMENT6 | SEGMENT4, //3
      SEGMENT3 | SEGMENT7 | SEGMENT2 | SEGMENT6, //4
      SEGMENT8 | SEGMENT3 | SEGMENT2 | SEGMENT6 | SEGMENT4, //5
      SEGMENT8 | SEGMENT3 | SEGMENT2 | SEGMENT5 | SEGMENT6 | SEGMENT4, //6
      SEGMENT8 | SEGMENT7 | SEGMENT6, //7
      SEGMENT8 | SEGMENT3 | SEGMENT7 | SEGMENT2 | SEGMENT5 | SEGMENT6
          | SEGMENT4, //8
      SEGMENT8 | SEGMENT3 | SEGMENT7 | SEGMENT2 | SEGMENT6 | SEGMENT4, //9
      0, // nothing
      SEGMENT2, // -
      SEGMENT1 //ERROR
    };

#define NOTHING 10
#define MINUS_SIGN 11
#define ERROR_SIGN 12

volatile uint8_t value[] =
  { 0, 0, 0, 0 };
uint8_t display_value[] =
  { 0, 0, 0, 0 };
volatile uint8_t value_locked = 0;
volatile uint8_t segment = 0;

void
display_init(void)
{
  //enable the outputs
  DDRB |= DIGIT1 | DIGIT2 | DIGIT3;
  DDRC |= DIGIT4;
  //and enable the segment outputs
  DDRD = 255; // we use all anyhow

  //just a short self test
  PORTB |= (DIGIT1 | DIGIT2 | DIGIT3);
  PORTC |= (DIGIT4);
  PORTD = 0;
  _delay_ms(100);
  PORTD = 255;

  //disable the output
  PORTB &= ~(DIGIT1 | DIGIT2 | DIGIT3);
  PORTC &= ~(DIGIT4);

  PORTD = 255;

  display_show_number(1234);

}

void
display_enable()
{
  power_timer0_enable();
  //nothing to set on TCCR0A
  TCCR0A = 0;
  TCCR0B = _BV(CS01) | _BV(CS00);
  TIMSK0 = _BV(TOIE0);
  segment = 0;
}

void
display_disable()
{
  PORTD = 255;
  PORTB &= ~(DIGIT1 | DIGIT2 | DIGIT3);
  PORTC &= ~(DIGIT4);
  TCCR0B = 0;
  power_timer0_disable();
}

void
display_show_number(int16_t number)
{
  int16_t n = abs(number);
  uint8_t i = 0;
  value_locked = 1;
  //TODO how to not display leading 0
  for (; i < 3 && n > 0; i++)
    {
      value[i] = numbers[(uint8_t) (n % 10)];
      n /= 10;
    }
  if (i == 0)
    {
      value[i] = numbers[0];
      i++;
    }
  if (number < 0)
    {
      value[i] = numbers[MINUS_SIGN];
      i++;
    }
  for (; i < 4; i++)
    {
      value[i] = numbers[NOTHING];
    }
  value_locked = 0;
}

ISR(TIMER0_OVF_vect)
{
  //copy value
  if (segment == 0 && value_locked == 0)
    {
      for (int i = 0; i < 4; i++)
        {
          display_value[i] = value[i];
        }
    }
  //switch off
  PORTD = 255;
  PORTB &= ~(DIGIT1 | DIGIT2 | DIGIT3);
  PORTC &= ~(DIGIT4);
  switch (segment)
    {
  case 0:
    PORTB |= DIGIT1;
    break;
  case 1:
    PORTB |= DIGIT2;
    break;
  case 2:
    PORTB |= DIGIT3;
    break;
  case 3:
    PORTC |= DIGIT4;
    break;
    }
  PORTD = ~value[segment];
  //next segment
  segment++;
  segment &= 3; //4 bits are enough
}
