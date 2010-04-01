/*
 * status.h
 *
 *  Created on: 22.11.2009
 *      Author: marcus
 */

#ifndef STATUS_H_
#define STATUS_H_

typedef void(*state_callback)(void);

uint8_t state_register_task(state_callback callback);
uint8_t state_register_state();
void state_process(void);
uint8_t state_is_active(uint8_t state_number);
void state_activate(uint8_t state_number);
void state_deactivate(uint8_t state_number);


#endif /* STATUS_H_ */
