/*
 * watchdog.h
 *
 *  Created on: 20.12.2009
 *      Author: marcus
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

typedef void
(*watchdog_callback)(void);

void
watchdog_init(watchdog_callback callback);

#endif /* WATCHDOG_H_ */
