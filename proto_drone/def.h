/*
 * def.h
 *
 *  Created on: Jun 25, 2017
 *      Author: Kris
 */

#ifndef DEF_H_
#define DEF_H_

#include <avr/io.h>

#ifndef __AVR_ATmega328P__
#error Only ATMega328p supported.
#endif

#define DEBUG

#define LOOPTIME_US 3000
#define DELTA_TIME_US (LOOPTIME_US + 67000)


#endif /* DEF_H_ */
