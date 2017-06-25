/*
 * Utils.cpp
 *
 *  Created on: Jun 25, 2017
 *      Author: Kris
 */

#include "Utils.h"

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
