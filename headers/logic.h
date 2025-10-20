/*
 * RoborState.cpp
 *
 * Created: 14.07.2023 15:19:51
 *  Author: Ivan
 */ 


#ifndef LOGIC_H
#define LOGIC_H

#include "math.h"
#include "stdint.h"
#include "stddef.h"


/** The constant \a pi.	*/
#define M_PI		3.14159265358979323846	/* pi */

/** The constant \a pi/2.	*/
#define M_PI_2		1.57079632679489661923	/* pi/2 */

/** The constant \a pi/4.	*/
#define M_PI_4		0.78539816339744830962	/* pi/4 */

#define M_E		2.7182818284590452354


//how often is called main loop in ms
#define loopDelay 5
#define WHEEL_BASE 100.0f //mm
#define SEP "\t"




int strLen0(const char* str);

void logNote(const char * note);
void logNote(int number);
void logNote(float number);
void logNote(size_t number);
void logNotePop();
void printRobotState(bool force = false);

#endif