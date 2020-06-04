#ifndef CAP_TOUCH__
#define CAP_TOUCH__

#include "adc.h"

/*
 * Measure capacitance values for both sensors
 */
void _cap_read(int *val1, int *val2);
void cap_calibrate();
int cap_get_pos();

#endif
