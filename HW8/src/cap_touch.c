#include "cap_touch.h"

int capacitance_base_1;
int capacitance_base_2;

int capacitance_1;
int capacitance_2;

int CTMU_N_READS = 10;
int CTMU_DELAY = 100;
int i;


/*
 * Measure capacitance values for both sensors
 */
void _cap_read(int *val1, int *val2) {
    *val1 = 0;
    for (i=0; i<CTMU_N_READS; ++i) {
        *val1 += ctmu_read(5, CTMU_DELAY);
    }
    *val1 /= CTMU_N_READS;

    *val2 = 0;
    for (i=0; i<CTMU_N_READS; ++i) {
        *val2 += ctmu_read(4, CTMU_DELAY);
    }
    *val2 /= CTMU_N_READS;
}

void cap_calibrate() {
    _cap_read(&capacitance_base_1, &capacitance_base_2);
}

int cap_get_pos() {
    // Measure current values
    _cap_read(&capacitance_1, &capacitance_2);

    int delta_left = capacitance_base_1 - capacitance_1;
    int delta_right = capacitance_base_2 - capacitance_2;

    delta_left = delta_left < 1 ? 1 : delta_left;
    delta_right = delta_right < 1 ? 1 : delta_right;
    
    int left_pos = (delta_left * 100)/(delta_left + delta_right);
    int right_pos = ((1 - delta_right) * 100)/(delta_left + delta_right);
    int pos = (left_pos + right_pos)/2;

    return delta_left;
}
