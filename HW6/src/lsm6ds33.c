#include <string.h>
#include "lsm6ds33.h"

uint8_t BUFFER[14];

void lsm_write(uint8_t reg_addr, uint8_t data) {
    i2c_write(LSM_ADDR, reg_addr, data);
}

uint8_t lsm_read(uint8_t reg_addr) {
    return i2c_read(LSM_ADDR, reg_addr);
}

void lsm_init() {
    /* 
     * 
     * To turn on the accelerometer
     * 
     */
    // Write to the CTRL1_XL register
    // Set the sample rate to 1.66 kHz, 
    // with 2g sensitivity, and 100 Hz filter.
    lsm_write(LSM_CTRL1_XL_REGISTER, 0b10000010);
    
    /* 
     * 
     * To turn on the gyroscope 
     * 
     */
    // Write to the CTRL2_G register
    // Set the sample rate to 1.66 kHz, with 1000 dps sensitivity.
    lsm_write(LSM_CTRL2_G_REGISTER, 0b10001000);

    // Write to CTRL3_C
    // Make IF_INC bit 1 to enable the ability to read from multiple
    // registers in a row without specifying every reg location
    lsm_write(LSM_CTRL3_C_REGISTER, 0b00000100);
}

uint8_t lsm_who_am_i() {
    return lsm_read(LSM_WHO_AM_I_REGISTER);
}

/*
 */
void lsm_read_sensors(lsm_data *data) {
    int i;
    // Read i2c 14 times to get the 14 uint8_t
    i2c_read_multiple(LSM_ADDR, LSM_OUT_TEMP_L, BUFFER, 14);
    // Reconstruct into 7 int16_t
    // TODO: these readings don't work
    data->temp = (BUFFER[0] << 8) | BUFFER[1];
    data->gyro_x = (BUFFER[2] << 8) | BUFFER[3];
    data->gyro_y = (BUFFER[4] << 8) | BUFFER[5];
    data->gyro_z = (BUFFER[6] << 8) | BUFFER[7];
    data->acc_x = (BUFFER[8] << 8) | BUFFER[9];
    data->acc_y = (BUFFER[10] << 8) | BUFFER[11];
    data->acc_z = (BUFFER[12] << 8) | BUFFER[13];
}
