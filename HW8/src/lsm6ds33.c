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
    uint8_t whoami;
    while (whoami != LSM_WHO_AM_I_VALUE) {
        whoami = lsm_who_am_i();
    }
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
    // Read i2c 14 times to get the 14 uint8_t
    i2c_read_multiple(LSM_ADDR, LSM_OUT_TEMP_L, data->buffer, 14);
    
    // Reconstruct into 7 int16_t
    /*data->temp = BUFFER[1] << 8 | BUFFER[0];*/
    /*data->gyro_x = BUFFER[3] << 8 | BUFFER[2];*/
    /*data->gyro_y = BUFFER[5] << 8 | BUFFER[4];*/
    /*data->gyro_z = BUFFER[7] << 8 | BUFFER[6];*/
    /*data->acc_x = BUFFER[9] << 8 | BUFFER[8];*/
    /*data->acc_y = BUFFER[11] << 8 | BUFFER[10];*/
    /*data->acc_z = BUFFER[13] << 8 | BUFFER[12];*/
}
