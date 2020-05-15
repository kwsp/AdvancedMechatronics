#include "lsm6ds33.h"

void lsm_write(unsigned char reg_addr, unsigned char data) {
    i2c_master_start();         // Send start bit
    i2c_master_send(LSM_WRITE_ADDR);// Send I2C address with write bit
    i2c_master_send(reg_addr);  // Send register to write to
    i2c_master_send(data);      // Send data to write
    i2c_master_stop();          // Send stop bit
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
//    lsm_write()
    
    
    /* 
     * 
     * To turn on the gyroscope 
     * 
     */
    // Write to the CTRL2_G register
    
    // Set the sample rate to 1.66 kHz, with 1000 dps sensitivity.
    

}