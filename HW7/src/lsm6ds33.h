#ifndef LSM6DS33_H__
#define LSM6DS33_H__
/*
 Utilities for communicating with the LSM6DS33 IMU
 via I2C
 */
#include "stdint.h"
#include "i2c.h"

// LSM Chip Address
#define LSM_ADDR 0b11010110

// Register addresses and other constants
#define LSM_WHO_AM_I_VALUE 0x69
#define LSM_WHO_AM_I_REGISTER 0x0F
#define LSM_CTRL1_XL_REGISTER 0x10
#define LSM_CTRL2_G_REGISTER 0x11
#define LSM_CTRL3_C_REGISTER 0x12
#define LSM_OUT_TEMP_L 0x20
#define LSM_OUTZ_H_XL 0x21

typedef struct {
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
} _lsm_data;

typedef union {
    _lsm_data data;
    uint8_t buffer[14];
} lsm_data;

/*
 * Low-level functions
 */
void lsm_init();      // Configure PIC pins to communicate with LSM
void lsm_write(uint8_t reg_addr, uint8_t data);
uint8_t lsm_read(uint8_t reg_addr);
uint8_t lsm_who_am_i();  // Read the who_am_i register from LSM

/*
 * Mid-level functions
 */
void lsm_read_sensors(lsm_data *sensor_data);

#endif
