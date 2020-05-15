#ifndef LSM6DS33_H__
#define LSM6DS33_H__
/*
 Utilities for communicating with the LSM6DS33 IMU
 via I2C
 */
#include "i2c.h"

// LSM Chip Address
#define LSM_WRITE_ADDR 0b0
#define LSM_READ_ADDR 0b0

// Register addresses and other constants
#define LSM_WHO_AM_I_VALUE 0x69
#define LSM_WHO_AM_I_REGISTER 0x0F
#define LSM_CTRL1_XL_REGISTER 0x10
#define LSM_CTRL2_G_REGISTER 11h
#define LSM_TRL3_C_REGISTER 0x12

void lsm_init();      // Configure PIC pins to communicate with LSM
void lsm_who_am_i();  // Read the who_am_i register from LSM

#endif
