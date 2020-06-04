    #include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <math.h>
#include <stdio.h>

#include "i2c.h" // I2C interface
#include "mcp23017.h" // MCP23017 IO expander
#include "ssd1306.h"  // SSD1306 OLED display
#include "lsm6ds33.h" // LSM6DS33 IMU
#include "ws2812b.h"  // WS2812 3 Color LED driver
#include "i2c.h" // I2C functionality builtin
#include "rtcc.h" // Real time clock
/*#include "cap_touch.h" // Capacitive touch*/

// DEVCFG0
#pragma config DEBUG = OFF // disable debugging
#pragma config JTAGEN = OFF // disable jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // disable flash write protect
#pragma config BWP = OFF // disable boot write protect
#pragma config CP = OFF // disable code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = ON // enable secondary oscillator
#pragma config IESO = OFF // disable switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable clock output
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // disable clock switch and FSCM
#pragma config WDTPS = PS1048576 // use largest wdt
#pragma config WINDIS = OFF // use non-window mode wdt
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz

#define CLOCK_RATE 48000000

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations

// A constant that converts time in milliseconds to 
// the number of clock ticks
const uint32_t SLEEP_CONVERSION = CLOCK_RATE / 2000;

/*
 * Sleep for ms milliseconds
 * Does not reset CP0 clock and assumes the clock is reset in the 
 * main loop to avoid overflow.
 */
void sleep_ms(double ms) {
    uint32_t target_t = _CP0_GET_COUNT() + SLEEP_CONVERSION * ms;
    while (_CP0_GET_COUNT() < target_t) {;}
}

int main() {

    __builtin_disable_interrupts(); // disable interrupts while initializing things
    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;
    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;
    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0; // A4 output: User LED
    TRISBbits.TRISB4 = 1; // B4 input: User button
    LATAbits.LATA4 = 0;   // Set A4 to low

    // Setup I2C
    i2c_master_setup();  // Setup I2C
    mcp_init();          // Setup MCP23017 IO expander
    ssd1306_setup();     // Setup ssd1306 OLED display
    lsm_init();          // Setup lsm6ds33 IMU

    ws2812b_setup(); // Init 3 color LEDs

    // Setup ADC and CTMU
    adc_setup();
    ctmu_setup();

    // Setup Cap sensing
    /*cap_calibrate();*/

    // Hard code reset time:
    //   time format: 0xHrMnSe00
    //   date format: 0xYrMnDyWk
    rtcc_setup(0x13000000, 0x20060404);

    __builtin_enable_interrupts();

    uint32_t counter = 0;
    uint32_t clock = 0;
    char message[50];

    // IMU sensor data 
    lsm_data sensor_data;
    // Acceleration map 
    int16_t x_len, y_len;
    
    // Real time clock
    rtccTime time;

    while (1) {
        _CP0_SET_COUNT(0); // Set timer to zero
        /*sleep_ms(40); // Sleep for 40 ms, 25 frames per second*/

        uint8_t b_input = mcp_read_pin_B();
        if (b_input & 0x01) {
            mcp_clear_pin_A(7);
        } else {
            mcp_set_pin_A(7);
        }

        double fps = (double) 24000000 / clock;

        // Draw message on the SSD1306 OLED
        ssd1306_clear();
        sprintf(message, "FPS = %f", fps);
        ssd1306_drawMessage(0, 0, message);

        /*
         * Read IMU and plot acc map
         */
        /*lsm_read_sensors(&sensor_data); // Read IMU*/
        // Calculate acceleration bar length in Cartesian coordinates
        /*y_len = 0.002 * sensor_data.data.acc_x;*/
        /*x_len = - 0.002 * sensor_data.data.acc_y;*/
        /*ssd1306_drawAccMap(x_len, y_len);*/
        
        /*
         * Read real time clock
         */
        time = readRTCC();
        sprintf(message, "Time: %u%u:%u%u:%u%u", time.hr10, time.hr01, time.min10, time.min01, time.sec10, time.sec01);
        ssd1306_drawMessage(0, 10, message);
        
        sprintf(message, "Date: 20%u%u/%u%u/%u%u", time.yr10, time.yr01, time.mn10, time.mn01, time.dy10, time.dy01);
        ssd1306_drawMessage(0, 20, message);

        /*
         * Flush screen buffer to device
         */
        ssd1306_update();
        counter++;

        clock = _CP0_GET_COUNT();

        // Flip UserLED for heart beat
        /*LATAbits.LATA4 = !LATAbits.LATA4;*/
        /*sleep_ms(200);*/
    }
}

