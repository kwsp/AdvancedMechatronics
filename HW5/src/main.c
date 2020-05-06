#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <math.h>
#include <stdio.h>

/*#include "i2c.h" // I2C interface*/
/*#include "mcp23017.h" // MCP23017 IO expander*/
/*#include "ssd1306.h"  // SSD1306 OLED display */
#include <ws2812b.h> // 3 color LED driver

// DEVCFG0
#pragma config DEBUG = OFF // disable debugging
#pragma config JTAGEN = OFF // disable jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // disable flash write protect
#pragma config BWP = OFF // disable boot write protect
#pragma config CP = OFF // disable code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // disable secondary oscillator
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


void sleep_ms(double ms) {
    // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
    // remember the core timer runs at half the sysclk
    _CP0_SET_COUNT(0);
    while (_CP0_GET_COUNT() < CLOCK_RATE / 2000 * ms) {;}
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
    /*i2c_master_setup();*/
    /*mcp_init();*/
    /*ssd1306_setup();*/
    ws2812b_setup();
    
    __builtin_enable_interrupts();
    
    unsigned int counter = 0;
    unsigned int clock = 0;
    char message[50];
    
    while (1) {
        _CP0_SET_COUNT(0); // Set timer to zero
        
//        unsigned char b_input = mcp_read_pin_B();
//        if (b_input & 0x01) {
//            mcp_clear_pin_A(7);
//        } else {
//            mcp_set_pin_A(7);
//        }
        
        double fps = (double) 24000000 / clock;
        
        // Draw message on the SSD1306 OLED
        ssd1306_clear();
        sprintf(message, "FPS = %f", fps);
        ssd1306_drawMessage(10, 10, message);
        
        sprintf(message, "Counter = %d", counter);
        ssd1306_drawMessage(10, 20, message);
        
        ssd1306_update();
        counter++;

        clock = _CP0_GET_COUNT();
        
        // Flip UserLED for heart beat
        LATAbits.LATA4 = !LATAbits.LATA4; 
        sleep_ms(200);
    }
}

