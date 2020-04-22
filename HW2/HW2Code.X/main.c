#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <math.h>

#include"spi.h" // SPI interface

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

void writeDAC_(unsigned char channel, unsigned int n) {
    // Inputs: 
    //      Channel: 0 for DACA, 1 for DACB
    //      n range: 0-4096
    //
    // Control bits:
    // bit 15 = 0 : Channel select
    // bit 14 = 1 : Vref buffered
    // bit 13 = 1 : 1x output gain
    // bit 12 = 1 : Active mode operation, Vout available
    unsigned char b1 = 0b01110000;

    // Set the channel (A or B)
    b1 |= (0b1 & channel) << 7;

    // Get the upper 8 bits of the integer n
    b1 |= (n >> 8) & 0b11111111;

    // Cast the input int into an unsigned char
    // to only keep the least significant 8 bits
    unsigned char b2 = (unsigned char) n;

    // Write 2 byte over SPI
    // Bring CS low for two bytes as the 
    // DAC's write command is 16 bits
    LATAbits.LATA0 = 0; // Bring CS low
    spi_io(b1);
    spi_io(b2);
    LATAbits.LATA0 = 1; // Bring CS high
}


void writeDAC_A(unsigned int n) {
    writeDAC_(0, n);
}

void writeDAC_B(unsigned int n) {
    writeDAC_(1, n);
}

void sleep_ms(double ms) {
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
    
    // Setup SPI1
    initSPI();

    __builtin_enable_interrupts();
    
    unsigned int i = 0;
    unsigned int flag = 0;
    unsigned int sine_wave = 0;
    unsigned int triangle_wave = 0;
    
    while (1) {
        // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
        // remember the core timer runs at half the sysclk
        
        i++;
        // Generate sine wave between 0 - 2048
        sine_wave = 2048 * sin(6.2832 * 0.004 * i) + 2048;
        writeDAC_A(sine_wave);
        
        sleep_ms(1); // 500 Hz for 2 2ms sleeps
                
        // Triangle wave
        if (flag == 0) {
            triangle_wave += 4;
        } else {
            triangle_wave -= 4;
        }
        
        if (triangle_wave >= 1000) flag = 1;
        else if (triangle_wave == 0) flag = 0;
        
        writeDAC_B(4.096 * triangle_wave);

        sleep_ms(1);

    }
}

