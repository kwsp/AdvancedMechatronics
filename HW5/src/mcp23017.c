#include "mcp23017.h"

void mcp_write(unsigned char reg_addr, unsigned char data) {
    i2c_master_start();         // Send start bit
    i2c_master_send(MCP_WRITE_ADDR);// Send I2C address with write bit
    i2c_master_send(reg_addr);  // Send register to write to
    i2c_master_send(data);      // Send data to write
    i2c_master_stop();          // Send stop bit
}

unsigned char mcp_read(unsigned char reg_addr) {
    i2c_master_start();         // Send start bit
    i2c_master_send(MCP_WRITE_ADDR);// Send I2C address with write bit
    i2c_master_send(reg_addr);  // Send register to read from
    i2c_master_restart();       // Send restart bit
    i2c_master_send(MCP_READ_ADDR); // Send I2C address with read bit
    unsigned char ret = i2c_master_recv();      
    i2c_master_ack(1);
    i2c_master_stop();
    return ret;
}

void mcp_init() {
    // Set IOCON
    mcp_write(MCP_IODIRA, 0x00); // All A pins output
    mcp_write(MCP_IODIRB, 0xFF); // All B pins input
}

void mcp_set_pin_A(unsigned char pin) {
    mcp_write(MCP_OLATA, (0x01 << pin));
}

void mcp_clear_pin_A(unsigned char pin) {
    unsigned char curr_A = mcp_read(MCP_OLATA);
    mcp_write(MCP_OLATA, curr_A & ~(0x01 << pin));
}

unsigned char mcp_read_pin_B() {
    return mcp_read(MCP_GPIOB);
}
