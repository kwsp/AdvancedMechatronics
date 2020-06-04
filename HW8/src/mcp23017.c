#include "mcp23017.h"

void mcp_write(unsigned char reg_addr, unsigned char data) {
    i2c_write(MCP_ADDR, reg_addr, data);
}

unsigned char mcp_read(unsigned char reg_addr) {
    return i2c_read(MCP_ADDR, reg_addr);
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
