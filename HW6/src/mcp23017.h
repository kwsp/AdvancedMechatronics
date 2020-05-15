#ifndef MCP23017_H__
#define MCP23017_H__
// Header file for the MCP23017 IO expander
#include "i2c.h"

#define MCP_ADDR 0b01000000

#define MCP_IODIRA 0x00
#define MCP_IODIRB 0x01
#define MCP_OLATA 0x14
#define MCP_GPIOB 0x13

void mcp_write(unsigned char reg_addr, unsigned char data);
unsigned char mcp_read(unsigned char reg_addr);
void mcp_init(); // Configure MCP pins
void mcp_set_pin_A(unsigned char pin);
void mcp_clear_pin_A(unsigned char pin);
unsigned char mcp_read_pin_B();

#endif 
