#ifndef __MODBUS_H
#define __MODBUS_H

#include <stdint.h>
#define HOLDING_REG_NUM 10
#define MODBUS_SLAVE_ADDR    0x01
#define INPUT_REG_NUM 15

extern uint16_t InputReg[INPUT_REG_NUM];
extern uint16_t HoldingReg[HOLDING_REG_NUM];
uint16_t Modbus_CRC16(uint8_t *buf, uint16_t len);

void Modbus_Parse(uint8_t *buf, uint16_t len);
void Modbus_Func03(uint8_t *rx);
void Modbus_Func06(uint8_t *rx);
void Modbus_Func10(uint8_t *rx);
void Modbus_Func04(uint8_t *rx);
#endif
