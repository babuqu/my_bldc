//modbus 协议实现
#include <stdio.h> 
#include "usart.h"
#include "modbus.h"
#include "pid.h"
#include "bldc.h"
#include "zero_ctr.h"
#include "param.h"
#include "string.h"
#include "adc.h"

static UART_Frame_t ModbusTx;

uint16_t HoldingReg[HOLDING_REG_NUM] =
{
    0,  //g_speed_pid.Proportion   /* 比例常数 Proportional Const*/ 
    200,//g_speed_pid.Integral      /* 积分常数 Integral Const*/
    300,//g_speed_pid.Derivative    /* 微分常数 Derivative Const*/ 
    400,//g_speed_pid.SetPoint      /* 设定目标Desired Value*/
    600,//g_speed_pid.IngMax = 8000;积分相最大限幅
    700,//g_speed_pid.IngMin = 8000;积分相最小限幅
    800,//g_speed_pid.OutMax = 8000;PID输出最大限幅
    900,//g_speed_pid.OutMin = 8000;PID输出最小限幅
    0
};
uint16_t InputReg[INPUT_REG_NUM];

void InputReg_Publish(void)
{
    InputReg[0] = (int16_t)g_bldc_motor1.speed;//实际转速

    InputReg[1] = (int16_t)g_speed_pid.Error;//转速误差

    InputReg[2] = (uint16_t)(10*sensor_data.Vbus_v);//母线电压
    InputReg[3] = (uint16_t)(10*sensor_data.v12);//12V电压
    InputReg[4] = (uint16_t)(10*sensor_data.v5);//5V电压

    InputReg[5] = sensor_data.i_w;//w相电流
	  InputReg[6] = sensor_data.i_u;//u相电流
	  InputReg[7] = sensor_data.i_v;//v相电流
	
	  InputReg[8] = sensor_data.temperature;//mcu内部温度
	
	  InputReg[9] = sensor_data.temperature;//ntc温度
	
	  InputReg[10] = sensor_data.emf_w;//w相反向电动势
	  InputReg[11] = sensor_data.emf_u;//u相反向电动势
	  InputReg[12] = sensor_data.emf_v;//v相反向电动势	
	
}
	 
void Modbus_Parse(uint8_t *buf, uint16_t len)
{
    uint16_t crc_calc;
    uint16_t crc_recv;

    if(len < 8)
    {
        return;
    }

    crc_calc = Modbus_CRC16(buf, len - 2);

    crc_recv =
        buf[len - 2]
        |
        (buf[len - 1] << 8);

    if(crc_calc != crc_recv)
    {
        return;
    }

    if(buf[0] != MODBUS_SLAVE_ADDR)
    {
        return;
    }

    switch(buf[1])
    {
        case 0x03:
            Modbus_Func03(buf);//读保持寄存器
            break;
				case 0x06:
						Modbus_Func06(buf);//写单个寄存器
						break;
				case 0x10: 
					  Modbus_Func10(buf); //写多个寄存器
				    break;
				case 0x04:
					  Modbus_Func04(buf);//只读寄存器
				    break;
        default:
            break;
    }
}

void Modbus_Func03(uint8_t *rx)
{
    uint16_t addr;
    uint16_t num;
    uint16_t crc;
    uint16_t i;
    Param_Publish(HoldingReg);//参数同步
    addr =
        (rx[2] << 8)
        |
        rx[3];

    num =
        (rx[4] << 8)
        |
        rx[5];

    if((addr + num) > HOLDING_REG_NUM)
    {
        return;
    }

    ModbusTx.buf[0] = MODBUS_SLAVE_ADDR;
    ModbusTx.buf[1] = 0x03;
    ModbusTx.buf[2] = num * 2;

    uint16_t index = 3;

    for(i = 0; i < num; i++)
    {
        ModbusTx.buf[index++] =
            HoldingReg[addr+i] >> 8;

        ModbusTx.buf[index++] =
            HoldingReg[addr+i] & 0xFF;
    }

    crc = Modbus_CRC16(
        ModbusTx.buf,
        index);

    ModbusTx.buf[index++] =
        crc & 0xFF;

    ModbusTx.buf[index++] =
        crc >> 8;

    ModbusTx.len = index;

    HAL_UART_Transmit_DMA(
        &huart1,
        ModbusTx.buf,
        ModbusTx.len);
}
void Modbus_Func06(uint8_t *rx)
{
    uint16_t addr;
    uint16_t value;
    uint16_t crc;

    addr =
        (rx[2] << 8)
        | rx[3];

    value =
        (rx[4] << 8)
        | rx[5];

    if(addr >= HOLDING_REG_NUM)
    {
        return;
    }

    HoldingReg[addr] = value;

    //同步到实际变量
    Param_Apply();
    //06响应直接回显请求帧
    memcpy(ModbusTx.buf,rx,6);

    crc = Modbus_CRC16(ModbusTx.buf,6);

    ModbusTx.buf[6] = crc & 0xFF;
    ModbusTx.buf[7] = crc >> 8;

    ModbusTx.len = 8;

    HAL_UART_Transmit_DMA(
        &huart1,
        ModbusTx.buf,
        ModbusTx.len);
}

void Modbus_Func10(uint8_t *rx)
{
		uint16_t addr;
		uint16_t num;
		uint8_t byte_cnt;

		uint16_t value;
		uint16_t crc;

		uint16_t i;

		addr =
				((uint16_t)rx[2] << 8)
				| rx[3];

		num =
				((uint16_t)rx[4] << 8)
				| rx[5];

		byte_cnt = rx[6];

		if(byte_cnt != num * 2)
		{
				return;
		}

		if((addr + num) > HOLDING_REG_NUM)
		{
				return;
		}

		for(i = 0; i < num; i++)
		{
				value =
						((uint16_t)rx[7 + i * 2] << 8)
						| rx[8 + i * 2];

				HoldingReg[addr + i] = value;
		}

    Param_Apply();

		ModbusTx.buf[0] = MODBUS_SLAVE_ADDR;
		ModbusTx.buf[1] = 0x10;

		ModbusTx.buf[2] = rx[2];
		ModbusTx.buf[3] = rx[3];

		ModbusTx.buf[4] = rx[4];
		ModbusTx.buf[5] = rx[5];

		crc = Modbus_CRC16(ModbusTx.buf, 6);

		ModbusTx.buf[6] = crc & 0xFF;
		ModbusTx.buf[7] = crc >> 8;

		ModbusTx.len = 8;

		HAL_UART_Transmit_DMA(
				&huart1,
				ModbusTx.buf,
				ModbusTx.len);

}
void Modbus_Func04(uint8_t *rx)
{
    uint16_t addr;
    uint16_t num;
    uint16_t crc;
    uint16_t i;

    InputReg_Publish();

    addr =
        ((uint16_t)rx[2] << 8)
        | rx[3];

    num =
        ((uint16_t)rx[4] << 8)
        | rx[5];

    if((addr + num) > INPUT_REG_NUM)
    {
        return;
    }

    ModbusTx.buf[0] = MODBUS_SLAVE_ADDR;
    ModbusTx.buf[1] = 0x04;
    ModbusTx.buf[2] = num * 2;

    uint16_t index = 3;

    for(i = 0; i < num; i++)
    {
        ModbusTx.buf[index++] =
            InputReg[addr+i] >> 8;

        ModbusTx.buf[index++] =
            InputReg[addr+i] & 0xFF;
    }

    crc = Modbus_CRC16(ModbusTx.buf,index);

    ModbusTx.buf[index++] = crc & 0xFF;
    ModbusTx.buf[index++] = crc >> 8;

    ModbusTx.len = index;

    HAL_UART_Transmit_DMA(
        &huart1,
        ModbusTx.buf,
        ModbusTx.len);
}
uint16_t Modbus_CRC16(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for(uint16_t pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos];

        for(uint8_t i = 0; i < 8; i++)
        {
            if(crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}


