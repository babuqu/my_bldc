// Modbus RTU 协议处理实现，包含寄存器映射、请求解析、响应组帧和 CRC 校验。
#include <stdio.h>       // 标准输入输出头文件；本文件当前没有直接调用其中的接口。
#include "usart.h"      // 提供 huart1 UART 句柄和 HAL_UART_Transmit_DMA 接口。
#include "modbus.h"     // 提供 Modbus 地址、寄存器数量和函数声明。
#include "pid.h"        // 提供速度 PID 参数与误差数据。
#include "bldc.h"       // 提供 BLDC 电机状态数据。
#include "zero_ctr.h"   // 提供过零控制相关声明。
#include "param.h"      // 提供参数发布和寄存器写入应用接口。
#include "string.h"     // 提供 memcpy 内存复制函数。
#include "adc.h"        // 提供电压、电流、温度和反电动势采样数据。

static UART_Frame_t ModbusTx; // 静态发送帧缓存，包含待发送数据 buf 和数据长度 len。

/* 发送状态为 BUSY 时不启动新的 DMA 发送；调用入口也会避免改写在用缓冲区。 */
static void Modbus_StartTransmit(void)
{
    if (huart1.gState == HAL_UART_STATE_READY)
    {
        (void)HAL_UART_Transmit_DMA(&huart1, ModbusTx.buf, ModbusTx.len);
    }
}

/* 参数提交或持久化失败时，按 Modbus 异常帧格式回复主站。 */
static void Modbus_SendException(uint8_t slave, uint8_t function, uint8_t exception) // 组帧并发送异常响应。
{
    uint16_t crc; // 保存异常响应的 CRC16 校验值。

    ModbusTx.buf[0] = slave; // 响应第 1 字节为从站地址。
    ModbusTx.buf[1] = (uint8_t)(function | 0x80u); // 将功能码最高位置 1，表示这是异常响应。
    ModbusTx.buf[2] = exception; // 响应第 3 字节为 Modbus 异常代码。
    crc = Modbus_CRC16(ModbusTx.buf, 3u); // 对地址、异常功能码和异常代码计算 CRC。
    ModbusTx.buf[3] = (uint8_t)crc; // 按 RTU 顺序先写 CRC 低字节。
    ModbusTx.buf[4] = (uint8_t)(crc >> 8); // 再写 CRC 高字节。
    ModbusTx.len = 5u; // 异常响应由 3 字节数据和 2 字节 CRC 组成。
    Modbus_StartTransmit(); // 通过 USART1 DMA 发送异常响应。
}

// 保持寄存器映射；寄存器下标对应 Modbus 请求中的零基地址偏移。
uint16_t HoldingReg[HOLDING_REG_NUM] = // 创建保持寄存器数组并用初始值填充。
{
    0,   // 下标 0：速度 PID 比例系数的初始镜像值。
    200, // 下标 1：速度 PID 积分系数的初始镜像值。
    300, // 下标 2：速度 PID 微分系数的初始镜像值。
    400, // 下标 3：速度设定值的初始镜像值。
    600, // 下标 4：原代码预留的积分项上限初始值。
    700, // 下标 5：原代码预留的积分项下限初始值。
    800, // 下标 6：原代码预留的 PID 输出上限初始值。
    900, // 下标 7：原代码预留的 PID 输出下限初始值。
    0    // 下标 8：预留项；数组总长度由 HOLDING_REG_NUM 决定。
    //在这里加可读写的数据
};

uint16_t InputReg[INPUT_REG_NUM]; // 输入寄存器数组，用于保存只读运行数据。

// 把当前电机和传感器数据更新到输入寄存器映射中。
void InputReg_Publish(void) // 发布实时输入寄存器数据。
{
    InputReg[0] = (int16_t)g_bldc_motor1.speed; // 下标 0：电机 1 实际转速，以 16 位补码保存。
    InputReg[1] = (int16_t)g_speed_pid.Error; // 下标 1：速度 PID 误差，以 16 位补码保存。
    InputReg[2] = (uint16_t)(10 * sensor_data.Vbus_v); // 下标 2：母线电压乘 10 后转为整数上报。
    InputReg[3] = (uint16_t)(10 * sensor_data.v12); // 下标 3：12 V 电源电压乘 10 后上报。
    InputReg[4] = (uint16_t)(10 * sensor_data.v5); // 下标 4：5 V 电源电压乘 10 后上报。
    InputReg[5] = sensor_data.i_w; // 下标 5：W 相电流采样值。
    InputReg[6] = sensor_data.i_u; // 下标 6：U 相电流采样值。
    InputReg[7] = sensor_data.i_v; // 下标 7：V 相电流采样值。
    InputReg[8] = sensor_data.temperature; // 下标 8：MCU 温度数据。
    InputReg[9] = sensor_data.temperature; // 下标 9：当前也取 temperature，实际 NTC 数据需确认来源。
    InputReg[10] = sensor_data.emf_w; // 下标 10：W 相反电动势采样值。
    InputReg[11] = sensor_data.emf_u; // 下标 11：U 相反电动势采样值。
    InputReg[12] = sensor_data.emf_v; // 下标 12：V 相反电动势采样值。
    //在这里加只读的数据
}

// 校验并分发一帧收到的 Modbus RTU 请求。
void Modbus_Parse(uint8_t *buf, uint16_t len) // buf 指向接收帧，len 为接收帧长度。
{
    uint16_t crc_calc; // 保存本地计算得到的 CRC。
    uint16_t crc_recv; // 保存请求帧中携带的 CRC。

    if (buf == NULL || len < 4u || len > sizeof(((UART_Frame_t *)0)->buf)) // 至少包含地址、功能码和 CRC，且不得超过接收缓冲区。
    {
        return; // 长度不足时直接丢弃。
    }

    crc_calc = Modbus_CRC16(buf, len - 2); // 对除末尾 CRC 两字节外的数据计算校验值。
    crc_recv = // 将帧尾两个字节组合为 16 位 CRC。
        buf[len - 2] // 末尾倒数第 2 字节是 CRC 低字节。
        |
        (buf[len - 1] << 8); // 末尾字节是 CRC 高字节，左移后与低字节合并。

    if(crc_calc != crc_recv) // 检查收到的 CRC 是否正确。
    {
        return; // 校验失败时不处理该帧。
    }

    if(buf[0] != MODBUS_SLAVE_ADDR) // 检查帧中的从站地址是否为本设备地址。
    {
        return; // 地址不匹配时忽略该请求。
    }

    /* DMA 尚未发送完时不处理新请求，避免覆盖 DMA 正在读取的响应缓冲区。 */
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    switch(buf[1]) // 根据功能码选择请求处理函数。
    {
        case 0x03: // 功能码 0x03：读保持寄存器。
            if (len != 8u)
            {
                Modbus_SendException(buf[0], buf[1], 0x03u); // 固定请求长度错误：非法数据值。
                break;
            }
            Modbus_Func03(buf); // 将整帧交给 0x03 处理函数。
            break; // 结束本分支，避免继续执行其他 case。
        case 0x06: // 功能码 0x06：写单个保持寄存器。
            if (len != 8u)
            {
                Modbus_SendException(buf[0], buf[1], 0x03u);
                break;
            }
            Modbus_Func06(buf); // 将整帧交给 0x06 处理函数。
            break; // 结束本分支。
        case 0x10: // 功能码 0x10：写多个保持寄存器。
            if (len < 9u)
            {
                break; // 字节计数字段不完整，无法安全构造异常响应。
            }
            if (len != (uint16_t)(9u + buf[6]))
            {
                Modbus_SendException(buf[0], buf[1], 0x03u); // 实际帧长与字节计数字段不一致。
                break;
            }
            Modbus_Func10(buf); // 将整帧交给 0x10 处理函数。
            break; // 结束本分支。
        case 0x04: // 功能码 0x04：读输入寄存器。
            if (len != 8u)
            {
                Modbus_SendException(buf[0], buf[1], 0x03u);
                break;
            }
            Modbus_Func04(buf); // 将整帧交给 0x04 处理函数。
            break; // 结束本分支。
        default: // 其他功能码当前没有处理逻辑。
            Modbus_SendException(buf[0], buf[1], 0x01u); // 返回非法功能码异常。
            break; // 忽略不支持的功能码。
    }
}

// 功能码 0x03：读取一段保持寄存器并返回寄存器数据。
void Modbus_Func03(uint8_t *rx) // rx 指向已经通过 Modbus_Parse 校验的请求帧。
{
    uint16_t addr; // 请求起始寄存器偏移。
    uint16_t num; // 请求读取的寄存器数量。
    uint16_t crc; // 响应帧 CRC。
    uint16_t i; // 遍历寄存器时使用的循环变量。

    Param_Publish(HoldingReg); // 将参数服务中的当前值同步到保持寄存器镜像。
    addr = // 从请求帧中解析 16 位起始地址。
        (rx[2] << 8) // 地址高字节位于 rx[2]。
        |
        rx[3]; // 地址低字节位于 rx[3]。
    num = // 从请求帧中解析 16 位寄存器数量。
        (rx[4] << 8) // 数量高字节位于 rx[4]。
        |
        rx[5]; // 数量低字节位于 rx[5]。

    if (num == 0u)
    {
        Modbus_SendException(MODBUS_SLAVE_ADDR, 0x03u, 0x03u); // 读取数量不能为零。
        return;
    }
    if (((uint32_t)addr + num) > HOLDING_REG_NUM) // 检查所请求的地址区间是否超过数组末尾。
    {
        Modbus_SendException(MODBUS_SLAVE_ADDR, 0x03u, 0x02u); // 请求地址超出保持寄存器范围。
        return;
    }

    ModbusTx.buf[0] = MODBUS_SLAVE_ADDR; // 响应第 1 字节：本机从站地址。
    ModbusTx.buf[1] = 0x03; // 响应第 2 字节：功能码 0x03。
    ModbusTx.buf[2] = num * 2; // 响应第 3 字节：数据区字节数，每个寄存器 2 字节。
    uint16_t index = 3; // 从响应第 3 个下标开始写寄存器数据。

    for(i = 0; i < num; i++) // 遍历请求范围中的每个寄存器。
    {
        ModbusTx.buf[index++] = // 写入当前寄存器的高字节并递增写入位置。
            HoldingReg[addr+i] >> 8; // 右移 8 位得到寄存器高字节。
        ModbusTx.buf[index++] = // 写入当前寄存器的低字节并递增写入位置。
            HoldingReg[addr+i] & 0xFF; // 用掩码取寄存器低 8 位。
    }

    crc = Modbus_CRC16( // 对响应帧当前已有部分计算 CRC。
        ModbusTx.buf, // CRC 输入从响应缓存首字节开始。
        index); // 输入长度为地址、功能码、字节数和寄存器数据。

    ModbusTx.buf[index++] = // 在数据区后追加 CRC 低字节。
        crc & 0xFF; // 提取 CRC 的低 8 位。

    ModbusTx.buf[index++] = // 继续追加 CRC 高字节。
        crc >> 8; // 右移后取得 CRC 高 8 位。

    ModbusTx.len = index; // 记录完整响应帧长度。

    Modbus_StartTransmit(); // 启动 UART DMA 发送响应帧。
}

// 功能码 0x06：写入一个保持寄存器，成功时回显请求内容。
void Modbus_Func06(uint8_t *rx) // rx 指向功能码 0x06 请求帧。
{
    uint16_t addr; // 请求写入的寄存器偏移。
    uint16_t value; // 请求写入的 16 位寄存器值。
    uint16_t crc; // 回显响应帧 CRC。
    uint8_t exception_code;

    addr = // 从请求帧解析寄存器地址。
        (rx[2] << 8) // 地址高字节。
        | rx[3]; // 地址低字节。

    value = // 从请求帧解析寄存器值。
        (rx[4] << 8) // 值的高字节。
        | rx[5]; // 值的低字节。

    if(addr >= HOLDING_REG_NUM) // 检查目标地址是否在保持寄存器数组范围内。
    {
        Modbus_SendException(rx[0], rx[1], 0x02u); // 返回非法数据地址。
        return;
    }

    HoldingReg[addr] = value; // 先把收到的值写入保持寄存器镜像。

    if (!Param_ApplyEx(addr, 1u, &exception_code)) // 将本次写入的一个寄存器交给参数服务校验并应用。
    {
        Modbus_SendException(rx[0], rx[1], exception_code); // 按参数服务给出的原因返回异常。
        return; // 已发送异常响应，不再发送正常回显。
    }

    memcpy(ModbusTx.buf,rx,6); // 复制请求帧的地址、功能码、寄存器地址和值作为响应主体。

    crc = Modbus_CRC16(ModbusTx.buf,6); // 对回显主体的 6 个字节计算 CRC。

    ModbusTx.buf[6] = crc & 0xFF; // 响应第 7 字节写 CRC 低字节。
    ModbusTx.buf[7] = crc >> 8; // 响应第 8 字节写 CRC 高字节。

    ModbusTx.len = 8; // 功能码 0x06 的正常响应固定为 8 字节。

    Modbus_StartTransmit(); // 启动 UART DMA 发送正常响应。
}

// 功能码 0x10：解析并写入一段连续保持寄存器。
void Modbus_Func10(uint8_t *rx) // rx 指向功能码 0x10 请求帧。
{
		uint16_t addr; // 批量写入的起始寄存器偏移。
		uint16_t num; // 请求写入的寄存器数量。
		uint8_t byte_cnt; // 请求帧声明的数据区字节数。

		uint16_t value; // 暂存当前解析出来的寄存器值。
		uint16_t crc; // 正常响应帧 CRC。
		uint8_t exception_code;

		uint16_t i; // 遍历寄存器数据的循环变量。

		addr = // 从请求帧解析起始地址。
				((uint16_t)rx[2] << 8) // 起始地址高字节。
				| rx[3]; // 起始地址低字节。

		num = // 从请求帧解析寄存器数量。
				((uint16_t)rx[4] << 8) // 寄存器数量高字节。
				| rx[5]; // 寄存器数量低字节。

		byte_cnt = rx[6]; // 读取请求帧中的数据字节数声明。

		if (num == 0u)
		{
				Modbus_SendException(rx[0], rx[1], 0x03u); // 写入数量不能为零。
				return;
		}

		if(byte_cnt != num * 2) // 每个寄存器为 16 位，因此数据字节数应等于数量乘 2。
		{
				Modbus_SendException(rx[0], rx[1], 0x03u); // 返回非法数据值。
				return;
		}

		if (((uint32_t)addr + num) > HOLDING_REG_NUM) // 检查本次连续写入的地址范围是否越界。
		{
				Modbus_SendException(rx[0], rx[1], 0x02u); // 返回非法数据地址。
				return;
		}

		for(i = 0; i < num; i++) // 逐个解析并保存请求中的寄存器值。
		{
				value = // 组合当前寄存器的高、低字节。
						((uint16_t)rx[7 + i * 2] << 8) // 当前寄存器数据高字节位于数据区对应位置。
						| rx[8 + i * 2]; // 紧随其后的字节为当前寄存器数据低字节。

				HoldingReg[addr + i] = value; // 写入对应下标的保持寄存器镜像。
		}

    if (!Param_ApplyEx(addr, num, &exception_code)) // 将本次写入范围交给参数服务统一校验和应用。
    {
        Modbus_SendException(rx[0], rx[1], exception_code); // 按参数服务给出的原因返回异常。
        return; // 已发送异常响应，不再构造正常响应。
    }

		ModbusTx.buf[0] = MODBUS_SLAVE_ADDR; // 响应第 1 字节：本机从站地址。
		ModbusTx.buf[1] = 0x10; // 响应第 2 字节：功能码 0x10。

		ModbusTx.buf[2] = rx[2]; // 回显起始地址高字节。
		ModbusTx.buf[3] = rx[3]; // 回显起始地址低字节。

		ModbusTx.buf[4] = rx[4]; // 回显写入寄存器数量高字节。
		ModbusTx.buf[5] = rx[5]; // 回显写入寄存器数量低字节。

		crc = Modbus_CRC16(ModbusTx.buf, 6); // 对响应前 6 字节计算 CRC。

		ModbusTx.buf[6] = crc & 0xFF; // 响应第 7 字节为 CRC 低字节。
		ModbusTx.buf[7] = crc >> 8; // 响应第 8 字节为 CRC 高字节。

		ModbusTx.len = 8; // 功能码 0x10 的正常响应固定为 8 字节。

		Modbus_StartTransmit(); // 启动 UART DMA 发送写入确认响应。

}

// 功能码 0x04：读取一段只读输入寄存器并返回寄存器数据。
void Modbus_Func04(uint8_t *rx) // rx 指向功能码 0x04 请求帧。
{
    uint16_t addr; // 请求读取的起始输入寄存器偏移。
    uint16_t num; // 请求读取的输入寄存器数量。
    uint16_t crc; // 响应帧 CRC。
    uint16_t i; // 遍历输入寄存器时使用的循环变量。

    InputReg_Publish(); // 读取前刷新输入寄存器中的实时数据。

    addr = // 从请求帧解析输入寄存器起始地址。
        ((uint16_t)rx[2] << 8) // 地址高字节。
        | rx[3]; // 地址低字节。

    num = // 从请求帧解析读取数量。
        ((uint16_t)rx[4] << 8) // 数量高字节。
        | rx[5]; // 数量低字节。

    if (num == 0u)
    {
        Modbus_SendException(MODBUS_SLAVE_ADDR, 0x04u, 0x03u); // 读取数量不能为零。
        return;
    }
    if (((uint32_t)addr + num) > INPUT_REG_NUM) // 检查读取区间是否超过输入寄存器数组边界。
    {
        Modbus_SendException(MODBUS_SLAVE_ADDR, 0x04u, 0x02u); // 请求地址超出输入寄存器范围。
        return;
    }

    ModbusTx.buf[0] = MODBUS_SLAVE_ADDR; // 响应第 1 字节：本机从站地址。
    ModbusTx.buf[1] = 0x04; // 响应第 2 字节：功能码 0x04。
    ModbusTx.buf[2] = num * 2; // 响应第 3 字节：数据区长度，每个寄存器占 2 字节。

    uint16_t index = 3; // 从第 3 个下标开始写入输入寄存器数据。

    for(i = 0; i < num; i++) // 遍历请求范围内的输入寄存器。
    {
        ModbusTx.buf[index++] = // 写入当前输入寄存器的高字节。
            InputReg[addr+i] >> 8; // 右移 8 位取得高字节。

        ModbusTx.buf[index++] = // 写入当前输入寄存器的低字节。
            InputReg[addr+i] & 0xFF; // 掩码保留低 8 位。
    }

    crc = Modbus_CRC16(ModbusTx.buf,index); // 对响应当前已有数据计算 CRC。

    ModbusTx.buf[index++] = crc & 0xFF; // 追加 CRC 低字节。
    ModbusTx.buf[index++] = crc >> 8; // 追加 CRC 高字节。

    ModbusTx.len = index; // 保存响应帧总长度。

    Modbus_StartTransmit(); // 启动 UART DMA 发送输入寄存器响应。
}

// 计算 Modbus RTU 使用的 CRC-16，返回值发送时低字节在前、高字节在后。
uint16_t Modbus_CRC16(uint8_t *buf, uint16_t len) // buf 为参与校验的数据，len 为数据长度。
{
    uint16_t crc = 0xFFFF; // CRC16 的初始值为 0xFFFF。

    for(uint16_t pos = 0; pos < len; pos++) // 逐字节处理输入缓冲区。
    {
        crc ^= (uint16_t)buf[pos]; // 将当前数据字节异或到 CRC 寄存器低位。

        for(uint8_t i = 0; i < 8; i++) // 每个数据字节执行 8 次逐位运算。
        {
            if(crc & 0x0001) // 检查当前 CRC 的最低位是否为 1。
            {
                crc >>= 1; // CRC 右移一位。
                crc ^= 0xA001; // 最低位原为 1 时异或 Modbus 反转多项式 0xA001。
            }
            else // 当前 CRC 最低位为 0 时不需要应用多项式。
            {
                crc >>= 1; // 仅将 CRC 右移一位。
            }
        }
    }

    return crc; // 返回计算完成的 16 位 CRC 值。
}
