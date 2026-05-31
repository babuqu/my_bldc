/**
 ****************************************************************************************************
 * @file        motor_cmd.h
 * @author      Motor Control Team
 * @version     V1.0
 * @date        2026-05-24
 * @brief       电机命令解析头文件
 ****************************************************************************************************
 */
#ifndef __MOTOR_CMD_H
#define __MOTOR_CMD_H

#include "main.h"

/******************************************************************************************/
/* 命令定义 */
#define CMD_START           0x01    /* 启动电机 */
#define CMD_STOP            0x02    /* 停止电机 */
#define CMD_SET_SPEED       0x03    /* 设置速度 */
#define CMD_SET_POS         0x04    /* 设置位置 */
#define CMD_EMERGENCY_STOP  0x05    /* 急停 */
#define CMD_CLEAR_FAULT     0x06    /* 清除故障 */
#define CMD_GET_STATE       0x07    /* 获取状态 */
#define CMD_GET_DATA        0x08    /* 获取数据 */
#define CMD_SET_PID         0x09    /* 设置 PID 参数 */
#define CMD_HELP            0x0A    /* 帮助 */

/******************************************************************************************/
/* 命令帧格式 */
/* [帧头 0xAA][命令字][数据长度][数据 N][校验和] */
#define CMD_FRAME_HEADER    0xAA
#define CMD_FRAME_MAX_LEN   32

/******************************************************************************************/
/* 命令解析函数 */
void MotorCmd_Init(void);
void MotorCmd_Process(uint8_t *buf, uint16_t len);
void MotorCmd_PrintHelp(void);

#endif /* __MOTOR_CMD_H */
