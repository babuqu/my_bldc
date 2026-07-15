//参数存储
//参数同步
//参数校验
//g_param 是参数源
//HoldingReg 是通讯镜像
//g_speed_pid 是运行镜像

//1 参数源（唯一真值）param_t g_param;

//2 参数映射表  param_map_t g_map[]
//   参数变量
//      ↓
//   寄存器地址
//      ↓
//   放缩关系

//3 参数发布
//把内部参数同步到 Modbus 寄存器。

//4 参数读取
//Param_UpdateFromModbus(HoldingReg);

//5 参数校验
//Param_Sanitize();防止非法参数。

//6 参数下发到控制器
//Param_To_System(&g_param);

//7 上电初始化
//Flash读取
//    ↓
//检查标志位
//    ↓
//参数校验
//    ↓
//默认参数补充
//    ↓
//更新PID
//    ↓
//发布寄存器
#include "param.h"
#include <stdio.h>
#include "pid.h"
#include "modbus.h"


#define PARAM_FLAG_DEFAULT 0xA5A5

extern PID_TypeDef g_speed_pid;

param_t g_param;

/* =========================
   参数映射表（核心）
   ========================= */
static param_map_t g_map[] =
{
	  //参数-寄存器地址-放缩系数
    {&g_param.set_speed,0,1.0f,PARAM_S16},

    {&g_param.kp,1,100000.0f,PARAM_FLOAT},
    {&g_param.ki,2,100000.0f,PARAM_FLOAT},
    {&g_param.kd,3,100000.0f,PARAM_FLOAT},

    {&g_param.i_max,4,1.0f,PARAM_S16},
    {&g_param.i_min,5,1.0f,PARAM_S16},

    {&g_param.out_max,6,1.0f,PARAM_S16},
    {&g_param.out_min,7,1.0f,PARAM_S16},
		//{&g_param.flag,4, 1.0f},
};

//参数合理性检查
uint8_t Param_Sanitize(param_t *p)
{
    uint8_t ret = 0;
    if (!p) return 0;

    if (p->kp < 0) {p->kp = 0; ret = 1;}//这个地方超出范围后，应该使用默认值
    if (p->kp > 6.55) {p->kp = 6.55; ret = 1;}

    if (p->ki < 0) {p->ki = 0; ret = 1;}
    if (p->ki > 6.55) {p->ki = 6.55; ret = 1;}

    if (p->kd < 0) {p->kd = 0; ret = 1;}
    if (p->kd > 6.55) {p->kd = 6.55; ret = 1;}

    if (p->i_max <= p->i_min)
    {
        p->i_max = p->i_min + 1;
        ret = 1;
    }

    if (p->out_max <= p->out_min)
    {
        p->out_max = p->out_min + 1;
        ret = 1;
    }
		
    return ret;
}



#define MAP_SIZE (sizeof(g_map)/sizeof(g_map[0]))
//初始参数为系统使用参数
void Param_InitDefault()
{
  

    g_param.kp = KP;
    g_param.ki = KI;
    g_param.kd = KD;

    g_param.set_speed = 0;

    g_param.i_max = 8000;
    g_param.i_min = -8000;
    g_param.out_max = 8000;
    g_param.out_min = -8000;
  	g_param.flag = PARAM_FLAG_DEFAULT;
}


//更新当前系统使用参数
void Param_To_System(param_t *p)
{
    if (!p) return;

    g_speed_pid.Proportion = p->kp;
    g_speed_pid.Integral   = p->ki;
    g_speed_pid.Derivative = p->kd;

    g_speed_pid.SetPoint = p->set_speed;

    g_speed_pid.IngMax = p->i_max;
    g_speed_pid.IngMin = p->i_min;
    g_speed_pid.OutMax = p->out_max;
    g_speed_pid.OutMin = p->out_min;
}




//参数存储
uint8_t Param_SaveToFlash(param_t *p)
{
    uint32_t crc;

//    crc = CRC32((uint8_t*)p, sizeof(param_t));

//    flash_erase(PARAM_ADDR);
//    flash_write(PARAM_ADDR, (uint8_t*)p, sizeof(param_t));
//    flash_write(PARAM_ADDR + sizeof(param_t), (uint8_t*)&crc, 4);

    return 0;
}
//参数读取
uint8_t Param_LoadFromFlash(param_t *p)
{
    uint32_t crc_flash, crc_calc;

//    flash_read(PARAM_ADDR, (uint8_t*)p, sizeof(param_t));
//    flash_read(PARAM_ADDR + sizeof(param_t), (uint8_t*)&crc_flash, 4);

//    crc_calc = CRC32((uint8_t*)p, sizeof(param_t));

//    if (crc_calc != crc_flash)
//        return -1;

    return 0;
}

void Param_FromModbus(uint16_t *reg)
{
    int i;

    for(i = 0; i < MAP_SIZE; i++)
    {
        switch(g_map[i].type)//数据类型映射
        {
            case PARAM_FLOAT:
            {
                float *p = (float*)g_map[i].addr;

                *p =
                    (float)reg[g_map[i].reg]
                    / g_map[i].scale;
            }
            break;

            case PARAM_S16:
            {
                float *p = (float*)g_map[i].addr;

                *p =
                    (float)((int16_t)
                    reg[g_map[i].reg]);
            }
            break;

            case PARAM_U16:
            {
                float *p = (float*)g_map[i].addr;

                *p =
                    (float)reg[g_map[i].reg];
            }
            break;
        }
    }
}

void Param_ToModbus(uint16_t *reg)
{
    int i;

    for(i = 0; i < MAP_SIZE; i++)
    {
        switch(g_map[i].type)//数据类型映射
        {
            case PARAM_FLOAT:
            {
                float *p = (float*)g_map[i].addr;

                reg[g_map[i].reg] =
                    (uint16_t)
                    (*p * g_map[i].scale);
            }
            break;

            case PARAM_S16:
            {
                float *p = (float*)g_map[i].addr;

                reg[g_map[i].reg] =
                    (uint16_t)
                    ((int16_t)(*p));
            }
            break;

            case PARAM_U16:
            {
                float *p = (float*)g_map[i].addr;

                reg[g_map[i].reg] =
                    (uint16_t)(*p);
            }
            break;
        }
    }
}

//更新modbus数据到参数源中
uint8_t Param_UpdateFromModbus(uint16_t *HoldingReg)
{
    if (!HoldingReg)
        return 0;

    Param_FromModbus(HoldingReg);

    if(Param_Sanitize(&g_param))
    {
        Param_Publish(HoldingReg);
    }

    return 1;
}
//更新参数源中数据到modbus数据
void Param_Publish(uint16_t *HoldingReg)
{
    Param_ToModbus(HoldingReg);
}

uint8_t Param_Init_(void)//上电参数初始化函数
{
    uint8_t ret = 0;

    if(Param_LoadFromFlash(&g_param) != 0)
    {//如果参数读取失败
        Param_InitDefault();
        ret = 1;
    }
    else
    {//如果参数读取成功
        if(g_param.flag != PARAM_FLAG_DEFAULT)
        {//标志位检查错误
            Param_InitDefault();
            ret = 1;
        }
        else
        {//进行合理化判断
            ret = Param_Sanitize(&g_param);
        }
    }

    Param_To_System(&g_param);//将g_param参数更新到系统

    Param_Publish(HoldingReg);//参数发布

    return ret;
}

void Param_Apply(void)//数据同步
{
		Param_UpdateFromModbus(HoldingReg);

		Param_To_System(&g_param);

		Param_Publish(HoldingReg);

		//Param_SaveToFlash(&g_param);

}