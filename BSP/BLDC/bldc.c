/**
 ****************************************************************************************************
 * @file        bldc.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       BLDC 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211014
 * 第一次发布
 *
 ****************************************************************************************************
 */
 
#include "bldc.h"
#include "adc.h"
#include "math.h"
#include "tim.h"

_bldc_obj g_bldc_motor1 = {STOP,0,0,CCW,0,0,0,0,0,0};/*电机结构体*/
/**
 * @brief       无刷电机初始化，包括定时器，霍尔接口以及SD引脚初始化
 * @param       arr: 自动重装值
 * @param       psc: 时钟预分频数
 * @retval      无
 */


/**
 * @brief       BLDC控制函数
 * @param       dir :电机方向, Duty:PWM占空比
 * @retval      无
 */
void bldc_ctrl(uint8_t motor_id,int32_t dir,float duty)
{
    if(motor_id == MOTOR_1)
    {
        g_bldc_motor1.dir = dir;            /* 方向 */
        g_bldc_motor1.pwm_duty = duty;      /* 占空比 */
    }
}
/************************************* 霍尔&反电动势接口初始化 *************************************/

/**
  * @brief  霍尔&反电动势接口初始化
  * @param  无
  * @retval 无
  */
void hall_gpio_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    HALL1_U_GPIO_CLK_ENABLE();
    HALL1_V_GPIO_CLK_ENABLE();
    HALL1_W_GPIO_CLK_ENABLE();

    /* 霍尔通道 1 引脚初始化 */
    gpio_init_struct.Pin = HALL1_TIM_CH1_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(HALL1_TIM_CH1_GPIO, &gpio_init_struct);

    /* 霍尔通道 2 引脚初始化 */
    gpio_init_struct.Pin = HALL1_TIM_CH2_PIN;
    HAL_GPIO_Init(HALL1_TIM_CH2_GPIO, &gpio_init_struct);

    /* 霍尔通道 3 引脚初始化 */
    gpio_init_struct.Pin = HALL1_TIM_CH3_PIN;
    HAL_GPIO_Init(HALL1_TIM_CH3_GPIO, &gpio_init_struct);

}

/************************************* BLDC相关函数 *************************************/
/* 关闭电机运转 */
extern TIM_HandleTypeDef htim1;
void stop_motor1(void)
{
    /* 关闭半桥芯片输出 */
    SHUTDOWN_OFF;
    /* 关闭PWM输出 */
    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
    /* 上下桥臂全部关断 */
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR3 = 0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_RESET);
}

/* 开启电机运转 */
void start_motor1(void)
{
    SHUTDOWN_EN;
    /* 使能PWM输出 */
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
}

/*****************************************************************************************/
/* 无感过零信号对应的绕组通道情况 */
pctr pfunclist_m1[6] =
{
    /*按过零信号（513264）换向 顺序如下*/
    /*逆时针换相顺序：(U+V-)->(U+W-)->(V+W-)->(V+U-)->(W+U-)->(W+V-)*/
    /*逆时针对应的过零信号：513264（5(U+V-)1(U+W-)3(V+W-)2(V+U-)6(W+U-)4(W+V-)）*/
    &m1_uhwl, &m1_vhul, &m1_vhwl,
    &m1_whvl, &m1_uhvl, &m1_whul
};

/*顺时针旋转过零信号换向表（和顺时针旋转顺序相反）*/
pctr pfunclist_m2[6] =
{   
    /* 546231 */
    /*按过零信号（513264）换向 顺序如下*/
    /*5(W+V-)1(U+V-)3(U+W-)2(V+W-)6(V+U-)4(W+U-)*/
    /*顺时针对应的过零信号：546231（5(W+V-)->4(W+U-)->6(V+U-)->2(V+W-)->3(U+W-)->1(U+V-)）*/
    &m1_uhvl, &m1_vhwl, &m1_uhwl,
    &m1_whul, &m1_whvl, &m1_vhul       
};

/* 上下桥臂的导通情况，共6种，也称为6步换向 */
void m1_uhvl(void)
{
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR1 = g_bldc_motor1.pwm_duty;                 /* U相上桥臂PWM */
    htim1.Instance->CCR3 = 0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_SET);   /* V相下桥臂导通 */
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_RESET); /* U相下桥臂关闭 */
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_RESET); /* W相下桥臂关闭 */
}

void m1_uhwl(void)
{
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR1 = g_bldc_motor1.pwm_duty;
    htim1.Instance->CCR3 = 0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_RESET);
}

void m1_vhwl(void)
{
    htim1.Instance->CCR1=0;
    htim1.Instance->CCR2 = g_bldc_motor1.pwm_duty;
    htim1.Instance->CCR3=0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_RESET);
}

void m1_vhul(void)
{
    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR2 = g_bldc_motor1.pwm_duty;
    htim1.Instance->CCR3 = 0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_RESET);
}


void m1_whul(void)
{
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = g_bldc_motor1.pwm_duty;
    htim1.Instance->CCR1 = 0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_RESET);
}

void m1_whvl(void)
{
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = g_bldc_motor1.pwm_duty;
    htim1.Instance->CCR1 = 0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_RESET);
}


/**
 * @brief       检测输入信号是否发生变化
 * @param       val :输入信号
 * @note        测量速度使用，获取输入信号状态翻转情况，计算速度
 * @retval      0：计算高电平时间，1：计算低电平时间，2：信号未改变
 */
uint8_t uemf_edge(uint8_t val)
{
    /* 主要是检测val信号从0 - 1 在从 1 - 0的过程，即高电平所持续的过程 */
    static uint8_t oldval=0;
    if(oldval != val)
    {
        oldval = val;
        if(val == 0) return 0;
        else return 1;
    }
    return 2;
}
/*
    Rt = R *EXP(B*(1/T1-1/T2))
    这里T1和T2指的是K度即开尔文温度，K度=273.15(绝对温度)+摄氏度；其中T2=(273.15+25)
    Rt 是热敏电阻在T1温度下的阻值；
    R是热敏电阻在T2常温下的标称阻值；
    B值是热敏电阻的重要参数；
    EXP是e的n次方；
*/

const float Rp = 10000.0f;          /* 10K */
const float T2 = (273.15f + 25.0f); /* T2 */
const float Bx = 3380.0f;           /* B */
const float Ka = 273.15f;
float get_temp(uint16_t para)
{
    float Rt;
    float temp;
    Rt = 3.3f / (para * 3.3f / 4096.0f / 4700.0f) - 4700.0f;
    /* like this R=5000, T2=273.15+25,B=3470, RT=5000*EXP(3470*(1/T1-1/(273.15+25)) */
    temp = Rt / Rp;
    temp = log(temp);       /* ln(Rt/Rp) */
    temp /= Bx;             /* ln(Rt/Rp)/B */
    temp += (1.0f / T2);
    temp = 1.0f / (temp);
    temp -= Ka;
    return temp;
}
