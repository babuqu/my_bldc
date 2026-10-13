/**
 ****************************************************************************************************
 * @file        zero_ctr.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-19
 * @brief       无感过零控制 驱动代码
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
 * V1.0 20220518
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "zero_ctr.h"
#include "bldc.h"
#include "tim.h"


Hallless g_hallless_three;                      /* 反电势结构体 */
VvvF_start  g_vvvf_sp;                          /* 无感开环结构体 */
uint16_t g_delay_temp = 0;                      /* 延迟变量 */
uint8_t g_zero_ctr_status = 0;                  /* 无感状态标志位 */
static uint16_t count_i = 0;                    /* 过零信号稳定标志 */
static uint8_t switch_hallless = 0;             /* 获取当前过零信号是否稳定 */


static uint32_t count_j = 0;                    /* 运行时间 */


/**
 * @brief       无感过零控制逻辑
 * @note        实现定位并以一定速度旋转并获取到稳定的过零信号之后，切换至过零控制
 * @param       无
 * @retval      无
 */
void zero_ctr_loop(void)
{
    if(g_bldc_motor1.run_flag == RUN)//如果开始运行
    {
        switch(g_zero_ctr_status)
        {
            case 0:/* 定位：固定一相U+V-*/
            {
                g_vvvf_sp.Voilage_Ref = (uint16_t)(MAX_PWM_DUTY/20);//(MAX_PWM_DUTY/20)
                g_bldc_motor1.pwm_duty = g_vvvf_sp.Voilage_Ref;
                m1_uhvl();                      /* U+V- */
                zero_ctr_init();
                g_vvvf_sp.Freq_T_Ref = 300;     /* 初始延迟时间为：55*300us，不同电机此参数不同 */
                g_zero_ctr_status = 1;          /* 切换至延迟计数 */
                count_i = 0;
                switch_hallless = 0;
                g_vvvf_sp.Count = 0;
            }
            break;
            case 1:/* 定时延迟 */
            {
                g_delay_temp++;
                if(g_delay_temp >= g_vvvf_sp.Freq_T_Ref)
                {
                    g_delay_temp = 0;
                    g_zero_ctr_status = 2;      /* 切换至换相操作 */
                }
#if (DRIVE_MODE)                                /* 过零闭环控制 为0开环控制 */
                switch_hallless = hallless_sw();/* 检测过零信号 稳定后切换至闭环 */
                if(switch_hallless == 1)
                {
                    g_zero_ctr_status = 3;
                }
#endif
            }
            break;
            case 2:                             /* 加速换相（提高换向频率）*/
            {
                g_vvvf_sp.Freq_T_Ref -= g_vvvf_sp.Freq_T_Ref/20+1;/* 改变换向频率 g_vvvf_sp.Freq_T_Ref/15+1;*/
                g_vvvf_sp.Count++;
                change_voltage();               /* 改变换向所需电压 */
                if(g_vvvf_sp.Freq_T_Ref < 180)
                {
                    g_vvvf_sp.Freq_T_Ref = 180; /* 固定次频率（延迟时间），不同电机此参数有差异4对级的可设置150 */
                    g_zero_ctr_status = 1;      /* 切换至延迟计数*/
                }
                else
                {
                    g_zero_ctr_status = 1;      /* 切换至延迟计数 */
                }
                g_vvvf_sp.VvvF_Count++;         /* 换向计数 */

                if(g_vvvf_sp.VvvF_Count == 6)
                {
                    g_vvvf_sp.VvvF_Count = 0;
                }
                anwerfen_sw();

            }
            break;
            case 3:
            {
                hallless_sw();                  /* 切换至闭环过零检测状态 */
            }
            break;
            default:break;
        }
    }
    else if(g_bldc_motor1.run_flag == STOP)
    {
        zero_ctr_init();
    }
}
/**
 * @brief       无感控制初始化，将无感状态标志清0
 * @param       无
 * @retval      无
 */
void zero_ctr_init(void)
{
    //g_zero_ctr_status = 2;
	  g_zero_ctr_status = 0;
    g_delay_temp = 0;
    g_vvvf_sp.Count = 0;
    g_vvvf_sp.VvvF_Count = 0;
    g_hallless_three.Filter_Delay = 0;
}
/**
 * @brief       无感开环6步换相顺序
 * @param       无
 * @retval      无
 */
void anwerfen_sw(void)
{
    if(g_bldc_motor1.dir == CCW)
    {
        switch(g_vvvf_sp.VvvF_Count)/*换向次数*/
        {
            /* 六步换向顺序:(U+V-)-> (U+W-)-> (V+W-)-> (V+U-)-> (W+U-)->(W+V-) */
            case  0x00:  m1_uhvl();break;       /* U+V-* 对应初始固定相 */
            case  0x01:  m1_uhwl();break;
            case  0x02:  m1_vhwl();break;
            case  0x03:  m1_vhul();break;
            case  0x04:  m1_whul();break;
            case  0x05:  m1_whvl();break;
            default:break;
        }
    }
    else
    {
        switch(g_vvvf_sp.VvvF_Count)
        {
            case  0x00:  m1_uhvl();break;       /* U+V-* 对应初始固定相 */
            case  0x01:  m1_whvl();break;
            case  0x02:  m1_whul();break;
            case  0x03:  m1_vhul();break;
            case  0x04:  m1_vhwl();break;
            case  0x05:  m1_uhwl();break;
            default:    break;
        }
    }

}
#define FSCA 2
/**
 * @brief       换相电压修改
 * @param       无
 * @retval      无
 */
void change_voltage(void)
{
    switch(g_vvvf_sp.Count)
    {
        case  1:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 10;break;
        case  2:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 9;break;
        case  3:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 8;break;
        case  4:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 7;break;
        case  5:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 7;break;
        case  6:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 6;break;
        case  7:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 6;break;
        case  8:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 5;break;
        case  9:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 5;break;
        case  10: g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 4;break;
        case  11: g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 4;break;
        case  12: g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 4;break;
        case  13: g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 4;break;
        case  14: g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 4;break;
        case  15: g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 4;break;
        default:    break;   
    }
}
/**
  * @brief  过零闭环控制
  * @param  无
  * @retval 0 ： 不稳定的过零信号  1：稳定的过零信号
  */
uint8_t hallless_sw(void)
{
    static uint8_t edge_flag = 0;
    count_j++;  /* 运行过程时间计数 18K的中断频率为单位 */
    /* 过零信号滤波 */
    g_hallless_three.Queue_Status[0] = g_hallless_three.Queue_Status[0] << 1;
    g_hallless_three.Queue_Status[1] = g_hallless_three.Queue_Status[1] << 1;
    g_hallless_three.Queue_Status[2] = g_hallless_three.Queue_Status[2] << 1;
    /* 检测过零点 */
    g_hallless_three.Queue_Status[0] |= HAL_GPIO_ReadPin(HALL1_TIM_CH1_GPIO,HALL1_TIM_CH1_PIN);   /* U相过零点 */
    g_hallless_three.Queue_Status[1] |= HAL_GPIO_ReadPin(HALL1_TIM_CH2_GPIO,HALL1_TIM_CH2_PIN);   /* V相过零点 */
    g_hallless_three.Queue_Status[2] |= HAL_GPIO_ReadPin(HALL1_TIM_CH3_GPIO,HALL1_TIM_CH3_PIN);   /* W相过零点 */
    g_hallless_three.Filter_Math = g_hallless_three.Queue_Status[0] & FILTER_LONG;
    /* 过零信号U */
    if(g_hallless_three.Filter_Math == FILTER_LONG)
    {
        g_hallless_three.QFilter_Status[0] = 1;

    }
    else if(g_hallless_three.Filter_Math == 0x00)
    {
        g_hallless_three.QFilter_Status[0] = 0;
    }
    else
    {
        g_hallless_three.Filter_Count++;
        return 2;
    }
    /* 过零信号V */
    g_hallless_three.Filter_Math = g_hallless_three.Queue_Status[1] & FILTER_LONG;
    if(g_hallless_three.Filter_Math == FILTER_LONG)
    {
        g_hallless_three.QFilter_Status[1] = 1;
    }
    else if(g_hallless_three.Filter_Math == 0x00)
    {
        g_hallless_three.QFilter_Status[1] = 0;
    }
    else
    {
        g_hallless_three.Filter_Count++;
        return 2;
    }
    /* 过零信号W */
    g_hallless_three.Filter_Math = g_hallless_three.Queue_Status[2] & FILTER_LONG;
    if(g_hallless_three.Filter_Math == FILTER_LONG)
    {
        g_hallless_three.QFilter_Status[2] = 1;
    }
    else if(g_hallless_three.Filter_Math == 0x00)
    {
        g_hallless_three.QFilter_Status[2] = 0;
    }
    else
    {
        g_hallless_three.Filter_Count++;
        return 2;
    }
    /************************************** 速度计算 ***************************************/
    g_hallless_three.Filter_Edge = uemf_edge(g_hallless_three.QFilter_Status[0]);
    if(g_hallless_three.Filter_Edge == 0)       /* 统计过零信号的高电平时间 */
    {
        /* 延迟30°换相，因为硬件上低通滤波器和软件延迟的原因，实际延迟角度需小于30° */
        if(count_i >= 4)                        /* 稳定检测到过零信号才对速度进行测量 */
        {
            if(g_bldc_motor1.dir == CW)
                g_hallless_three.Speed_RPM = ((uint32_t)SPEED_COEFF/count_j);
            else
                g_hallless_three.Speed_RPM = -((uint32_t)SPEED_COEFF/count_j);
            FirstOrderRC_LPF(g_bldc_motor1.speed,g_hallless_three.Speed_RPM,0.2379);
        }
        g_hallless_three.Filter_Delay = count_j/ 10;/* 实际需延迟时间 */
        g_hallless_three.Filter_Count = 0;
        count_j = 0;
        count_i++;                              /* 捕捉过零点，累计次数 */
    }
    if(g_hallless_three.Filter_Edge == 1)
    {
        g_hallless_three.Filter_Count = 0;
        count_j = 0;
    }
    if(g_hallless_three.Filter_Edge == 2)
    {
        g_hallless_three.Filter_Count++;        /* 不换相时间累计 超时则判定速度为0*/
        if(g_hallless_three.Filter_Count > 15000)
        {
            g_hallless_three.Filter_Count = 0;
            g_hallless_three.Speed_RPMF = 0;    /* 超时换向 判定为停止 速度为0*/
            g_bldc_motor1.locked_rotor = 1;
            stop_motor1();
            g_bldc_motor1.run_flag = STOP;      /* 标记停机*/
            g_bldc_motor1.pwm_duty = 0;
					  //g_bldc_motor1.pwm_duty = 1000;
        }
    }
    /************************************** 过零控制 ***************************************/
    if(count_i >= 4)                        /* 稳定检测到过零信号 并旋转2圈之后进入 */
    {
        count_i = 4;
        edge_flag++;                        /* edge_flag==2 才正式进入过零闭环控制 */
        if(edge_flag >= 2)
        {
            edge_flag = 2;
            g_hallless_three.Hallless_State = g_hallless_three.QFilter_Status[0] | (g_hallless_three.QFilter_Status[1]<<1) | (g_hallless_three.QFilter_Status[2]<<2);
            if(g_hallless_three.Hallless_State <= 0x00 || g_hallless_three.Hallless_State > 0x06)
            {
                return 0;
            }
            if(g_hallless_three.Hallless_State != g_hallless_three.OldHallless_State)
            {
                g_hallless_three.Filter_Count_All++;
            }
            /******************* 调试延迟30°的时间之后，执行换相 ********************/
            //HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET);                       /* 代码调试用到 */
            if(g_hallless_three.Filter_Count_All >= (g_hallless_three.Filter_Delay))
            {
                g_hallless_three.Filter_Count_All = 0;
                if(g_hallless_three.Hallless_State != g_hallless_three.OldHallless_State)
                {
                    //HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_SET);
                    g_bldc_motor1.sum_pos++;
                    if(g_bldc_motor1.dir == CCW)
                    {
                        g_bldc_motor1.pos--;    /* 电机位置计数 */
                        pfunclist_m1[g_hallless_three.Hallless_State-1]();
                    }
                    else
                    {
                        g_bldc_motor1.pos++;    /* 电机位置计数 */
                        pfunclist_m2[g_hallless_three.Hallless_State-1]();
                    }
                }
                g_hallless_three.OldHallless_State = g_hallless_three.Hallless_State ;
            }
          return 1;
        }

     
    }
    return 0;

}



