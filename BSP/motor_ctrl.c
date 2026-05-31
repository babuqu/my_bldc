/**
 ****************************************************************************************************
 * @file        motor_ctrl.c
 * @author      Motor Control Team
 * @version     V1.0
 * @date        2026-05-24
 * @brief       无刷电机综合控制实现
 ****************************************************************************************************
 */

#include "motor_ctrl.h"
#include "cmsis_os2.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"

/******************************************************************************************/
/* 全局变量 */
MotorCtrl_t g_motor_ctrl = {0};

/* 信号�?*/
osSemaphoreId_t g_bemf_ready_sem = NULL;
osSemaphoreId_t g_commutation_ready_sem = NULL;

/* 队列 */
QueueHandle_t g_motor_cmd_queue = NULL;
QueueHandle_t g_motor_data_queue = NULL;

/* 任务句柄 (�?freertos.c 引入) */
extern osThreadId_t CommutationTaskHandle;
extern osThreadId_t BEMF_TaskHandle;
extern osThreadId_t SpeedTaskHandle;
extern osThreadId_t MonitorTaskHandle;

/* 其他全局变量 */
extern _bldc_obj g_bldc_motor1;
extern PID_TypeDef g_speed_pid;
extern Hallless g_hallless_three;
extern VvvF_start g_vvvf_sp;
extern uint8_t g_zero_ctr_status;
extern GetValue_t sensor_data;

/* 静态函数声�?*/
static void MotorCtrl_StateMachine(void);
static void MotorCtrl_ProtectionCheck(void);
static void MotorCtrl_UpdateData(void);
static uint8_t MotorCtrl_CheckFaults(void);

/******************************************************************************************/
/**
 * @brief       电机控制初始�? * @param       �? * @retval      �? */
void MotorCtrl_Init(void)
{
    /* 初始化电机控制结构体 */
    g_motor_ctrl.state = MOTOR_STATE_IDLE;
    g_motor_ctrl.ctrl_mode = MOTOR_CTRL_MODE_OPENLOOP;
    g_motor_ctrl.target_speed = 0;
    g_motor_ctrl.actual_speed = 0;
    g_motor_ctrl.target_pos = 0;
    g_motor_ctrl.actual_pos = 0;
    g_motor_ctrl.pwm_duty = 0;
    g_motor_ctrl.dir = CCW;
    g_motor_ctrl.fault_code = MOTOR_FAULT_NONE;
    g_motor_ctrl.run_time = 0;
    
    /* 创建信号�?*/
    osSemaphoreAttr_t sem_attr = {0};
    sem_attr.name = "bemf_ready";
    g_bemf_ready_sem = osSemaphoreNew(1, 0, &sem_attr);
    
    sem_attr.name = "commutation_ready";
    g_commutation_ready_sem = osSemaphoreNew(1, 0, &sem_attr);
    
    /* 创建队列 */
    g_motor_cmd_queue = xQueueCreate(10, sizeof(uint32_t));
    g_motor_data_queue = xQueueCreate(10, sizeof(MotorCtrl_t));
    
    /* 初始�?PID */
    pid_init();
    
    /* 初始�?BLDC 硬件 */
    hall_gpio_init();
    
    UartPrintf("MotorCtrl initialized\r\n");
}

/**
 * @brief       启动电机
 * @param       speed: 目标速度 (RPM)
 * @param       dir: 转向 (CCW/CW)
 * @retval      �? */
void MotorCtrl_StartMotor(int32_t speed, uint8_t dir)
{
    if(g_motor_ctrl.state == MOTOR_STATE_FAULT)
    {
        UartPrintf("Motor in fault state, cannot start\r\n");
        return;
    }
    
    g_motor_ctrl.target_speed = speed;
    g_motor_ctrl.dir = dir;
    g_bldc_motor1.dir = dir;
    g_speed_pid.SetPoint = (float)speed;
    
    /* 进入预定位状�?*/
    g_motor_ctrl.state = MOTOR_STATE_ALIGN;
    g_bldc_motor1.run_flag = RUN;
    
    UartPrintf("Motor start: speed=%d, dir=%d\r\n", speed, dir);
}

/**
 * @brief       停止电机
 * @param       �? * @retval      �? */
void MotorCtrl_StopMotor(void)
{
    g_motor_ctrl.state = MOTOR_STATE_STOP;
    g_bldc_motor1.run_flag = STOP;
    g_bldc_motor1.pwm_duty = 0;
    g_speed_pid.SetPoint = 0;
    stop_motor1();
    
    UartPrintf("Motor stopped\r\n");
}

/**
 * @brief       设置速度
 * @param       speed: 目标速度 (RPM)
 * @retval      �? */
void MotorCtrl_SetSpeed(int32_t speed)
{
    if(g_motor_ctrl.state == MOTOR_STATE_CLOSEDLOOP)
    {
        g_motor_ctrl.target_speed = speed;
        g_speed_pid.SetPoint = (float)speed;
        UartPrintf("Speed set: %d RPM\r\n", speed);
    }
}

/**
 * @brief       设置位置
 * @param       pos: 目标位置
 * @retval      �? */
void MotorCtrl_SetPosition(int32_t pos)
{
    g_motor_ctrl.target_pos = pos;
    UartPrintf("Position set: %d\r\n", pos);
}

/**
 * @brief       急停
 * @param       �? * @retval      �? */
void MotorCtrl_EmergencyStop(void)
{
    g_motor_ctrl.state = MOTOR_STATE_FAULT;
    g_motor_ctrl.fault_code = MOTOR_FAULT_NONE; /* 手动急停，非故障 */
    g_bldc_motor1.run_flag = STOP;
    g_bldc_motor1.pwm_duty = 0;
    stop_motor1();
    
    UartPrintf("Emergency stop!\r\n");
}

/**
 * @brief       获取电机状�? * @param       �? * @retval      电机状�? */
MotorState_t MotorCtrl_GetState(void)
{
    return g_motor_ctrl.state;
}

/**
 * @brief       获取故障�? * @param       �? * @retval      故障�? */
uint8_t MotorCtrl_GetFaultCode(void)
{
    return g_motor_ctrl.fault_code;
}

/**
 * @brief       清除故障
 * @param       �? * @retval      �? */
void MotorCtrl_ClearFault(void)
{
    if(g_motor_ctrl.state == MOTOR_STATE_FAULT)
    {
        g_motor_ctrl.fault_code = MOTOR_FAULT_NONE;
        g_motor_ctrl.state = MOTOR_STATE_IDLE;
        UartPrintf("Fault cleared\r\n");
    }
}

/******************************************************************************************/
/**
 * @brief       电机控制状态机
 * @param       �? * @retval      �? */
static void MotorCtrl_StateMachine(void)
{
    static uint32_t align_count = 0;
    static uint32_t openloop_count = 0;
    
    switch(g_motor_ctrl.state)
    {
        case MOTOR_STATE_IDLE:
            g_motor_ctrl.pwm_duty = 0;
            g_bldc_motor1.pwm_duty = 0;
            break;
            
        case MOTOR_STATE_INIT:
            /* 初始化状态，等待启动命令 */
            break;
            
        case MOTOR_STATE_ALIGN:
            /* 预定位：固定转子到初始位�?(U+V-) */
            SHUTDOWN_EN;
            start_motor1();
            g_bldc_motor1.pwm_duty = MAX_PWM_DUTY / 20; /* 5% 占空�?*/
            m1_uhvl();                                   /* U+V- */
            
            align_count++;
            if(align_count >= 200) /* 定位 200ms */
            {
                align_count = 0;
                g_motor_ctrl.state = MOTOR_STATE_OPENLOOP;
                g_zero_ctr_status = 0;
                g_vvvf_sp.Freq_T_Ref = 300;
                UartPrintf("Align complete, enter openloop\r\n");
            }
            break;
            
        case MOTOR_STATE_OPENLOOP:
            /* 开环加速：V/F 曲线升�?*/
            if(g_zero_ctr_status < 3)
            {
                /* 无感启动流程�?TIM1 中断中执�?*/
                openloop_count++;
                if(openloop_count % 100 == 0)
                {
                    UartPrintf("Openloop running, status=%d\r\n", g_zero_ctr_status);
                }
            }
            else
            {
                /* 检测到反电动势，进入闭�?*/
                openloop_count = 0;
                g_motor_ctrl.state = MOTOR_STATE_CLOSEDLOOP;
                g_motor_ctrl.ctrl_mode = MOTOR_CTRL_MODE_CLOSEDLOOP;
                UartPrintf("Openloop complete, enter closedloop\r\n");
            }
            break;
            
        case MOTOR_STATE_CLOSEDLOOP:
            /* 闭环运行：PID 速度控制 */
            g_motor_ctrl.actual_speed = g_bldc_motor1.speed;
            g_motor_ctrl.actual_pos = g_bldc_motor1.pos;
            
            /* 检查速度误差 */
            float speed_error = fabsf(g_speed_pid.SetPoint - (float)g_motor_ctrl.actual_speed);
            if(speed_error > 100.0f) /* 误差超过 100RPM */
            {
                g_motor_ctrl.fault_code = MOTOR_FAULT_SPEED_ERROR;
                g_motor_ctrl.state = MOTOR_STATE_FAULT;
                UartPrintf("Speed error! error=%.2f\r\n", speed_error);
            }
            break;
            
        case MOTOR_STATE_STOP:
            /* 停止状�?*/
            g_bldc_motor1.pwm_duty = 0;
            stop_motor1();
            g_motor_ctrl.state = MOTOR_STATE_IDLE;
            break;
            
        case MOTOR_STATE_FAULT:
            /* 故障状�?*/
            MotorCtrl_ProtectionCheck();
            break;
            
        default:
            g_motor_ctrl.state = MOTOR_STATE_IDLE;
            break;
    }
}

/**
 * @brief       保护检�? * @param       �? * @retval      �? */
static void MotorCtrl_ProtectionCheck(void)
{
    MotorCtrl_UpdateData();
    
    /* 过流保护 */
    if(fabsf(sensor_data.i_u) > 10.0f || 
       fabsf(sensor_data.i_v) > 10.0f || 
       fabsf(sensor_data.i_w) > 10.0f)
    {
        g_motor_ctrl.fault_code = MOTOR_FAULT_OVER_CURRENT;
        UartPrintf("Over current fault! i_u=%.2f, i_v=%.2f, i_w=%.2f\r\n", 
                   sensor_data.i_u, sensor_data.i_v, sensor_data.i_w);
    }
    
    /* 过压保护 */
    if(sensor_data.Vbus_v > 28.0f)
    {
        g_motor_ctrl.fault_code = MOTOR_FAULT_OVER_VOLTAGE;
        UartPrintf("Over voltage fault! Vbus=%.2f\r\n", sensor_data.Vbus_v);
    }
    
    /* 欠压保护 */
    if(sensor_data.Vbus_v < 10.0f)
    {
        g_motor_ctrl.fault_code = MOTOR_FAULT_UNDER_VOLTAGE;
        UartPrintf("Under voltage fault! Vbus=%.2f\r\n", sensor_data.Vbus_v);
    }
    
    /* 过温保护 */
    if(sensor_data.temperature > 80.0f)
    {
        g_motor_ctrl.fault_code = MOTOR_FAULT_OVER_TEMP;
        UartPrintf("Over temperature fault! temp=%.2f\r\n", sensor_data.temperature);
    }
    
    /* 堵转保护 */
    if(g_bldc_motor1.locked_rotor == 1)
    {
        g_motor_ctrl.fault_code = MOTOR_FAULT_LOCKED_ROTOR;
        UartPrintf("Locked rotor fault!\r\n");
    }
    
    /* 故障时关闭电�?*/
    if(g_motor_ctrl.fault_code != MOTOR_FAULT_NONE)
    {
        g_bldc_motor1.run_flag = STOP;
        g_bldc_motor1.pwm_duty = 0;
        stop_motor1();
    }
}

/**
 * @brief       更新电机数据
 * @param       �? * @retval      �? */
static void MotorCtrl_UpdateData(void)
{
    g_motor_ctrl.actual_speed = g_bldc_motor1.speed;
    g_motor_ctrl.actual_pos = g_bldc_motor1.pos;
    g_motor_ctrl.actual_current = (int16_t)((sensor_data.i_u + sensor_data.i_v + sensor_data.i_w) / 3.0f * 1000);
    g_motor_ctrl.temperature = sensor_data.temperature;
    g_motor_ctrl.bus_voltage = sensor_data.Vbus_v;
    g_motor_ctrl.pwm_duty = g_bldc_motor1.pwm_duty;
    
    if(g_motor_ctrl.state == MOTOR_STATE_CLOSEDLOOP || 
       g_motor_ctrl.state == MOTOR_STATE_OPENLOOP)
    {
        g_motor_ctrl.run_time += 10; /* 10ms 更新一�?*/
    }
}

/**
 * @brief       检查故�? * @param       �? * @retval      1-有故障，0-无故�? */
static uint8_t MotorCtrl_CheckFaults(void)
{
    /* 过流检�?*/
    if(fabsf(sensor_data.i_u) > 10.0f || 
       fabsf(sensor_data.i_v) > 10.0f || 
       fabsf(sensor_data.i_w) > 10.0f)
    {
        return 1;
    }
    
    /* 过压检�?*/
    if(sensor_data.Vbus_v > 28.0f)
    {
        return 1;
    }
    
    /* 欠压检�?*/
    if(sensor_data.Vbus_v < 10.0f)
    {
        return 1;
    }
    
    /* 过温检�?*/
    if(sensor_data.temperature > 80.0f)
    {
        return 1;
    }
    
    return 0;
}

/******************************************************************************************/
/**
 * @brief       换相任务 (1ms 周期)
 * @param       argument: 未使�? * @retval      �? */
void MotorCtrl_CommutationTaskFun(void *argument)
{
    static uint32_t loop_count = 0;
    
    UartPrintf("CommutationTask started\r\n");
    
    for(;;)
    {
        loop_count++;
        
        /* 在闭环状态下处理换相逻辑 */
        if(g_motor_ctrl.state == MOTOR_STATE_CLOSEDLOOP)
        {
            /* 这里可以添加额外的换相处理逻辑 */
            /* 基本的换相已经在 TIM1 中断�?zero_ctr_loop() 中完�?*/
            
            if(loop_count % 100 == 0)
            {
                /* 每秒发送一次同步信�?*/
                osSemaphoreRelease(g_commutation_ready_sem);
            }
        }
        
        osDelay(COMMUTATION_TASK_PERIOD);
    }
}

/**
 * @brief       BEMF 处理任务 (2ms 周期)
 * @param       argument: 未使�? * @retval      �? */
void MotorCtrl_BEMF_Task_Fun(void *argument)
{
    static uint16_t bemf_filter[3] = {0};
    static uint8_t last_bemf_state = 0;
    
    UartPrintf("BEMF_Task started\r\n");
    
    for(;;)
    {
        /* 读取 ADC 中的反电动势�?*/
        float bemf_u = sensor_data.emf_u;
        float bemf_v = sensor_data.emf_v;
        float bemf_w = sensor_data.emf_w;
        
        /* 一阶滤�?*/
        FirstOrderRC_LPF(bemf_filter[0], (uint16_t)(bemf_u * 1000), 0.1f);
        FirstOrderRC_LPF(bemf_filter[1], (uint16_t)(bemf_v * 1000), 0.1f);
        FirstOrderRC_LPF(bemf_filter[2], (uint16_t)(bemf_w * 1000), 0.1f);
        
        /* 过零检测辅助判�?*/
        if(g_motor_ctrl.state == MOTOR_STATE_OPENLOOP)
        {
            /* 在开环阶段监测反电动势，准备切入闭环 */
            if(bemf_u > 0.5f || bemf_v > 0.5f || bemf_w > 0.5f)
            {
                /* 检测到足够的反电动�?*/
                if(g_zero_ctr_status >= 3)
                {
                    UartPrintf("BEMF ready for closedloop\r\n");
                }
            }
        }
        
        /* 发送同步信�?*/
        osSemaphoreRelease(g_bemf_ready_sem);
        
        osDelay(BEMF_TASK_PERIOD);
    }
}

/**
 * @brief       速度控制任务 (10ms 周期)
 * @param       argument: 未使�? * @retval      �? */
void MotorCtrl_SpeedTaskFun(void *argument)
{
    static int32_t temp_pwm = 0;
    static int32_t motor_pwm_s = 0;
    
    UartPrintf("SpeedTask started\r\n");
    
    for(;;)
    {
        /* 状态机处理 */
        MotorCtrl_StateMachine();
        
        /* 更新电机数据 */
        MotorCtrl_UpdateData();
        
        /* 保护检�?*/
        if(MotorCtrl_CheckFaults())
        {
            g_motor_ctrl.state = MOTOR_STATE_FAULT;
            g_motor_ctrl.fault_code = MOTOR_FAULT_OVER_CURRENT; /* 简化处�?*/
        }
        
        /* 速度�?PID 控制 (仅在闭环状态下执行) */
        if(g_motor_ctrl.state == MOTOR_STATE_CLOSEDLOOP)
        {
            /* 执行 PID 计算 */
            temp_pwm = increment_pid_ctrl(&g_speed_pid, (float)g_bldc_motor1.speed);
            
            /* 一阶低通滤�?*/
            FirstOrderRC_LPF(motor_pwm_s, temp_pwm, 0.085f);
            
            /* 限制最低速度 */
            if(motor_pwm_s < 0)
            {
                if(motor_pwm_s >= -600)
                    motor_pwm_s = -600;
                g_bldc_motor1.pwm_duty = -motor_pwm_s;
            }
            else
            {
                if(motor_pwm_s <= 600)
                    motor_pwm_s = 600;
                g_bldc_motor1.pwm_duty = motor_pwm_s;
            }
            
            g_motor_ctrl.pwm_duty = g_bldc_motor1.pwm_duty;
        }
        
        /* 发送数据到监控任务 */
        if(g_motor_data_queue != NULL)
        {
            xQueueSend(g_motor_data_queue, &g_motor_ctrl, 0);
        }
        
        osDelay(SPEED_TASK_PERIOD);
    }
}
