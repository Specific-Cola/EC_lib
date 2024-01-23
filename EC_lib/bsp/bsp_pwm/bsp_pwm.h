//=====================================================================================================
// bsp_pwm.h
//=====================================================================================================
//
//       IRobot  EC_lib
//
//GitHub: https://github.com/Specific_Cola
// question:  specificcola@proton.me
// Date			Author			Notes
// 
//
//=====================================================================================================
#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "struct_typedef.h"
#define PWM_DEVICE_CNT     16

typedef struct PWM_Device_
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    uint32_t channel;                        // 通道
    uint32_t tclk;                           // 时钟频率
    float period;                         // 周期   单位毫秒
    float dutyratio;                      // 占空比
    void (*callback)(struct PWM_Device_ *); // DMA传输完成回调函数
    void *id;                                // 实例ID
} PWM_Device_t;

typedef struct
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    uint32_t channel;                        // 通道
    uint32_t period;                         // 周期  单位毫秒
    float dutyratio;                      // 占空比
    void (*callback)(PWM_Device_t*); // DMA传输完成回调函数
    void *id;                                // 实例ID
} PWM_Register_t;


PWM_Device_t *pwmRegister(PWM_Register_t *config);
void pwmOnActivate(PWM_Device_t *pwm);
void pwmOnDeactivate(PWM_Device_t *pwm);
void pwmSetDuty(PWM_Device_t *pwm,float dutyratio);
void pwmSetPeriod(PWM_Device_t *pwm,uint32_t period);


#endif