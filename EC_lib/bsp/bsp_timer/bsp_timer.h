//=====================================================================================================
// bsp_timer.h
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
#ifndef BSP_TIMER_H
#define BSP_TIMER_H

#include "struct_typedef.h"
#include "tim.h"
#include "main.h"

#define TIMER_DEVICE_CNT 5	//6,11,12,13,14
#define TIMER_FREQUENCY_MAX 1000000	

typedef struct TIMER_Device_ {
	
	
    TIM_HandleTypeDef *htim;
    uint32_t tclk;                           // 时钟频率
	uint32_t period;
    void (*timer_device_callback)(struct TIMER_Device_ *);
	
	uint32_t period_cnt;					//溢出次数
	
}Timer_Device_t;

typedef struct{
	
    TIM_HandleTypeDef *htim;
	uint32_t period;
    void (*timer_device_callback)(struct TIMER_Device_ *);
	
}Timer_Register_t;

Timer_Device_t *timerDeviceRegister(Timer_Register_t *reg);
void timerOnActivate(Timer_Device_t *timer);
void timerOnDeactivate(Timer_Device_t *timer);
void timerSetPeriod(Timer_Device_t *timer, uint32_t period);
float timerGetDeltaTime(Timer_Device_t *timer, uint32_t *cnt_last);
#endif