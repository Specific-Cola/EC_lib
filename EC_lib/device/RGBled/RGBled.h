//=====================================================================================================
// Buzzer.h
//=====================================================================================================
//
//       IRobot  EC_lib
//
// author: @ dji
// modify: @ Specific_Cola
// 
//
//=====================================================================================================
#ifndef RGBLED_H__
#define RGBLED_H__

#include "bsp_pwm.h"
#include "struct_typedef.h"

#define MAX_LED_NUM 1//理论上C板应该也只有一个RGB灯

typedef struct{
	
	PWM_Device_t *pwm_red;
	PWM_Device_t *pwm_blue;
	PWM_Device_t *pwm_green;
	
	uint8_t color[3];
	
}Buzzer_t;

typedef struct{
	
	
	
}Buzzer_Register_t;

Buzzer_t *buzzerInit(Buzzer_Register_t *reg);
void buzzerOnce(Buzzer_t *buzzer);
void buzzerSeveralTimes(Buzzer_t *buzzer,uint8_t times,uint32_t duration_ms);
void buzzerSetTune(Buzzer_t *buzzer,uint32_t hz);
void buzzerSetVolume(Buzzer_t *buzzer,fp32 percent);

void buzzerSetMusicScore(Buzzer_t *buzzer,int16_t *score,uint32_t length,uint16_t bpm,uint8_t standard);


#endif // !RGB_H__

