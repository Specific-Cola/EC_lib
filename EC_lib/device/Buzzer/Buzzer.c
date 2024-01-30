//=====================================================================================================
// Buzzer.c
//=====================================================================================================
//
//       IRobot  EC_lib
//
// author: @ dji
// modify: @ Specific_Cola
// 
//
//=====================================================================================================
#include "Buzzer.h"
#include "Buzzer_def.h"
#include "bsp_delay.h"

#include "main.h"

#include <stdlib.h>
#include <string.h>

static Buzzer_t *buzzer_instance[MAX_BUZZER_NUM];
static uint8_t id_cnt=0; //记录电机数量

const uint16_t buzzer_frequency[]={
	131,//倍低1
	139,//倍低升1
	147,//倍低2
	156,//倍低升2
	165,//倍低3
	175,//倍低4
	185,//倍低升4
	196,//倍低5
	208,//倍低升5
	220,//倍低6
	233,//倍低升6
	247,//倍低7
	
	262,//低1
	277,//低升1
	294,//低2
	311,//低升2
	330,//低3
	349,//低4
	370,//低升4
	392,//低5
	415,//低升5
	440,//低6
	466,//低升6
	494,//低7
	
	523,//1
	554,//升1
	587,//2
	622,//升2
	659,//3
	698,//4
	740,//升4
	784,//5
	831,//升5
	880,//6
	932,//升6
	988,//7
	
	1046,//高1
	1109,//高升1
	1175,//高2
	1245,//高升2
	1318,//高3
	1397,//高4
	1480,//高升4
	1568,//高5
	1661,//高升5
	1760,//高6
	1864,//高升6
	1975,//高7
	
	2093,//倍高1
	2217,//倍高升1
	2349,//倍高2
	2489,//倍高升2
	2637,//倍高3
	2794,//倍高4
	2960,//倍高升4
	3136,//倍高5
	3322,//倍高升5
	3520,//倍高6
	3729,//倍高升6
	3951,//倍高7
};

Buzzer_t *buzzerInit(Buzzer_Register_t *reg){
	
	Buzzer_t *instance = (Buzzer_t *)malloc(sizeof(Buzzer_t));
	
	PWM_Register_t pwm_reg;
	pwm_reg.htim 		= reg->htim;
	pwm_reg.channel		= reg->channel;
	pwm_reg.dutyratio	= 0.5;
	pwm_reg.period		= 1000.0/buzzer_frequency[BUZZER_C_MAJOR_OFFSET + BUZZER_SI_OFFSET];
	pwm_reg.callback	= NULL;
	instance->pwm = pwmRegister(&pwm_reg);
	instance->volume = 0.5;
	pwmOnDeactivate(instance->pwm);
	buzzer_instance[id_cnt++]=instance;
	return instance;
}

void buzzerOnce(Buzzer_t *buzzer){
	pwmOnActivate(buzzer->pwm);
	delayMs(50);
	pwmOnDeactivate(buzzer->pwm);
	delayMs(50);
}

void buzzerSeveralTimes(Buzzer_t *buzzer,uint8_t times,uint32_t duration_ms){
	uint8_t i=0;
	uint8_t beeing_ms=duration_ms*1.0/times/2;
	uint8_t gap_ms=duration_ms*1.0/times-beeing_ms;
	
	
	for(i=0;i<times;i++){
		pwmOnActivate(buzzer->pwm);
		delayMs(beeing_ms);
		pwmOnDeactivate(buzzer->pwm);
		delayMs(gap_ms);
	}
}

void buzzerSetTune(Buzzer_t *buzzer,uint32_t hz){
	pwmSetPeriod(buzzer->pwm,1000.0*hz);
}

void buzzerSetVolume(Buzzer_t *buzzer,fp32 percent){
	if(percent>1){
		percent=1;
	}
	if(percent<0){
		percent=0;
	}
	buzzer->volume = percent;
	pwmSetDuty(buzzer->pwm,0.5*buzzer->volume);
}

void buzzerSetMusicScore(Buzzer_t *buzzer,int16_t *score,uint32_t length,uint16_t bpm,uint8_t standard){
	fp32 beat_ms=60000.0/bpm;
	fp32 buzzer_ms=0;
	fp32 this_beat_ms=0;
	uint32_t i=0;
	
	for(i=0;i<length;i++){
		if(score[i<<1]==BUZZER_GAP){
			pwmSetDuty(buzzer->pwm,0);
		}
		else{
			pwmSetPeriod(buzzer->pwm,1000.0/buzzer_frequency[standard+score[i<<1]]);
			pwmSetDuty(buzzer->pwm,0.5*buzzer->volume);
		}
		
		this_beat_ms = beat_ms;
		buzzer_ms = beat_ms*score[(i<<1)+1]/100.0;
		if(score[(i<<1)+1]<=12){
			this_beat_ms = beat_ms/8;
		}
		else if(score[(i<<1)+1]<=25){
			this_beat_ms = beat_ms/4;
		}
		else if(score[(i<<1)+1]<=50){
			this_beat_ms = beat_ms/2;
		}
		
		pwmOnActivate(buzzer->pwm);
		delayUs(buzzer_ms*1000);
		pwmOnDeactivate(buzzer->pwm);
		delayUs((this_beat_ms-buzzer_ms)*1000);
	}
}
