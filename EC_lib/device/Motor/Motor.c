//=====================================================================================================
// Motor.c
//=====================================================================================================
//
//       IRobot  EC_lib
//
// author: @ dji
// modify: @ Specific_Cola
// 
//
//=====================================================================================================
#include "Motor.h"
#include "main.h"
#include "arm_math.h"
#include <stdlib.h>
#include <string.h>

static Motor_t *motor_indicator[MAX_MOTOR_NUM];
static uint8_t id_cnt; //记录电机数量

#define get_motor_mask(indicator) ((indicator)->motor_type&0xFFF0)

static void djiMotorInfo(DJI_Motor_t *motor);
static void dmMotorInfo(DM_Motor_t *motor);

Motor_t *motorAdd(Motor_Register_t *reg){
	uint16_t motor_mask;
	motor_mask = get_motor_mask(reg);
	
	Motor_t *motor = (Motor_t *)malloc(sizeof(Motor_t));
	
	switch(motor_mask){
		case DJI_MOTOR_MASK:
		{
			motor->motor.dji = djiMotorAdd(&reg->dji_motor_set);
			motor->motor.dji->motorCallback = djiMotorInfo;
		}
		break;
		
		case DM_MOTOR_MASK:
		{
			motor->motor.dm = dmMotorAdd(&reg->dm_motor_set);
			motor->motor.dm->motorCallback = dmMotorInfo;
		}
		break;
	}
	motor->statu=0;
	
	motor_indicator[id_cnt++]=motor;
	
	return motor;
}

void motorEnable(Motor_t *motor){
	uint16_t motor_mask;
	motor_mask = get_motor_mask(motor);
	switch(motor_mask){
		case DJI_MOTOR_MASK:
		{
			__NOP();
		}
		break;
		
		case DM_MOTOR_MASK:
		{
			dmMotorEnable(motor->motor.dm);
		}
		break;
	}
}
void motorDisable(Motor_t *motor){
	uint16_t motor_mask;
	motor_mask = get_motor_mask(motor);
	switch(motor_mask){
		case DJI_MOTOR_MASK:
		{
			__NOP();
		}
		break;
		
		case DM_MOTOR_MASK:
		{
			dmMotorDisable(motor->motor.dm);
		}
		break;
	}
}

void motorInfoUpdate(Motor_t *motor){
	uint16_t motor_mask;
	motor_mask = get_motor_mask(motor);
	switch(motor_mask){
		case DJI_MOTOR_MASK:
		{
			motor->state_interfaces.speed_rpm 		= motor->motor.dji->state_interfaces.speed_rpm;
			motor->state_interfaces.current 		= motor->motor.dji->state_interfaces.given_current;
			motor->state_interfaces.temperate 		= motor->motor.dji->state_interfaces.temperate;
			
			motor->state_interfaces.last_angle 		= motor->state_interfaces.angle;
			
			motor->state_interfaces.speed_rpm 		= motor->motor.dji->state_interfaces.speed_rpm;
			motor->state_interfaces.speed_rpm 		= motor->motor.dji->state_interfaces.speed_rpm;
			motor->state_interfaces.speed_rpm 		= motor->motor.dji->state_interfaces.speed_rpm;
			motor->state_interfaces.speed_rpm 		= motor->motor.dji->state_interfaces.speed_rpm;
		}
		break;
		
		case DM_MOTOR_MASK:
		{
			dmMotorDisable(motor->motor.dm);
		}
		break;
	}
	
}

//不建议使用，使用各电机头中的发送函数
Return_t MotorSendAll(){
	Return_t ret=RETURN_SUCCESS;
	
	ret|=djiMotorSendMessage();
	ret|=dmMotorSendAll();
	
	return ret;
}

static void djiMotorInfo(DJI_Motor_t *motor){
	uint8_t i=0;
	for(i=0;i<id_cnt;i++){
		if(motor_indicator[i]->motor.dji == motor){
			motor_indicator[i]->state_interfaces.last_angle	= motor_indicator[i]->state_interfaces.angle;
			motor_indicator[i]->state_interfaces.angle 		= motor->state_interfaces.ecd/8192*360;
			motor_indicator[i]->state_interfaces.current	= motor->state_interfaces.given_current;
			motor_indicator[i]->state_interfaces.speed_rpm	= motor->state_interfaces.speed_rpm;
			motor_indicator[i]->state_interfaces.temperate	= motor->state_interfaces.temperate;
			motor_indicator[i]->state_interfaces.torque		= 0;
			
			//转速不可高于45000rpm
			if ((motor_indicator[i]->state_interfaces.angle - motor_indicator[i]->state_interfaces.last_angle) > 270)
            {
                motor_indicator[i]->state_interfaces.rounds--;
            }
            else if ((motor_indicator[i]->state_interfaces.angle - motor_indicator[i]->state_interfaces.last_angle) < -270)
            {
                motor_indicator[i]->state_interfaces.rounds++;
            }
			
            motor_indicator[i]->state_interfaces.series_angle = motor_indicator[i]->state_interfaces.angle  + 360* motor_indicator[i]->state_interfaces.rounds;
			
			
			break;
		}
	}
}

static void dmMotorInfo(DM_Motor_t *motor){
	uint8_t i=0;
	for(i=0;i<id_cnt;i++){
		if(motor_indicator[i]->motor.dm == motor){
			motor_indicator[i]->state_interfaces.last_angle	= motor_indicator[i]->state_interfaces.angle;
			motor_indicator[i]->state_interfaces.angle 		= motor->state_interfaces.position/PI*180;
			
			motor_indicator[i]->state_interfaces.current	= 0;
			motor_indicator[i]->state_interfaces.speed_rpm	= motor->state_interfaces.velocity/PI*60;
			motor_indicator[i]->state_interfaces.temperate	= motor->state_interfaces.t_mos;
			motor_indicator[i]->state_interfaces.torque		= motor->state_interfaces.torque;
			
			if ((motor_indicator[i]->state_interfaces.angle - motor_indicator[i]->state_interfaces.last_angle) > 270)
            {
                motor_indicator[i]->state_interfaces.rounds--;
            }
            else if ((motor_indicator[i]->state_interfaces.angle - motor_indicator[i]->state_interfaces.last_angle) < -270)
            {
                motor_indicator[i]->state_interfaces.rounds++;
            }
			
            motor_indicator[i]->state_interfaces.series_angle = motor_indicator[i]->state_interfaces.angle  + 360* motor_indicator[i]->state_interfaces.rounds;
			
			
			break;
		}
	}
}
