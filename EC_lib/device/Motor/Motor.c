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
#include <stdlib.h>
#include <string.h>

static Motor_t *motor_indicator[MAX_DJI_MOTOR_NUM];
static uint8_t id_cnt; //记录大疆电机数量

#define get_motor_mask(indicator) ((indicator)->motor_type&0xFFF0)

Motor_t *motorAdd(Motor_Register_t *reg){
	Motor_t *motor;
	uint16_t motor_mask;
	motor_mask = get_motor_mask(reg);
	
	switch(motor_mask){
		case DJI_MOTOR_MASK:
		{
			motor->motor.dji = djiMotorAdd(&reg->dji_motor_set);
		}
		break;
		
		case DM_MOTOR_MASK:
		{
			motor->motor.dm = dmMotorAdd(&reg->dm_motor_set);
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
			motor->state_interfaces.last_angle 		= motor->state_interfaces.angle;
			motor->state_interfaces.angle 			= motor->motor.dji->state_interfaces.angle;
			motor->state_interfaces.speed_rpm 		= motor->motor.dji->state_interfaces.speed_rpm;
			motor->state_interfaces.given_current 	= motor->motor.dji->state_interfaces.given_current;
			motor->state_interfaces.angle = motor->motor.dji->state_interfaces.angle;
			motor->state_interfaces.angle = motor->motor.dji->state_interfaces.angle;
			motor->state_interfaces.angle = motor->motor.dji->state_interfaces.angle;
			motor->state_interfaces.angle = motor->motor.dji->state_interfaces.angle;
			motor->state_interfaces.angle = motor->motor.dji->state_interfaces.angle;
			motor->state_interfaces.angle = motor->motor.dji->state_interfaces.angle;
		}
		break;
		
		case DM_MOTOR_MASK:
		{
			dmMotorDisable(motor->motor.dm);
		}
		break;
	}
	
}
void motorSpeedControl(Motor_t *motor,Speed_Controller_t *controller);//todo    移到电机总
Speed_Controller_t *speedControllerInit(PID_Init_Config_s *config);
void motorPositionControl(DJI_Motor_t *motor);
Return_t MotorSendMessage();