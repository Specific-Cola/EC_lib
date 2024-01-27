//=====================================================================================================
// Motor.h
//=====================================================================================================
//
//       IRobot  EC_lib
//
// author: @ dji
// modify: @ Specific_Cola
// 
// 
//
//=====================================================================================================
#ifndef MOTOR_H__
#define MOTOR_H__

#include "main.h"
#include "struct_typedef.h"
#include "controller.h"

#include "djiMotor.h"
#include "DMMotor.h"



#define MAX_MOTOR_NUM      30 //
#define OFFLINE_TIME_MAX       0.1//单位s

typedef struct{
    fp32 last_angle;
    fp32 angle;
    int32_t rounds;
    fp32 series_angle;
    int32_t speed_rpm;
	
    fp32 given_current;
    fp32 torque;
    int32_t temperate;
}Motor_Info_t;

typedef struct{
    int16_t speed_rpm;
    fp32 angle;
    int16_t current;
    int16_t voltage;
    int16_t command;
}Motor_Command_t;

typedef struct{
	uint8_t statu;  //online 0  / offline 1 
	uint16_t motor_type;
    Motor_Info_t state_interfaces;
    Motor_Command_t command_interfaces;
	
	union{
		DJI_Motor_t *dji;
		DM_Motor_t 	*dm;
	}motor;
	
}Motor_t;

typedef union{
	DJI_Motor_Register_t dji_motor_set;
	DM_Motor_Register_t dm_motor_set;
	struct{
		uint32_t motor_type;
	};
}Motor_Register_t;

typedef struct{
    
    PIDInstance *pid;

}Controller_t;

Motor_t *motorAdd(Motor_Register_t *reg);
void motorEnable(Motor_t *motor);
void motorDisable(Motor_t *motor);
void motorInfoUpdate(Motor_t *motor);
void motorSpeedControl(Motor_t *motor,Speed_Controller_t *controller);//todo    移到电机总
Speed_Controller_t *speedControllerInit(PID_Init_Config_s *config);
void motorPositionControl(DJI_Motor_t *motor);
Return_t motorSendMessage();

#endif
