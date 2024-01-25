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



#define MAX_MOTOR_NUM      30 //
#define OFFLINE_TIME_MAX       0.1//单位s
typedef enum
{
    DJI_MOTOR_6020 = 0,
	DJI_MOTOR_3508,
	DJI_MOTOR_2006,
	
	DAMIAO_MOTOR_3510,
	
	YUSHU_MOTOR_A1,
} Motor_type_t;

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
	Motor_type_t motor_type; //
	union{
		DJI_Motor_t	*self;
	};
    Motor_Info_t state_interfaces;
    Can_Device_t *can_info;
    Command_t command_interfaces;
                    

}Motor_t;

typedef struct{

	Motor_type_t motor_type;
	uint8_t id;
	CAN_HandleTypeDef *hcan;
}Motor_Register_t;

typedef struct{
    
    PIDInstance *pid;

}Controller_t;

Motor_t *motorAdd(Motor_Register_t *reg);
void motorInfoUpdate(Motor_t *motor,uint8_t *data);
void motorSpeedControl(Motor_t *motor,Speed_Controller_t *controller);//todo    移到电机总
Speed_Controller_t *speedControllerInit(PID_Init_Config_s *config);
void motorPositionControl(DJI_Motor_t *motor);
Return_t MotorSendMessage();

#endif
