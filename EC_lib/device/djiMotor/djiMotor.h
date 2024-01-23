//=====================================================================================================
// djiMotor.h
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
#ifndef DJIMOTOR_H__
#define DJIMOTOR_H__

#include "struct_typedef.h"
#include "bsp_can.h"
#include "controller.h"
#define MAX_DJI_MOTOR_NUM      21 //姑且算一个can7个电机

typedef enum
{
    MOTOR_6020 = 0,
	MOTOR_3508,
	MOTOR_2006,
	
} Motor_type_t;

typedef struct{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    uint16_t last_ecd;
    fp32 angle;

}Motor_Info_t;

typedef struct{
    int16_t speed_rpm;
    fp32 angle;
    int16_t current;
    int16_t voltage;
    int16_t command;
}Command_t;

typedef struct{
	uint8_t statu;  //online 0  / offline 1 
	uint8_t motor_type; //6020   3508   2006   need add pls contact lwt
    Motor_Info_t state_interfaces;
    Can_Device_t *can_info;
    Command_t command_interfaces;

}DJI_Motor_t;

typedef struct{
    
    PIDInstance *pid;

}Speed_Controller_t;

DJI_Motor_t *djiMotorAdd(uint8_t id ,uint8_t type,CAN_HandleTypeDef *hcan);
void djiMotorDelete(DJI_Motor_t *motor);
void djiMotorInfoUpdate(DJI_Motor_t *motor,uint8_t *data);
void djiMotorSpeedControl(DJI_Motor_t *motor,Speed_Controller_t *controller);
Speed_Controller_t *speedControllerInit(PID_Init_Config_s *config);
void djiMotorPositionControl(DJI_Motor_t *motor);
Return_t djiMotorSendMessage();
void djiMotorCallback(Can_Device_t *instance);
#endif // !DJIMOTOR_H__
