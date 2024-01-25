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

#include "main.h"
#include "struct_typedef.h"
#include "controller.h"
#include "bsp_can.h"
#include "bsp_dwt.h"

#define MAX_DJI_MOTOR_NUM      21 //姑且算一个can7个电机
#define OFFLINE_TIME_MAX       0.1//单位s
typedef enum
{
    MOTOR_6020 = 0,
	MOTOR_3508,
	MOTOR_2006,
	
} DJI_Motor_type_t;

typedef struct{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    uint16_t last_ecd;
    fp32 angle;

}DJI_Motor_Info_t;

typedef struct{
    int16_t speed_rpm;
    fp32 angle;
    int16_t current;
    int16_t voltage;
    int16_t command;
}Command_t;

typedef struct{
	uint8_t statu;  //online 0  / offline 1 
	DJI_Motor_type_t motor_type; //6020   3508   2006   need add pls contact lwt
    DJI_Motor_Info_t state_interfaces;
    Can_Device_t *can_info;
    Command_t command_interfaces;
                    
}DJI_Motor_t;

typedef struct{

	DJI_Motor_type_t motor_type;
	uint8_t id;
	CAN_HandleTypeDef *hcan;
}DJI_Motor_Register_t;

typedef struct{
    
    PIDInstance *pid;

}Speed_Controller_t;

DJI_Motor_t *djiMotorAdd(DJI_Motor_Register_t *reg);
void djiMotorDelete(DJI_Motor_t *motor);
void djiMotorInfoUpdate(DJI_Motor_t *motor,uint8_t *data);
void djiMotorSpeedControl(DJI_Motor_t *motor,Speed_Controller_t *controller);//todo    移到电机总
Speed_Controller_t *speedControllerInit(PID_Init_Config_s *config);
void djiMotorPositionControl(DJI_Motor_t *motor);
Return_t djiMotorSendMessage();
#endif // !DJIMOTOR_H__
