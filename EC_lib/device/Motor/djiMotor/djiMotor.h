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

#define MAX_DJI_MOTOR_NUM      14 //姑且算一个can7个电机
#define OFFLINE_TIME_MAX       0.1//单位s
typedef enum
{
	DJI_MOTOR_MASK 	= 0x10,
    DJI_MOTOR_6020 	= 0x11,
	DJI_MOTOR_3508	= 0x12,
	DJI_MOTOR_2006	= 0x13,
	
} DJI_Motor_type_t;

typedef struct{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
	
}DJI_Motor_Info_t;

typedef struct{
    int16_t speed_rpm;
    fp32 angle;
    int16_t current;
    int16_t voltage;
    int16_t command;
}DJI_Command_t;

typedef struct DJI_Motor_{
	uint8_t statu;  //online 0  / offline 1 
	DJI_Motor_type_t motor_type; //6020   3508   2006   need add pls contact lwt
    DJI_Motor_Info_t state_interfaces;
    Can_Device_t *can_info;
    DJI_Command_t command_interfaces;
	
	void (*motorCallback)(struct DJI_Motor_*);
}DJI_Motor_t;

typedef struct{

	DJI_Motor_type_t motor_type;
	CAN_HandleTypeDef *hcan;
	uint8_t id;
}DJI_Motor_Register_t;

DJI_Motor_t *djiMotorAdd(DJI_Motor_Register_t *reg);
void djiMotorDelete(DJI_Motor_t *motor);
void djiMotorInfoUpdate(DJI_Motor_t *motor,uint8_t *data);
Return_t djiMotorSendMessage();
#endif // !DJIMOTOR_H__
