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
#ifndef DMMOTOR_H__
#define DMMOTOR_H__

#include "main.h"
#include "struct_typedef.h"
#include "controller.h"
#include "bsp_can.h"

#define MAX_DM_MOTOR_NUM      5
#define OFFLINE_TIME_MAX      0.1//单位s
typedef enum
{
	DM_MOTOR_MASK = 0x20,
    DM_MOTOR_4310 = 0x21,
	
} DM_Motor_type_t;

typedef struct{
    uint8_t id;
	uint8_t error;
	uint16_t pos;	//16位
	uint16_t vel; 	//12位
	uint16_t t;		//12位
	int8_t t_mos;
	int8_t t_rotor;
	
    fp32 position;
	fp32 velocity;
	fp32 torque;

}DM_Motor_Info_t;

typedef struct{
	fp32 pos;
	fp32 vel;
	fp32 t;
	fp32 kd;
	fp32 kp;
}DM_MIT_Command_t;

typedef struct DM_Motor_{
	uint8_t statu;  //online 0  / offline 1 
	DM_Motor_type_t motor_type; //6020   3508   2006   need add pls contact lwt
    DM_Motor_Info_t state_interfaces;
    Can_Device_t *can_info;
	DM_MIT_Command_t command_interfaces;
	
	void (*motorCallback)(struct DM_Motor_*);
	
	fp32 p_max;
	fp32 v_max;
	fp32 t_max;
	fp32 kp_max;
	fp32 kd_max;
	fp32 kp_min;
	fp32 kd_min;
}DM_Motor_t;

typedef struct{

	DM_Motor_type_t motor_type;
	CAN_HandleTypeDef *hcan;
	uint16_t rx_id;
	uint16_t tx_id;
	
	fp32 p_max;
	fp32 v_max;
	fp32 t_max;
	fp32 kp_max;
	fp32 kd_max;
	fp32 kp_min;
	fp32 kd_min;
	
}DM_Motor_Register_t;

DM_Motor_t *dmMotorAdd(DM_Motor_Register_t *reg);
void dmMotorDelete(DM_Motor_t *motor);
void dmMotorInfoUpdate(DM_Motor_t *motor,uint8_t *data);
void dmMotorEnable(DM_Motor_t *motor);
void dmMotorDisable(DM_Motor_t *motor);
void dmMotorSetZero(DM_Motor_t *motor);
void dmMotorClearError(DM_Motor_t *motor);
Return_t dmMotorSendMessage(DM_Motor_t *motor);
Return_t dmMotorSendAll();

#endif
