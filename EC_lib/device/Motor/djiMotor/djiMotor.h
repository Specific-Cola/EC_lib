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
    uint16_t last_ecd;
	
    fp32 angle;

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
	
}DJI_Motor_t;

typedef struct{

	DJI_Motor_type_t motor_type;
	CAN_HandleTypeDef *hcan;
	uint8_t id;
}DJI_Motor_Register_t;

typedef struct{
    
    PIDInstance *pid;
    float *fdb_src;

}Speed_Controller_t;
typedef struct{

    cascadePIDInstacne *cascade_pid;
    float *out_fdb_src;
    float *in_fdb_src;

}Position_Controller_t;

DJI_Motor_t *djiMotorAdd(DJI_Motor_Register_t *reg);
void djiMotorDelete(DJI_Motor_t *motor);
void djiMotorInfoUpdate(DJI_Motor_t *motor,uint8_t *data);
void djiMotorSpeedControl(DJI_Motor_t *motor,Speed_Controller_t *controller);//todo    移到电机总
Speed_Controller_t *speedControllerInit(PID_Init_Config_s *config);
void djiMotorPositionControl(DJI_Motor_t *motor,Position_Controller_t *controller);//todo   移到电机总
Position_Controller_t *positionControllerInit(cascade_PID_Init_Config_s *config,float *out_fdb);
Return_t djiMotorSendMessage();
void djiMotor_SwitchRing(DJI_Motor_t *motor,Position_Controller_t *controller);
#endif // !DJIMOTOR_H__
