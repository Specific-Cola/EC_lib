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
#include "user_lib.h"
#include "main.h"
#include "arm_math.h"

#include <stdlib.h>
#include <string.h>

static Motor_t *motor_instance[MAX_MOTOR_NUM];
static uint8_t id_cnt; //记录电机数量

#define get_motor_mask(indicator) ((indicator)->motor_type&0xFFF0)


/***                        添加电机种类时必须修改的部分START                                         ***/
static void djiMotorInfo(DJI_Motor_t *motor);
static void dmMotorInfo(DM_Motor_t *motor);

Motor_t *motorAdd(Motor_Register_t *reg){
	uint16_t motor_mask;
	motor_mask = get_motor_mask(reg);
	
	Motor_t *motor = (Motor_t *)malloc(sizeof(Motor_t));
	
	switch(motor_mask){
		case DJI_MOTOR_MASK:
		{
			motor->dji = djiMotorAdd(&reg->dji_motor_set);
			motor->dji->motorCallback = djiMotorInfo;
		}
		break;
		
		case DM_MOTOR_MASK:
		{
			motor->dm = dmMotorAdd(&reg->dm_motor_set);
			motor->dm->motorCallback = dmMotorInfo;
		}
		break;
	}
	motor->statu=0;
	motor->motor_type = reg->motor_type;
	motor_instance[id_cnt++]=motor;
	
	return motor;
}

void motorEnable(Motor_t *motor){
	uint16_t motor_mask;
	motor_mask = get_motor_mask(motor);
	switch(motor_mask){
		case DJI_MOTOR_MASK:
		{
		}
		break;
		
		case DM_MOTOR_MASK:
		{
			dmMotorEnable(motor->dm);
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
		}
		break;
		
		case DM_MOTOR_MASK:
		{
			dmMotorDisable(motor->dm);
		}
		break;
	}
}

Return_t motorSetMessage(Motor_t *motor){
	uint16_t motor_mask;
	motor_mask = get_motor_mask(motor);
	switch(motor_mask){
		case DJI_MOTOR_MASK:
		{
			motor->dji->command_interfaces.command = motor->command_interfaces.command;
		}
		break;
		
		case DM_MOTOR_MASK:
		{
			motor->dm->command_interfaces.t = motor->command_interfaces.command;
		}
		break;
	}
	return RETURN_SUCCESS;
}

Return_t motorSendMessage(Motor_t *motor){
	
	motorSetMessage(motor);
	
	uint16_t motor_mask;
	motor_mask = get_motor_mask(motor);
	switch(motor_mask){
		case DJI_MOTOR_MASK:
		{
			//大疆电机无法单独发送
		}
		break;
		
		case DM_MOTOR_MASK:
		{
			dmMotorSendMessage(motor->dm);
		}
		break;
	}
	
	return RETURN_SUCCESS;
}

Return_t motorSendAll(const uint32_t mask_cmd){
	Return_t ret=RETURN_SUCCESS;
	
	uint8_t i=0;
	
	for(i=0;i<id_cnt;i++){
		if(motor_instance[i]->motor_type&mask_cmd){
			ret|=motorSendMessage(motor_instance[i]);
		}
	}	
	
	if(mask_cmd&DJI_MOTOR_MASK)
		ret|=djiMotorSendMessage();
	
	return ret;
}

/***                        添加电机种类时必须修改的部分END                                         ***/

void motorSpeedControl(Motor_t *motor,Speed_Controller_t *controller)
{
    motor->command_interfaces.command = (int16_t)PIDCalculate(controller->pid, motor->state_interfaces.speed_rpm,motor->command_interfaces.speed_rpm);
}

Speed_Controller_t *speedControllerInit(PID_Init_Config_s *config)
{
    Speed_Controller_t *controller = (Speed_Controller_t *)malloc(sizeof(Speed_Controller_t));
    memset(controller, 0, sizeof(Speed_Controller_t));
    PIDInstance *instance = (PIDInstance *)malloc(sizeof(PIDInstance));
    PIDInit(instance, config);
    controller->pid = instance;

    return controller;
}
Position_Controller_t *positionControllerInit(cascade_PID_Init_Config_s *config,float *out_fdb)
{
    Position_Controller_t *controller = (Position_Controller_t *)malloc(sizeof(Position_Controller_t));
    memset(controller, 0, sizeof(Position_Controller_t));
    cascadePIDInstacne *instance = (cascadePIDInstacne *)malloc(sizeof(cascadePIDInstacne));
    cascadePIDInit(instance,config);
    controller->out_fdb_src = out_fdb;
    controller->cascade_pid = instance;
    return controller;
}

void motorPositionControl(Motor_t *motor,Position_Controller_t *controller)
{
    motor->command_interfaces.angle = loop_float_constrain(motor->command_interfaces.angle,*controller->out_fdb_src - 180.f ,*controller->out_fdb_src + 180.f);
    motor->command_interfaces.command = cascadePIDCalculate(controller->cascade_pid,*controller->out_fdb_src,motor->state_interfaces.speed_rpm,motor->command_interfaces.angle);

}
void motor_SwitchRing(Motor_t *motor,Position_Controller_t *controller)
{   
    if(motor->state_interfaces.angle - motor->command_interfaces.angle > 5.0f)
        motor->command_interfaces.angle += 10.0f;
    else if (motor->state_interfaces.angle - motor->command_interfaces.angle < -5.0f)
        motor->command_interfaces.angle -= 10.0f;
    
    motorPositionControl(motor,controller);
}


static void djiMotorInfo(DJI_Motor_t *motor){
	uint8_t i=0;
	for(i=0;i<id_cnt;i++){
		if(motor_instance[i]->dji == motor){
			motor_instance[i]->state_interfaces.last_angle	= motor_instance[i]->state_interfaces.angle;
			motor_instance[i]->state_interfaces.angle 		= motor->state_interfaces.ecd/8191*360;
			motor_instance[i]->state_interfaces.current	= motor->state_interfaces.given_current;
			motor_instance[i]->state_interfaces.speed_rpm	= motor->state_interfaces.speed_rpm;
			motor_instance[i]->state_interfaces.temperate	= motor->state_interfaces.temperate;
			motor_instance[i]->state_interfaces.torque		= 0;
			
			//转速不可高于45000rpm
			if ((motor_instance[i]->state_interfaces.angle - motor_instance[i]->state_interfaces.last_angle) > 270)
            {
                motor_instance[i]->state_interfaces.rounds--;
            }
            else if ((motor_instance[i]->state_interfaces.angle - motor_instance[i]->state_interfaces.last_angle) < -270)
            {
                motor_instance[i]->state_interfaces.rounds++;
            }
			
            motor_instance[i]->state_interfaces.series_angle = motor_instance[i]->state_interfaces.angle  + 360* motor_instance[i]->state_interfaces.rounds;
			
			
			break;
		}
	}
}

static void dmMotorInfo(DM_Motor_t *motor){
	uint8_t i=0;
	for(i=0;i<id_cnt;i++){
		if(motor_instance[i]->dm == motor){
			motor_instance[i]->state_interfaces.last_angle	= motor_instance[i]->state_interfaces.angle;
			motor_instance[i]->state_interfaces.angle 		= loop_float_constrain(motor->state_interfaces.position/PI*180,0,360);
			
			
			
			motor_instance[i]->state_interfaces.current	= 0;
			motor_instance[i]->state_interfaces.speed_rpm	= motor->state_interfaces.velocity/PI*60;
			motor_instance[i]->state_interfaces.temperate	= motor->state_interfaces.t_mos;
			motor_instance[i]->state_interfaces.torque		= motor->state_interfaces.torque;
			
			if ((motor_instance[i]->state_interfaces.angle - motor_instance[i]->state_interfaces.last_angle) > 270)
            {
                motor_instance[i]->state_interfaces.rounds--;
            }
            else if ((motor_instance[i]->state_interfaces.angle - motor_instance[i]->state_interfaces.last_angle) < -270)
            {
                motor_instance[i]->state_interfaces.rounds++;
            }
			
            motor_instance[i]->state_interfaces.series_angle = motor_instance[i]->state_interfaces.angle  + 360* motor_instance[i]->state_interfaces.rounds;
			
			
			break;
		}
	}
}
