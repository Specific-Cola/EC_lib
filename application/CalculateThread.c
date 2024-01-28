#include "main.h"
#include "CalculateThread.h"
#include "cmsis_os.h"
#include "RM_remote.h"
#include "Motor.h"

#include <string.h>


//DJI_Motor_t *my_motor;
//DM_Motor_t *dm_motor;

Motor_t *my_motor;
Motor_t *dm_motor;


//Speed_Controller_t *my_controller;
//PID_Init_Config_s my_pid;
Position_Controller_t *my_controller;
cascade_PID_Init_Config_s my_pid;
RM_Remote_t *my_remote;


void CalculateThread(void const * argument)
{
	osDelay(1000);
	
//	DJI_Motor_Register_t motor_reg;
	Motor_Register_t motor_reg;
	motor_reg.motor_type=DJI_MOTOR_6020;
	motor_reg.dji_motor_set.hcan=&hcan1;
	motor_reg.dji_motor_set.id=1;
	my_motor = motorAdd(&motor_reg);
	
//	memset(&my_pid,0,sizeof(PID_Init_Config_s));
//	my_pid.Kp = 20;
//	my_pid.Ki = 0;
//	my_pid.Kd = 0;
//	my_pid.MaxOut = 0x7fff;
//	my_pid.Improve = PID_IMPROVE_NONE;
//	my_controller = speedControllerInit(&my_pid);]
	memset(&my_pid,0,sizeof(cascade_PID_Init_Config_s));
	my_pid.out_pid_config.Kp = 20;
	my_pid.out_pid_config.Ki = 0;
	my_pid.out_pid_config.Kd = 0;
	my_pid.out_pid_config.MaxOut = 0x7fff;
	my_pid.out_pid_config.Improve = PID_IMPROVE_NONE;
	my_pid.in_pid_config.Kp = 10;
	my_pid.in_pid_config.Ki = 0;
	my_pid.in_pid_config.Kd = 0;
	my_pid.in_pid_config.MaxOut = 0x7fff;
	my_pid.in_pid_config.Improve = PID_IMPROVE_NONE;

	my_controller = positionControllerInit(&my_pid);
	
//	DM_Motor_Register_t dm_motor_reg;
	memset(&motor_reg,0,sizeof(motor_reg));
	
	motor_reg.motor_type=DM_MOTOR_4310;
	motor_reg.dm_motor_set.hcan=&hcan1;
	motor_reg.dm_motor_set.kd_max=5;
	motor_reg.dm_motor_set.kd_min=0;
	motor_reg.dm_motor_set.kp_max=500;
	motor_reg.dm_motor_set.kp_min=0;
	motor_reg.dm_motor_set.p_max=12.5;
	motor_reg.dm_motor_set.v_max=30;
	motor_reg.dm_motor_set.t_max=10;
	motor_reg.dm_motor_set.rx_id=0x01E;
	motor_reg.dm_motor_set.tx_id=0x10E;
	
	dm_motor = motorAdd(&motor_reg);
	
	my_remote = rmRemoteAdd(&huart3);
	while(1)
	{
		// djiMotor_SwitchRing(my_motor,my_controller);
		my_motor->command_interfaces.angle += my_remote->state_interfaces.rc.ch[0]/660.0;
		djiMotorPositionControl(my_motor,my_controller);
		my_motor->command_interfaces.speed_rpm = my_remote->state_interfaces.rc.ch[0];
//		my_motor->command_interfaces.command=2000;
//		djiMotorSpeedControl(my_motor,my_controller);

		
		djiMotorSendMessage();
		dmMotorSendMessage(dm_motor->motor.dm);
		osDelay(1);
	}

}