#include "main.h"
#include "CalculateThread.h"
#include "cmsis_os.h"
#include "djiMotor.h"
#include "RM_remote.h"

#include <string.h>


DJI_Motor_t *my_motor;

//Speed_Controller_t *my_controller;
//PID_Init_Config_s my_pid;
Position_Controller_t *my_controller;
cascade_PID_Init_Config_s my_pid;
RM_Remote_t *my_remote;


void CalculateThread(void const * argument)
{
	DJI_Motor_Register_t motor_reg;
	motor_reg.hcan=&hcan1;
	motor_reg.motor_type=MOTOR_6020;
	motor_reg.id=1;
	
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

	my_motor = djiMotorAdd(&motor_reg);
	my_controller = positionControllerInit(&my_pid,&my_motor->state_interfaces.angle);
			//my_motor->command_interfaces.command=200;

	my_remote = rmRemoteAdd(&huart3);
	while(1)
	{
		// djiMotor_SwitchRing(my_motor,my_controller);
		my_motor->command_interfaces.angle += my_remote->state_interfaces.rc.ch[0]/660.0;
		djiMotorPositionControl(my_motor,my_controller);


		
		djiMotorSendMessage();
		osDelay(1);
	}

}