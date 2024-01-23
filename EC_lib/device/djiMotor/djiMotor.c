//=====================================================================================================
// djiMotor.c
//=====================================================================================================
//
//       IRobot  EC_lib
//
// author: @ dji
// modify: @ Specific_Cola
// 
//
//=====================================================================================================
#include "djiMotor.h"
#include "main.h"       
#include "bsp_can.h"

 DJI_Motor_t *dji_motor[MAX_DJI_MOTOR_NUM];
static uint8_t id_cnt; //记录大疆电机数量
static CAN_TxHeaderTypeDef  can_tx_message;
static uint8_t   MotorSendBuffer_can1[24];
static uint8_t   MotorSendBuffer_can2[24];
static uint32_t             send_mail_box_can1;
static uint32_t             send_mail_box_can2;


DJI_Motor_t *djiMotorAdd(DJI_Motor_Register_t *reg)//使用can instance注册电机    //id就直接填写灯电调闪烁的次数
{
    if (id_cnt > MAX_DJI_MOTOR_NUM) 
    {
        Error_Handler();//电机太多了
    }
    
    
    
    Can_Register_t can_reg;
    DJI_Motor_t *motor = (DJI_Motor_t *)malloc(sizeof(DJI_Motor_t));
    memset(&can_reg, 0, sizeof(Can_Register_t));
    memset(motor, 0, sizeof(DJI_Motor_t));
	
    can_reg.can_handle = reg->hcan;
    can_reg.tx_dlc = 8; 
    can_reg.can_device_callback = djiMotorCallback;
	
    switch (reg->motor_type)
    {
    case MOTOR_6020:
        if (reg->id <= 4)
            can_reg.tx_id = 0x1FF;
        else if (reg->id > 4)
            can_reg.tx_id = 0x2FF;
        can_reg.rx_id = 0x204 + reg->id;
        break;
    case MOTOR_3508:
        if (reg->id <= 4)
            can_reg.tx_id = 0x200;
        else if (reg->id > 4)
            can_reg.tx_id = 0x1FF;
        can_reg.rx_id = 0x200 + reg->id;
        break;
    case MOTOR_2006:
        if (reg->id <= 4)
            can_reg.tx_id = 0x200;
        else if (reg->id > 4)
            can_reg.tx_id = 0x1FF;
        can_reg.rx_id = 0x200 + reg->id;
        break;
    default:
        Error_Handler();//电机类型不存在
        break;
    }
    for(uint8_t i = 0; i < id_cnt; i++)
    {
        if(dji_motor[i]->can_info->can_handle == reg->hcan && dji_motor[i]->can_info->rx_id == can_reg.rx_id)
        {
            Error_Handler();//电机id冲突
        }
    }
    
    motor->can_info = canDeviceRegister(&can_reg);
    motor->motor_type = reg->motor_type;
    dji_motor[id_cnt++] = motor;

    return motor;   
}

Return_t djiMotorSendMessage()//不能使用bsp_can里面的发送函数，因为dji电机是广播模式
{   
	
	int16_t can_send_num[2][3]={{-1,-1,-1},{-1,-1,-1}};
	
    for(uint8_t i =0;i<id_cnt;i++)
    {
        if(dji_motor[i]->statu == Offline)
        {
            dji_motor[i]->command_interfaces.command = 0;
        }


        if(dji_motor[i]->can_info->can_handle == &hcan1)
        {
            MotorSendBuffer_can1[(dji_motor[i]->can_info->rx_id - 0x201)*2 ] = dji_motor[i]->command_interfaces.command >> 8;
            MotorSendBuffer_can1[(dji_motor[i]->can_info->rx_id - 0x201)*2 + 1] = dji_motor[i]->command_interfaces.command;
			
			switch(dji_motor[i]->can_info->tx_id){
				case 0x200:
					can_send_num[0][0]=i;
				break;
				case 0x1FF:
					can_send_num[0][1]=i;
				break;
				case 0x2FF:
					can_send_num[0][2]=i;
				break;
			}
        }
        else if (dji_motor[i]->can_info->can_handle == &hcan2)
        {
            MotorSendBuffer_can2[(dji_motor[i]->can_info->rx_id - 0x201)*2 ] = dji_motor[i]->command_interfaces.command >> 8;
            MotorSendBuffer_can2[(dji_motor[i]->can_info->rx_id - 0x201)*2 + 1] = dji_motor[i]->command_interfaces.command;
			
			switch(dji_motor[i]->can_info->tx_id){
				case 0x200:
					can_send_num[1][0]=i;
				break;
				case 0x1FF:
					can_send_num[1][1]=i;
				break;
				case 0x2FF:
					can_send_num[1][2]=i;
				break;
			}
        }
    }
	
	if(can_send_num[0][0]>=0){
		canSendMessage(dji_motor[can_send_num[0][0]]->can_info,MotorSendBuffer_can1);
	}
	if(can_send_num[0][1]>=0){
		canSendMessage(dji_motor[can_send_num[0][1]]->can_info,MotorSendBuffer_can1+8);
	}
	if(can_send_num[0][2]>=0){
		canSendMessage(dji_motor[can_send_num[0][2]]->can_info,MotorSendBuffer_can1+16);
	}
	
	if(can_send_num[1][0]>=0){
		canSendMessage(dji_motor[can_send_num[1][0]]->can_info,MotorSendBuffer_can2);
	}
	if(can_send_num[1][1]>=0){
		canSendMessage(dji_motor[can_send_num[1][1]]->can_info,MotorSendBuffer_can2+8);
	}
	if(can_send_num[1][2]>=0){
		canSendMessage(dji_motor[can_send_num[1][2]]->can_info,MotorSendBuffer_can2+16);
	}
	
    return RETURN_SUCCESS;
}


void djiMotorCallback(Can_Device_t *instance)
{
    for(uint8_t i = 0; i < id_cnt;i++)
    {
        if(instance->rx_id == dji_motor[i]->can_info->rx_id)
        {
            djiMotorInfoUpdate(dji_motor[i],instance->rx_buff);
            return;
        }
    }     
}

void djiMotorInfoUpdate(DJI_Motor_t *motor,uint8_t *data)
{
	
    motor->state_interfaces.last_ecd = motor->state_interfaces.ecd;
    motor->state_interfaces.ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
    motor->state_interfaces.speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);
    motor->state_interfaces.given_current = (uint16_t)((data)[4] << 8 | (data)[5]);
    motor->state_interfaces.temperate = (data)[6];
}

void djiMotorSpeedControl(DJI_Motor_t *motor,Speed_Controller_t *controller)
{
//    motor->command_interfaces.command =  (int16_t)PIDCalculate(controller->pid, motor->state_interfaces.speed_rpm,motor->command_interfaces.speed_rpm);
	PIDCalculate(controller->pid, motor->state_interfaces.speed_rpm,motor->command_interfaces.speed_rpm);
	motor->command_interfaces.command = controller->pid->DWT_CNT;
//	motor->command_interfaces.command = controller->pid->Improve;
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