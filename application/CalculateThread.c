#include "CalculateThread.h"
#include "AttitudeThread.h"
#include "InterruptService.h"
#include "Remote.h"
#include "AimbotCan.h"
#include "user_lib.h"
#include "pid.h"
#include "Motor.h"
#include "RefereeCan.h"
#include "tim.h"
#include "bsp_can.h"
//#include "stdio.h"
#include "loop_fifo.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>
#include "Infantry4KeyMap.h"
#include "Setting.h"
#include PARAMETER_FILE
#include KEYMAP_FILE

//#define printf(...)  HAL_UART_Transmit_DMA(&huart6,\
//																				(uint8_t  *)u1_buf,\
//																				sprintf((char*)u1_buf,__VA_ARGS__))
//uint8_t u1_buf[30];

Gimbal_t                Gimbal;//云台状态结构
Chassis_t               Chassis;//底盘状态
RC_ctrl_t               Remote;//遥控器数据
AimbotCommand_t         Aimbot;//自瞄数据
OfflineMonitor_t        Offline;//离线检测结构体
RefereeInformation_t    Referee;//裁判系统数据
GimbalRequestState_t RequestStatePacket;//云台请求can包

first_order_filter_type_t  pitch_aimbot_filter;
fp32 pitch_aimbot_filter_param = 0.10f;



void GimbalStateMachineUpdate(void);
void ChassisStateMachineUpdate(void);
void GimbalControlModeUpdate(void);
void GimbalFireModeUpdate(void);
void SetGimbalDisable(void);
void GimbalPIDUpdate(void);
void RotorPIDUpdate(void);
void AmmoPIDUpdate(void);
void GimbalMeasureUpdate(void);
void GimbalCommandUpdate(void);
void ChassisCommandUpdate(void);
void RotorCommandUpdate(void);
void AmmoCommandUpdate(void);
void DebugLEDShow(void);
void GimbalRequestStatePacketSend(void);
void BoomBayCover(void);


//int dafu_flag = 0;


bool_t single_shoot_flag=0;//单发开关
bool_t auto_fire_flag=1;//自动开火开关
bool_t switch_flag=0;//打符切换开关
int16_t dealta_heat=0;
int32_t onelasttime=0;
int16_t onelastheat=0;
uint16_t count=0;

int32_t    gimbal_init_countdown = 0;          //  云台初始化倒计时器
int32_t    gimbal_fire_countdown = 0;          //  云台射击拨盘转动倒计时器
int32_t    gimbal_lagging_counter = 0;         //  云台堵转计数器




fp32 LimitNormalization(fp32 input);
extern ImuPacketNormal_t ImuPacket;
extern ImuPacketMini_t ImuPackageMini;
int16_t minus = 0;


void CalculateThread(void const * pvParameters)
{
	
	uint16_t control_counter = 0;
	
    osDelay(500);
    PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_10MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);//左右摩擦轮pid初始化
    PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_10MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    LoopFifoFp32_init(&Gimbal.ImuBuffer.YawLoopPointer, Gimbal.ImuBuffer.YawAddress, 64);//自瞄数据fifo初始化
    LoopFifoFp32_init(&Gimbal.ImuBuffer.PitchLoopPointer, Gimbal.ImuBuffer.PitchAddress, 64);
    first_order_filter_init(&pitch_aimbot_filter, 1000, &pitch_aimbot_filter_param);//滤波器初始化

    while(1)
    {
        Remote = *get_remote_control_point();//更新遥控器数据
        GetAimbotCommand(&Aimbot);//获取自瞄指令
        GetRefereeInformation(&Referee);//获取裁判系统信息 包括枪口的限制
        DeviceOfflineMonitorUpdate(&Offline);//获取模块离线信息
        
        LoopFifoFp32_push(&Gimbal.ImuBuffer.YawLoopPointer, Gimbal.Imu.YawAngle);
        LoopFifoFp32_push(&Gimbal.ImuBuffer.PitchLoopPointer, Gimbal.Imu.PitchAngle);//陀螺仪数据入栈
        
        GimbalStateMachineUpdate();//根据遥控器拨杆决定当前状态（无力，初始化，测试，比赛）
        ChassisStateMachineUpdate();//底盘状态改变
        GimbalControlModeUpdate();//控制权
        GimbalFireModeUpdate();//开火状态转换
        GimbalPIDUpdate();//云台pid重装载
        RotorPIDUpdate();//拨盘pid重装载
        AmmoPIDUpdate();//射击pid重装载
        GimbalMeasureUpdate();//获取电机和imu数据
        GimbalCommandUpdate();//指令的转换
        ChassisCommandUpdate();//底盘指令转换
        RotorCommandUpdate();//拨盘控制转换
        AmmoCommandUpdate();//发射部分控制转化
		
		if(control_counter > 10)
		{
			control_counter = 0;
			GimbalRequestStatePacketSend();//云台指令下发
		}
		control_counter+=1;
			
        
		
		
        DebugLEDShow();
        BoomBayCover();//弹舱盖开合
				
				minus = Aimbot.CommandTimer - ImuPacket.TimeStamp;


				GimbalMotorControl( Gimbal.Output.Yaw * YAW_MOTOR_DIRECTION ,
                            Gimbal.Output.Pitch * PITCH_MOTOR_DIRECTION, 
                            Gimbal.Output.Rotor, //Gimbal.Output.Rotor
                            Gimbal.Output.AmmoLeft, 
                            Gimbal.Output.AmmoRight
                        );
				osDelay(1);
    }
}

void GimbalStateMachineUpdate(void)
{
		// 电机离线保护
    if(Offline.PitchMotor==DEVICE_OFFLINE||Offline.YawMotor == DEVICE_OFFLINE)
		{
        if(Gimbal.StateMachine!=GM_NO_FORCE)
						Gimbal.StateMachine = GM_NO_FORCE;
        return;
    }
    // 遥控器离线保护
    if(Offline.Remote==DEVICE_OFFLINE)
		{
        if(Gimbal.StateMachine!=GM_NO_FORCE)
						Gimbal.StateMachine=GM_NO_FORCE;
        return;
    }
    
    // 云台状态机
    switch (Remote.rc.s[0])
		{
        // 右拨杆打到最上，云台复位后进入比赛模式，该模式下开摩擦轮
        case RC_SW_UP:
            if (Gimbal.StateMachine == GM_NO_FORCE)
						{
                Gimbal.StateMachine = GM_INIT;
                gimbal_init_countdown = 800;
            }
            else if (Gimbal.StateMachine == GM_INIT)
            {
                if (gimbal_init_countdown > 0){
                    gimbal_init_countdown--;
                }
                else{
                    Gimbal.StateMachine = GM_MATCH;//比赛模式
                }
            }
            else{
                Gimbal.StateMachine = GM_MATCH;
            }
            break;
        
        // 右拨杆打到中间，云台复位后进入调试模式
        case RC_SW_MID:
            if (Gimbal.StateMachine == GM_NO_FORCE){
                Gimbal.StateMachine = GM_INIT;
                gimbal_init_countdown = 800;
            }
            else if (Gimbal.StateMachine == GM_INIT)
            {
                if (gimbal_init_countdown > 0){
                    gimbal_init_countdown--;
                }
                else{
                    Gimbal.StateMachine = GM_TEST;
                }
            }
            else{
                Gimbal.StateMachine = GM_TEST;
            }
            break;
            
        // 右拨杆打到最下，或遥控器数据出错，云台进入无力模式
        case RC_SW_DOWN:
            if (Gimbal.StateMachine != GM_NO_FORCE){
                Gimbal.StateMachine = GM_NO_FORCE;
            }
            break;
        default:
            if (Gimbal.StateMachine != GM_NO_FORCE){
                Gimbal.StateMachine = GM_NO_FORCE;
            }
            break;
    }
}
uint8_t rote_flag = 0;
void ChassisStateMachineUpdate(void)
{
	if(CheakKeyPressOnce(KEY_PRESSED_OFFSET_F)) 
        rote_flag=(rote_flag+1)%2;   
	
    //if ((Gimbal.StateMachine == GM_NO_FORCE)  ||  (Gimbal.StateMachine == GM_INIT)) {
		if((Gimbal.StateMachine==GM_NO_FORCE))
        Chassis.ChassisState=CHASSIS_NO_FORCE;//云台无力，底盘强制进入无力状态
		if(Gimbal.StateMachine==GM_INIT)
		{
			 if(Remote.rc.s[1]==2)
					Chassis.ChassisState=CHASSIS_FOLLOW;
			 else
					Chassis.ChassisState=CHASSIS_NO_FORCE;
		}
    if(Gimbal.StateMachine==GM_TEST||Gimbal.StateMachine==GM_MATCH)
		{
        if(Remote.rc.s[1]==2/*||Remote.rc.s[1]==1*/)
				{//左侧拨杆在最下面是底盘有力
            if((rote_flag == 1) || (RemoteDial() == -1.0f))//小陀螺模式
                Chassis.ChassisState=CHASSIS_ROTATE;
						else 
								if(CHASSIS_STOP_KEYMAP)
										Chassis.ChassisState=CHASSIS_NO_MOVE;
								else
										Chassis.ChassisState=CHASSIS_FOLLOW;												
            if(CHASSIS_HIGH_SPEED_KEYMAP)
                Chassis.ChassisSpeed=CHASSIS_FAST_SPEED;
            else
                Chassis.ChassisSpeed=CHASSIS_NORMAL_SPEED;
        }
        else
            Chassis.ChassisState=CHASSIS_NO_FORCE;
    }
}
void SetGimbalDisable(void)
{
    Gimbal.StateMachine = GM_NO_FORCE;
    Gimbal.ControlMode = GM_NO_CONTROL;
    Gimbal.FireMode = GM_FIRE_UNABLE;
}


void GimbalControlModeUpdate(void)
{
    // 比赛模式下
    if(Gimbal.StateMachine==GM_MATCH||Gimbal.StateMachine==GM_TEST)
		{
        // 如果按下鼠标右键并且视觉发现目标，进入自瞄控制
        if(((Remote.mouse.press_r==PRESS)||(Remote.rc.s[1]==RC_SW_UP))&&(Offline.AimbotStateNode == DEVICE_ONLINE)&&(Offline.AimbotDataNode == DEVICE_ONLINE)/*&&(Aimbot.State&AIMBOT_TARGET_INSIDE_OFFSET)*/)
				{
            if(single_shoot_flag)
								Gimbal.ControlMode = GM_AIMBOT_RUNES;
						else
								Gimbal.ControlMode = GM_AIMBOT_OPERATE;
				}
        else
            Gimbal.ControlMode = GM_MANUAL_OPERATE;//手动状态
    }	
    if(Gimbal.StateMachine==GM_INIT)
        Gimbal.ControlMode=GM_RESET_POSITION;
    if(Gimbal.StateMachine==GM_NO_CONTROL)
        Gimbal.ControlMode=GM_NO_CONTROL;
}


// qylann: 以下代码太乱
uint8_t big_rune_flag = 0;
uint8_t small_rune_flag = 0;
extern GimbalRequestState_t RequestStatePacket;
void GimbalFireModeUpdate(void)
{		
	
	
    //自动开火开关
	  if(FIRE_MODE_KEYMAP) 
        auto_fire_flag=(auto_fire_flag+1)%2;   
	
		//单发开关
	  if( big_rune_flag|| small_rune_flag)
		  single_shoot_flag = 1;
	  else
		{
			if(SINGLE_SHOOT_KEMAP)
			single_shoot_flag=(single_shoot_flag+1)%2;			
		}	
    
		dealta_heat=Referee.Ammo0Limit.Heat-Referee.Realtime.Ammo0Heat;
		if(GetSystemTimer()-onelasttime>=1000)
		{
				onelasttime=GetSystemTimer(),
				onelastheat=dealta_heat,
				count=0; 
		}

		if(Gimbal.StateMachine!=GM_MATCH)
		{
				Gimbal.FireMode=GM_FIRE_UNABLE;
				gimbal_fire_countdown=0;
		}
    if(Gimbal.StateMachine==GM_MATCH) 
		{
        if(Gimbal.FireMode==GM_FIRE_UNABLE)
            Gimbal.FireMode=GM_FIRE_READY;
        if (Gimbal.FireMode==GM_FIRE_READY) 
				{						
						if((SHOOT_COMMAND_KEYMAP)//收到操作手发弹指令
							&&(((Gimbal.ControlMode==GM_AIMBOT_OPERATE||Gimbal.ControlMode==GM_AIMBOT_RUNES)&&((Aimbot.State & 0x02) != 0)&&auto_fire_flag==1)//自动开火
						        ||((Gimbal.ControlMode==GM_AIMBOT_OPERATE||Gimbal.ControlMode==GM_AIMBOT_RUNES)&&auto_fire_flag==0)//无自动开火
								||((Gimbal.ControlMode==GM_MANUAL_OPERATE&&Remote.mouse.press_r!=PRESS)||auto_fire_flag==0))//手动开火
									&&((count*10<=Referee.Ammo0Limit.Cooling+onelastheat&&dealta_heat>10)||Referee.Ammo0Limit.Heat==0xFFFF)	)//且热量闭环允许 
						{
								Gimbal.FireMode=GM_FIRE_BUSY;									
								gimbal_fire_countdown=ROTOR_TIMESET_BUSY; 
								count++;
						}
        }
				if(Gimbal.FireMode==GM_FIRE_BUSY&&gimbal_fire_countdown<=0)
				{
						if(single_shoot_flag==1||Offline.RefereeAmmoLimitNode0==1)
								gimbal_fire_countdown=450;
						else 
								gimbal_fire_countdown=(int)(10000.0/(dealta_heat/1.7+Referee.Ammo0Limit.Cooling/2.0+5)-45);
						Gimbal.FireMode=GM_FIRE_COOLING;
				}

				if(Gimbal.FireMode==GM_FIRE_COOLING&&gimbal_fire_countdown<=0) 
						Gimbal.FireMode=GM_FIRE_READY;    
				
        //  异常射击模式的状态机，用于反堵转
				if(Gimbal.FireMode==GM_FIRE_LAGGING)
				{
						if(gimbal_fire_countdown<=0)
								Gimbal.FireMode=GM_FIRE_READY;  
				}
				else
				{
						if ((Gimbal.FireMode == GM_FIRE_BUSY)&&(Gimbal.MotorMeasure.ShootMotor.RotorMotorSpeed<400))
								gimbal_lagging_counter++;
						else
								gimbal_lagging_counter=0;      
						if (gimbal_lagging_counter>ROTOR_LAGGING_COUNTER_MAX)//ROTOR_LAGGING_COUNTER_MAX
						{        
								gimbal_lagging_counter=0;
								gimbal_fire_countdown=ROTOR_TIMESET_RESERVE;
								Gimbal.FireMode=GM_FIRE_LAGGING;
						}
				}				
				gimbal_fire_countdown--;
		}
}

// qylann: "     "


GimbalControlMode_e CMthis = GM_NO_CONTROL;
GimbalControlMode_e CMlast = GM_NO_CONTROL;

void GimbalPIDUpdate(void)
{
    CMthis = Gimbal.ControlMode;
    
    if (CMthis == CMlast){
        return;
    }
    
    
    //  
    
    if (CMthis == GM_MANUAL_OPERATE){
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_MANUAL_OPERATE, 
                            YAW_SPEED_MANUAL_OPERATE, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_MANUAL_OPERATE, 
                            PITCH_SPEED_MANUAL_OPERATE, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }
    else if (CMthis == GM_AIMBOT_OPERATE){
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_AIMBOT_OPERATE, 
                            YAW_SPEED_AIMBOT_OPERATE, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_AIMBOT_OPERATE, 
                            PITCH_SPEED_AIMBOT_OPERATE, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }
    else if (CMthis == GM_AIMBOT_RUNES){
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_AIMBOT_RUNES, 
                            YAW_SPEED_AIMBOT_RUNES, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_AIMBOT_RUNES, 
                            PITCH_SPEED_AIMBOT_RUNES, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }
    else if (CMthis == GM_RESET_POSITION){
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_RESET_POSITION, 
                            YAW_SPEED_RESET_POSITION, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_RESET_POSITION, 
                            PITCH_SPEED_RESET_POSITION, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }
    else{
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_NO_FORCE, 
                            YAW_SPEED_NO_FORCE, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_NO_FORCE, 
                            PITCH_SPEED_NO_FORCE, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }
    
    CMlast = CMthis;
}





GimbalFireMode_e FMthis = GM_FIRE_UNABLE;
GimbalFireMode_e FMlast = GM_FIRE_UNABLE;
void RotorPIDUpdate(void)
{
    FMthis = Gimbal.FireMode;
    
    if (FMthis == FMlast){
        return;
    }
    
    //
    
    if ((FMthis == GM_FIRE_READY)  ||  (FMthis == GM_FIRE_COOLING)){
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_STOP, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }
    else if (FMthis == GM_FIRE_BUSY){
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_FORWARD, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }
    else if (FMthis == GM_FIRE_LAGGING){
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_BACK, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }
    else{
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_UNABLE, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }
    
    FMlast = FMthis;
}


uint8_t MSthis = 0;
uint8_t MSlast = 0;
void AmmoPIDUpdate(void)
{
    MSthis = Referee.Ammo0Limit.Speed;//Gimbal.Referee.MaxSpeed;
    
    if (MSthis != MSlast){
        switch (MSthis){
            case 10:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_10MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_10MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 12:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_12MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_12MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 14:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_14MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_14MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 15:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_15MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_15MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 16:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_16MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_16MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 18:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_18MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_18MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 22:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_22MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_22MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 30:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            default:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, DEFAULT_AMMOL_PID, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, DEFAULT_AMMOR_PID, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
        }
    }
    
    MSlast = MSthis;
}


void GimbalMeasureUpdate(void)
{
    GimbalMotorMeasureUpdate(&Gimbal.MotorMeasure.GimbalMotor);
    ShootMotorMeasureUpdate(&Gimbal.MotorMeasure.ShootMotor);
    GimbalEulerSystemMeasureUpdate(&Gimbal.Imu);
}



int t=8;

fp32 aimbot_pitch_bias = 0;

void GimbalCommandUpdate(void)
{
	if (AIMBOT_PITCH_BIAS_LOW_KEYMAP) {
		aimbot_pitch_bias += 0.3;
	}
	else if (AIMBOT_PITCH_BIAS_HIGH_KEYMAP) {
		aimbot_pitch_bias -= 0.3;
	}
	else if (AIMBOT_PITCH_BIAS_ZERO_KEYMAP) {
		aimbot_pitch_bias = 0;
	}
	
	if (aimbot_pitch_bias > 6) {
		aimbot_pitch_bias = 6;
	}
	else if (aimbot_pitch_bias < -6) {
		aimbot_pitch_bias = -6;
	}
	
    if (Gimbal.ControlMode == GM_MANUAL_OPERATE){
        Gimbal.Command.Yaw += GIMBAL_CMD_YAW_KEYMAP;
        Gimbal.Command.Pitch += GIMBAL_CMD_PITCH_KEYMAP;
        Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
        Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
        Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle-Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*0.0142 , Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
        pitch_aimbot_filter.out = Gimbal.Command.Pitch;
    }
    else if (Gimbal.ControlMode == GM_AIMBOT_OPERATE || (Gimbal.ControlMode == GM_AIMBOT_RUNES)){
//        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle + Aimbot.YawRelativeAngle;
//        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle + Aimbot.PitchRelativeAngle;
        Gimbal.Command.Yaw = LoopFifoFp32_read(&Gimbal.ImuBuffer.YawLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer+t)) + Aimbot.YawRelativeAngle;
        Gimbal.Command.Pitch = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer+t)) + Aimbot.PitchRelativeAngle + aimbot_pitch_bias;
//        fp32 pitch_command = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer)) + Aimbot.PitchRelativeAngle;
//        first_order_filter_cali(&pitch_aimbot_filter, pitch_command);
//        Gimbal.Command.Pitch = pitch_aimbot_filter.out;
        Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
        Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
        Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle-Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*0.0142 , Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
    } 
    else if (Gimbal.ControlMode == GM_AIMBOT_RUNES){
        
//        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle + Aimbot.YawRelativeAngle;
//        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle + Aimbot.PitchRelativeAngle;
        Gimbal.Command.Yaw = LoopFifoFp32_read(&Gimbal.ImuBuffer.YawLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer)) + Aimbot.YawRelativeAngle;
        Gimbal.Command.Pitch = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer)) + Aimbot.PitchRelativeAngle + aimbot_pitch_bias;
//        fp32 pitch_command = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer)) + Aimbot.PitchRelativeAngle;
//        first_order_filter_cali(&pitch_aimbot_filter, pitch_command);
//        Gimbal.Command.Pitch = pitch_aimbot_filter.out;
        Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
        Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
        Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle-Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*0.0142 , Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);

    }
    else if (Gimbal.ControlMode == GM_RESET_POSITION){
        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle;
        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle;
        fp32 YawTempCommand = loop_fp32_constrain(YAW_ZERO_ECDANGLE, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle - 180.0f, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle + 180.0f);
//        Gimbal.Output.Yaw = YAW_MOTOR_DIRECTION * cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle, Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed, YAW_ZERO_ECDANGLE);
        Gimbal.Pid.Yaw.v_set = PID_calc(&Gimbal.Pid.Yaw.pid_outside, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle, YawTempCommand);
        Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle-Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*0.0142 , Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, 0);
        pitch_aimbot_filter.out = Gimbal.Command.Pitch;
    }
    else{
        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle;
        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle;
        Gimbal.Output.Yaw = 0;
        Gimbal.Output.Pitch = 0;
        pitch_aimbot_filter.out = Gimbal.Command.Pitch;
    }
}

void ChassisCommandUpdate(void)
{
    if ((Chassis.ChassisState == CHASSIS_NO_FORCE) ) {
        Chassis.ChassisCommandX = 0.0f;
        Chassis.ChassisCommandY = 0.0f;
    }
    else {
        Chassis.ChassisCommandX = CHASSIS_CMD_X_KEYMAP;
        Chassis.ChassisCommandY = CHASSIS_CMD_Y_KEYMAP;
    }
}

void RotorCommandUpdate(void)
{
    if (Gimbal.FireMode == GM_FIRE_BUSY){
        Gimbal.Command.Rotor = ROTOR_SPEEDSET_FORWARD * ROTOR_MOTOR_DIRECTION;
		
    }
    else if (Gimbal.FireMode == GM_FIRE_LAGGING){
        Gimbal.Command.Rotor = ROTOR_SPEEDSET_BACKWARD * ROTOR_MOTOR_DIRECTION;

    }
    else if (Gimbal.FireMode == GM_FIRE_UNABLE){
        Gimbal.Command.Rotor = 0;
        Gimbal.Output.Rotor = 0;
        return;
    }
    else{
        Gimbal.Command.Rotor = 0;
    }
    
    Gimbal.Output.Rotor = PID_calc(&Gimbal.Pid.Rotor, Gimbal.MotorMeasure.ShootMotor.RotorMotorSpeed, Gimbal.Command.Rotor);
}






void AmmoCommandUpdate(void)
{
    if (Gimbal.FireMode == GM_FIRE_UNABLE){
        Gimbal.Command.AmmoLeft = 0;
        Gimbal.Command.AmmoRight = 0;
        Gimbal.Output.AmmoLeft = 0;
        Gimbal.Output.AmmoRight = 0;
        return;
    }
    switch (MSthis){
        case 10:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft,
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_10MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_10MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 12:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_12MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_12MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 14:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_14MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_14MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 15:
			
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_15MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_15MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
			
			
            break;
        
        case 16:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_16MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_16MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 18:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_18MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_18MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 22:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_22MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_22MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 30:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_30MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_30MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        default:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                DEFAULT_AMMO_SPEEDSET * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                DEFAULT_AMMO_SPEEDSET * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
    }
    
}


void GetGimbalMotorOutput(GimbalOutput_t *out)
{
    memcpy(out, &Gimbal.Output, sizeof(GimbalOutput_t));
}

bool_t cover_flag = 0;
void BoomBayCover(void)
{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2000);
		
    if (Gimbal.StateMachine == GM_MATCH) {
        if (cover_flag == 0) {
            cover_flag = 2;
            
        }
        
        if (COVER_SWITCH_KEYMAP) {
            if (cover_flag == 1) {
                cover_flag = 2;
                HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
            }
            else if (cover_flag == 2) {
                cover_flag = 1;
                HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
            }
        }
    }
    else {
        if (SHOOT_COMMAND_KEYMAP) {
              cover_flag = 1;
                HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
        }
		else{
		 cover_flag = 2;
                HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
		}

    }
    
		
   
    if ((cover_flag == 1)  ||  (cover_flag == 0)) {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1400);//四号：2245 三号 1250    打开
     }
    if (cover_flag == 2) {
         __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 600);//四号：500   关闭
    }
    
    
    
}



void GetGimbalRequestState(GimbalRequestState_t *RequestState)
{
	   if (Gimbal.StateMachine == GM_NO_FORCE) {
         RequestState->GimbalState |= (uint8_t)(1 << 0);
    }
	
	
	
    RequestState->AimbotRequest = 0x00;
	
	if(BIG_RUNE_KEYMAP)
	{
		small_rune_flag = 0;
		 big_rune_flag=(big_rune_flag+1)%2; 
	}
	if(SMALL_RUNE_KEYMAP)
	{	
		big_rune_flag = 0;
		 small_rune_flag=(small_rune_flag+1)%2;
	}
	if(big_rune_flag)
		RequestState->AimbotRequest |= (uint8_t)(1 << 5);
	else if(small_rune_flag)		
		RequestState->AimbotRequest |= (uint8_t)(1 << 4);
	else
		RequestState->AimbotRequest |= (uint8_t) (1 << 0);
    
    
        
    
    
    
    RequestState->ChassisMoveXRequest = Chassis.ChassisCommandX * 32767;
    RequestState->ChassisMoveYRequest = Chassis.ChassisCommandY * 32767;
    RequestState->ChassisStateRequest = 0x00;

    
    
    if (Chassis.ChassisState != CHASSIS_NO_FORCE) {
        RequestState->ChassisStateRequest |= (uint8_t)(1 << 1);
        // 运动状态
        if (Chassis.ChassisState == CHASSIS_NO_MOVE) {
            RequestState->ChassisStateRequest |= (uint8_t)(1 << 2);
        }
        else if (Chassis.ChassisState == CHASSIS_FOLLOW) {
            RequestState->ChassisStateRequest |= (uint8_t)(1 << 3);
        }
        else if (Chassis.ChassisState == CHASSIS_ROTATE) {
            RequestState->ChassisStateRequest |= (uint8_t)(1 << 4);
        }
       
				if(CHASSIS_HIGH_SPEED_KEYMAP){
					RequestState->ChassisStateRequest |= (uint8_t)(1<<5);
				}
				if(CHASSIS_STOP_KEYMAP){
					RequestState->ChassisStateRequest |= (uint8_t)(1<<2);
				}
        
//        if (Chassis.ChassisSpeed = CHASSIS_NORMAL_SPEED) {
//            RequestState->ChassisStateRequest |= (uint8_t)(1 << 5);
//        }
//        else if (Chassis.ChassisSpeed == CHASSIS_FAST_SPEED) {
//            RequestState->ChassisStateRequest |= (uint8_t)(1 << 6);
//        }
//        else if (Chassis.ChassisSpeed == CHASSIS_LOW_SPEED) {
//            RequestState->ChassisStateRequest |= (uint8_t)(1 << 7);
//        }
    }
    else {
        RequestState->ChassisStateRequest |= (uint8_t)(1 << 0);
    }
    
    RequestState->GimbalState = 0x00;
    
    
    
    if ((Remote.mouse.press_r == PRESS)  ||  (Remote.rc.s[1] == RC_SW_UP)) {
        RequestState->GimbalState |= (uint8_t) (1 << 1);
    }
    
    if (cover_flag == 2) {
        RequestState->GimbalState |= (uint8_t) (1 << 3);
    }
    else if (cover_flag == 1) {
        RequestState->GimbalState |= (uint8_t) (1 << 4);
    }
	
		if(auto_fire_flag == 1){
			RequestState->GimbalState |= (uint8_t) (1 << 6);
		}
		
		
    
        
    
    
    RequestState->Reserve = 0x00;
    
}



void DebugLEDShow(void)
{
    if ((Offline.AimbotStateNode == DEVICE_ONLINE)  &&  (Offline.AimbotDataNode == DEVICE_ONLINE)){
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
        if ((Aimbot.State & AIMBOT_TARGET_INSIDE_OFFSET)){
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
        }
        else{
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        }
    }
    else{
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    }
    
    
}


fp32 LimitNormalization(fp32 input)
{
    if (input > 1.0f){
        return 1.0f;
    }
    else if (input < -1.0f){
        return -1.0f;
    }
    else{
        return input;
    }
}



void RefereeHeatInterpolation(void)
{
    Referee.Realtime.Ammo0Heat -= Referee.Ammo0Limit.Cooling / 10;
    if (Referee.Realtime.Ammo0Heat < 0) {
        Referee.Realtime.Ammo0Heat = 0;
    }
}


void GimbalRequestStatePacketSend(void)
{
    GetGimbalRequestState(&RequestStatePacket);
    CanSendMessage(&COMMUNICATE_CANPORT, GINBAL_REQUEST_STATE_ID, 8, (uint8_t *)&RequestStatePacket);
}

