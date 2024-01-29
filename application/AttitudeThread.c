#include "AttitudeThread.h"
#include "BMI088driver.h"
#include "controller.h"
#include "MahonyAHRS.h"
#include "user_lib.h"

#include "CanPacket.h"
#include "Setting.h"

#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"

#include <math.h>
#include <string.h>




#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm����


/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          ����bmi088���¶�
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_temp_control(fp32 temp);

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����imu_update_flag��ֵ����SPI DMA
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_cmd_spi_dma(void);


void AHRS_init(fp32 quat[4], fp32 accel[3]);
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3]);
void get_angle(fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

bmi088_real_data_t bmi088_real_data;


static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;


fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.ŷ���� ��λ rad
fp32 INS_palstance[3] = {0.0f, 0.0f, 0.0f};

uint32_t IMU_Timer;

const fp32 *get_gyro_data_point(void);
const fp32 *get_INS_angle_point(void);
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU);
void GetCurrentQuaternion(fp32 q[4]);


/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu����, ��ʼ�� bmi088, ist8310, ����ŷ����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void AttitudeThread(void const *pvParameters)
{
    //wait a time
    osDelay(INS_TASK_INIT_TIME);
    while(BMI088_init())
    {
        osDelay(100);
    }

    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);

    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);

    AHRS_init(INS_quat, bmi088_real_data.accel);

    //set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }


    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;//����Ҫ��ʼ��DMA֮�����ȥʹ��DMA����������ֻ��һ��DMA�ж�������˴�����

    while (1)
    {
        AHRS_update(INS_quat, 0.001f, bmi088_real_data.gyro, bmi088_real_data.accel);
        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
//        IMU_Timer = GetSystemTimer();
//        CanSendMessage(&COMMUNICATE_CANPORT, IMU_PACKET_TIME_ID, 4, (uint8_t *)&IMU_Timer);
//        CanSendMessage(&COMMUNICATE_CANPORT, IMU_PACKET_DATA0_ID, 8, (uint8_t *)&INS_quat[0]);
//        CanSendMessage(&COMMUNICATE_CANPORT, IMU_PACKET_DATA1_ID, 8, (uint8_t *)&INS_quat[2]);
        osDelay(1);//����Ƶ��ֻ��Ҫ������̨����Ƶ�ʼ���
    }
}

void AHRS_init(fp32 quat[4], fp32 accel[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

const fp32 *get_gyro_data_point(void)
{
    return bmi088_real_data.gyro;
}
void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}
const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}
void GetCurrentQuaternion(fp32 q[4])
{
    memcpy(q, INS_quat, sizeof(INS_quat));
}


#ifdef IMU_DIRECTION_xyz_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2] - IMU_GYRO_YAW_BIAS, accel[0], accel[1], accel[2], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = INS_palstance[0];
    IMU->PitchSpeed = INS_palstance[1];
    IMU->YawSpeed = INS_palstance[2];
}
#endif
#ifdef IMU_DIRECTION_yrxz_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, gyro[1], -gyro[0], gyro[2] - IMU_GYRO_YAW_BIAS, accel[1], -accel[0], accel[2], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = INS_palstance[1];
    IMU->PitchSpeed = -INS_palstance[0];
    IMU->YawSpeed = INS_palstance[2];
}
#endif
#ifdef IMU_DIRECTION_rxryz_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, -gyro[0], -gyro[1], gyro[2] - IMU_GYRO_YAW_BIAS, -accel[0], -accel[1], accel[2], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = -INS_palstance[0];
    IMU->PitchSpeed = -INS_palstance[1];
    IMU->YawSpeed = INS_palstance[2];
}
#endif
#ifdef IMU_DIRECTION_ryxz_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, -gyro[1], gyro[0], gyro[2] - IMU_GYRO_YAW_BIAS, -accel[1], accel[0], accel[2], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = -INS_palstance[1];
    IMU->PitchSpeed = INS_palstance[0];
    IMU->YawSpeed = INS_palstance[2];
}
#endif
#ifdef IMU_DIRECTION_zryx_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, gyro[2], -gyro[1], gyro[0] - IMU_GYRO_YAW_BIAS, accel[2], -accel[1], accel[0], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = INS_palstance[2];
    IMU->PitchSpeed = -INS_palstance[1];
    IMU->YawSpeed = INS_palstance[0];
}
#endif
#ifdef IMU_DIRECTION_yzx_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, gyro[1], gyro[2], gyro[0] - IMU_GYRO_YAW_BIAS, accel[1], accel[2], accel[0], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = INS_palstance[1];
    IMU->PitchSpeed = INS_palstance[2];
    IMU->YawSpeed = INS_palstance[0];
}
#endif
#ifdef IMU_DIRECTION_rzyx_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, -gyro[2], gyro[1], gyro[0] - IMU_GYRO_YAW_BIAS, -accel[2], accel[1], accel[0], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = -INS_palstance[2];
    IMU->PitchSpeed = INS_palstance[1];
    IMU->YawSpeed = INS_palstance[0];
}
#endif
#ifdef IMU_DIRECTION_ryrzx_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, -gyro[1], -gyro[2], gyro[0] - IMU_GYRO_YAW_BIAS, -accel[1], -accel[2], accel[0], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = -INS_palstance[1];
    IMU->PitchSpeed = -INS_palstance[2];
    IMU->YawSpeed = INS_palstance[0];
}
#endif
#ifdef IMU_DIRECTION_xzry_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, gyro[0], gyro[2], -gyro[1] - IMU_GYRO_YAW_BIAS, accel[0], accel[2], -accel[1], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = INS_palstance[0];
    IMU->PitchSpeed = INS_palstance[2];
    IMU->YawSpeed = -INS_palstance[1];
}
#endif





/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          ����bmi088���¶�
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, 45.0f);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //��û�дﵽ���õ��¶ȣ�һֱ����ʼ���
        //in beginning, max power
        if (temp > 45.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //�ﵽ�����¶ȣ�������������Ϊһ������ʣ���������
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }
taskENTER_CRITICAL_FROM_ISR
        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

void DMA2_Stream0_IRQHandler(void)
{

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //�����Ƕ�ȡ���
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_SET);
					
						BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro, INS_palstance);
        }

        //accel read over
        //���ٶȼƶ�ȡ���
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_SET);
					
						BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
        }
        //temperature read over
        //�¶ȶ�ȡ���
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
						HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_SET);
					
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
						imu_temp_control(bmi088_real_data.temp);
        }

        imu_cmd_spi_dma();//�¶ȡ����ٶ�ͬʱ��ȡ
    }
}
