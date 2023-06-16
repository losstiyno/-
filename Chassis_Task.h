#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "pid.h"
#include "Gimbal_Task.h"
#include "INS_task.h"

#define CHASSIS_TASK_INIT_TIME 	3000
#define CHASSIS_CONTROL_TIME		2			//���������ʱ��

//������� �ٶȻ� LT1
#define CAR_LT1_KP 3.0f
#define CAR_LT1_KI 0.015f
#define CAR_LT1_KD 0.0f
#define CAR_LT1_I_LIMIT 250.f
#define CAR_LT1_MAX 9000.f
static const fp32 PID_CAR_LT1[] = {CAR_LT1_KP, CAR_LT1_KI, CAR_LT1_KD};

//������� �ٶȻ� LT2
#define CAR_LT2_KP 2.3f
#define CAR_LT2_KI 0.0f
#define CAR_LT2_KD 0.0f
#define CAR_LT2_I_LIMIT 250
#define CAR_LT2_MAX 9000.f
static const fp32 PID_CAR_LT2[] = {CAR_LT2_KP, CAR_LT2_KI, CAR_LT2_KD};

//������� �ٶȻ� RT1
#define CAR_RT1_KP 3.0f
#define CAR_RT1_KI 0.2f
#define CAR_RT1_KD 0.0f
#define CAR_RT1_I_LIMIT 250
#define CAR_RT1_MAX 9000.f
static const fp32 PID_CAR_RT1[] = {CAR_RT1_KP, CAR_RT1_KI, CAR_RT1_KD};

//������� �ٶȻ� RT2
#define CAR_RT2_KP 3.5f
#define CAR_RT2_KI 0.2f
#define CAR_RT2_KD 0.0f
#define CAR_RT2_I_LIMIT 250
#define CAR_RT2_MAX 9000.f
static const fp32 PID_CAR_RT2[] = {CAR_RT2_KP, CAR_RT2_KI, CAR_RT2_KD};

//ת���� �ٶȻ� FL
#define MS7010_SPEED_FL_KP 6.0f
#define MS7010_SPEED_FL_KI 0.1f
#define MS7010_SPEED_FL_KD 60.0f
#define MS7010_SPEED_FL_I_LIMIT 6000.f
#define MS7010_SPEED_FL_MAX 10000.f
static const fp32 PID_MS7010_SPEED_FL[] = {MS7010_SPEED_FL_KP, MS7010_SPEED_FL_KI, MS7010_SPEED_FL_KD};

//ת���� �ǶȻ� FL
#define MS7010_ANGLE_FL_KP 1.0f
#define MS7010_ANGLE_FL_KI 0.0f
#define MS7010_ANGLE_FL_KD 0.0f
#define MS7010_ANGLE_FL_I_LIMIT 0.f
#define MS7010_ANGLE_FL_MAX 2000.f
static const fp32 PID_MS7010_ANGLE_FL[] = {MS7010_ANGLE_FL_KP, MS7010_ANGLE_FL_KI, MS7010_ANGLE_FL_KD};

//ת���� �ٶȻ� FR
#define MS7010_SPEED_FR_KP 6.0f
#define MS7010_SPEED_FR_KI 0.1f
#define MS7010_SPEED_FR_KD 60.0f
#define MS7010_SPEED_FR_I_LIMIT 250.f
#define MS7010_SPEED_FR_MAX 10000.f
static const fp32 PID_MS7010_SPEED_FR[] = {MS7010_SPEED_FR_KP, MS7010_SPEED_FR_KI, MS7010_SPEED_FR_KD};

//ת���� �ǶȻ� FR
#define MS7010_ANGLE_FR_KP 1.0f
#define MS7010_ANGLE_FR_KI 0.0f
#define MS7010_ANGLE_FR_KD 0.0f
#define MS7010_ANGLE_FR_I_LIMIT 0.f
#define MS7010_ANGLE_FR_MAX 2000.f
static const fp32 PID_MS7010_ANGLE_FR[] = {MS7010_ANGLE_FR_KP, MS7010_ANGLE_FR_KI, MS7010_ANGLE_FR_KD};

//ת���� �ٶȻ� BL
#define MS7010_SPEED_BL_KP 6.0f
#define MS7010_SPEED_BL_KI 0.1f
#define MS7010_SPEED_BL_KD 60.0f
#define MS7010_SPEED_BL_I_LIMIT 6000.f
#define MS7010_SPEED_BL_MAX 10000.f
static const fp32 PID_MS7010_SPEED_BL[] = {MS7010_SPEED_BL_KP, MS7010_SPEED_BL_KI, MS7010_SPEED_BL_KD};

//ת���� �ǶȻ� BL
#define MS7010_ANGLE_BL_KP 1.0f
#define MS7010_ANGLE_BL_KI 0.0f
#define MS7010_ANGLE_BL_KD 0.0f
#define MS7010_ANGLE_BL_I_LIMIT 0.f
#define MS7010_ANGLE_BL_MAX 2000.f
static const fp32 PID_MS7010_ANGLE_BL[] = {MS7010_ANGLE_BL_KP, MS7010_ANGLE_BL_KI, MS7010_ANGLE_BL_KD};

//ת���� �ٶȻ� BR
#define MS7010_SPEED_BR_KP 6.0f
#define MS7010_SPEED_BR_KI 0.1f
#define MS7010_SPEED_BR_KD 60.0f
#define MS7010_SPEED_BR_I_LIMIT 6000.f
#define MS7010_SPEED_BR_MAX 10000.f
static const fp32 PID_MS7010_SPEED_BR[] = {MS7010_SPEED_BR_KP, MS7010_SPEED_BR_KI, MS7010_SPEED_BR_KD};

//ת���� �ǶȻ� BR
#define MS7010_ANGLE_BR_KP 1.0f
#define MS7010_ANGLE_BR_KI 0.0f
#define MS7010_ANGLE_BR_KD 0.0f
#define MS7010_ANGLE_BR_I_LIMIT 0.f
#define MS7010_ANGLE_BR_MAX 2000.f
static const fp32 PID_MS7010_ANGLE_BR[] = {MS7010_ANGLE_BR_KP, MS7010_ANGLE_BR_KI, MS7010_ANGLE_BR_KD};

//���̸�����̨
#define CAR_FOLLOW_GIMBAL_KP 0.00015f
#define CAR_FOLLOW_GIMBAL_KI 0.0f
#define CAR_FOLLOW_GIMBAL_KD 0.0f
#define CAR_FOLLOW_GIMBAL_I_LIMIT 250.f
#define CAR_FOLLOW_GIMBAL_MAX 2000.f
static const fp32 PID_CAR_FOLLOW_GIMBAL[] = {CAR_FOLLOW_GIMBAL_KP, CAR_FOLLOW_GIMBAL_KI, CAR_FOLLOW_GIMBAL_KD};

typedef enum
{
		CHASSIS_ZERO_FORCE = 0, //����ģʽ
		CHASSIS_RC_GYROSCOPE = 1, //ң���� С����ģʽ
	  CHASSIS_RC_FOLLOW_GIMBAL = 2, //ң���� ������̨ģʽ
		CHASSIS_PC_CONTROL = 3, //PC ����ģʽ
		
}CHASSIS_MODE_e; //����ģʽ

extern fp32 vx_set, vy_set, wz_set;
extern fp32 PID_CurrentLT1, PID_CurrentLT2, PID_CurrentRT1, PID_CurrentRT2;
extern fp32 M3508_SPEED[4], MS7010_ANGLE[4];

void pid_chassis_all_init(void);
void chassis_vector_to_M3508_wheel_speed(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_speed[4]);
void chassis_vector_to_M7010_wheel_angle(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_angle[4]);

void chassis_mode_switch(void); //ģʽѡ����
void chassis_target_calc(uint8_t Mode); //Target���㺯��
void chassis_calc_cmd(uint8_t Mode); //����PID���㼰�������

float Angle_Limit (float angle ,float max);

#endif

