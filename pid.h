/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H

//#define ecd_range 32767
//#define half_ecd_range 16384

#define LIMIT(x,min,max) (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#define ABS(x) (((x)>0)?(x):(-(x)))

#include "main.h"
#include "arm_math.h"

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef float fp32;

typedef struct
{
    uint8_t mode;
    //PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�

} PidTypeDef;
extern void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
extern fp32 PID_Calc_Ecd(PidTypeDef *pid, fp32 ref, fp32 set, uint16_t ecd_range);
extern fp32 PID_Calc_Angle(PidTypeDef *pid, fp32 ref, fp32 set);
extern void PID_clear(PidTypeDef *pid);

void Ecd_zero(float *set,float *ref);
static fp32 ecd_zero(uint16_t ecd, uint16_t offset_ecd, uint16_t ecd_range);
fp32 angle_zero(fp32 angle, fp32 offset_angle);

#endif
