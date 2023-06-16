#include "Chassis_Task.h"
#include "CAN_receive.h"
#include "RC_Task.h"
#include "arm_math.h"

PidTypeDef pid_car_lt1, pid_car_lt2, pid_car_rt1, pid_car_rt2, pid_car_follow_gimbal;
PidTypeDef pid_ms7010_speed_fl, pid_ms7010_speed_fr, pid_ms7010_speed_bl, pid_ms7010_speed_br;
PidTypeDef pid_ms7010_angle_fl, pid_ms7010_angle_fr, pid_ms7010_angle_bl, pid_ms7010_angle_br;

fp32 PID_CurrentLT1, PID_CurrentLT2, PID_CurrentRT1, PID_CurrentRT2;
fp32 PID_MS7010_Speed_Current_FL, PID_MS7010_Speed_Current_FR, PID_MS7010_Speed_Current_BL, PID_MS7010_Speed_Current_BR;
fp32 PID_MS7010_Current_FL, PID_MS7010_Current_FR, PID_MS7010_Current_BL, PID_MS7010_Current_BR;

fp32 M3508_SPEED[4], MS7010_ANGLE[4];
fp32 ms7010_speed[4];

fp32 relative_angle_set;

fp32 vx_set, vy_set, wz_set;

#define Motor_Ecd_to_Rad 0.000766990394f //2 * PI / 8192

#define WHEEL_PERIMETER 			 376.98f	//�����ܳ� (����ֱ�� * PI ��ת����mm)
#define M3508_RATIO 	 				 19				//������ٱ�
#define Radius 								 60				//�־�mm

//�ĸ������������ǰ��ʱ ת������ʼ����ֵ
#define MS7010_FL_ANGLE 23434		//װ��ȥ��ʱ��Y�������Ӧ�ı���ֵ(ƫ�ýǶ�)
#define MS7010_FR_ANGLE 26985
#define MS7010_BL_ANGLE 344
#define MS7010_BR_ANGLE 13310

//���̿���ģʽ
uint8_t CHASSIS_MODE; 

void Chassis_Task(void const * argument)
{
		vTaskDelay(CHASSIS_TASK_INIT_TIME);			
	
		//��ʼ������ģʽΪ ��������ģʽ
		CHASSIS_MODE = CHASSIS_ZERO_FORCE;
	
		pid_chassis_all_init(); //pid������ʼ��
	
		while(1)
		{
				chassis_mode_switch();  //����ң���� ѡ����̿���ģʽ
				chassis_target_calc(CHASSIS_MODE); //���ݲ�ͬģʽ �������Ŀ��ֵ
				chassis_calc_cmd(CHASSIS_MODE); //PID���㲢���
	
				vTaskDelay(CHASSIS_CONTROL_TIME);			
		
		}

}

void pid_chassis_all_init(void)
{
		//������� M3508 ��ʼ��
		PID_Init(&pid_car_lt1,PID_POSITION,PID_CAR_LT1,CAR_LT1_MAX,CAR_LT1_I_LIMIT);
		PID_Init(&pid_car_lt2,PID_POSITION,PID_CAR_LT2,CAR_LT2_MAX,CAR_LT2_I_LIMIT);	
		PID_Init(&pid_car_rt1,PID_POSITION,PID_CAR_RT1,CAR_RT1_MAX,CAR_RT1_I_LIMIT);	
		PID_Init(&pid_car_rt2,PID_POSITION,PID_CAR_RT2,CAR_RT2_MAX,CAR_RT2_I_LIMIT);	
		
		//ת���� MS7010 �ٶȻ� ��ʼ��
		PID_Init(&pid_ms7010_speed_fl,PID_POSITION,PID_MS7010_SPEED_FL,MS7010_SPEED_FL_MAX,MS7010_SPEED_FL_I_LIMIT);
		PID_Init(&pid_ms7010_speed_fr,PID_POSITION,PID_MS7010_SPEED_FR,MS7010_SPEED_FR_MAX,MS7010_SPEED_FR_I_LIMIT);
		PID_Init(&pid_ms7010_speed_bl,PID_POSITION,PID_MS7010_SPEED_BL,MS7010_SPEED_BL_MAX,MS7010_SPEED_BL_I_LIMIT);
		PID_Init(&pid_ms7010_speed_br,PID_POSITION,PID_MS7010_SPEED_BR,MS7010_SPEED_BR_MAX,MS7010_SPEED_BR_I_LIMIT);
	
		//ת���� MS7010 �ǶȻ� ��ʼ��
		PID_Init(&pid_ms7010_angle_fl,PID_POSITION,PID_MS7010_ANGLE_FL,MS7010_ANGLE_FL_MAX,MS7010_ANGLE_FL_I_LIMIT);	
		PID_Init(&pid_ms7010_angle_fr,PID_POSITION,PID_MS7010_ANGLE_FR,MS7010_ANGLE_FR_MAX,MS7010_ANGLE_FR_I_LIMIT);		
		PID_Init(&pid_ms7010_angle_bl,PID_POSITION,PID_MS7010_ANGLE_BL,MS7010_ANGLE_BL_MAX,MS7010_ANGLE_BL_I_LIMIT);	
		PID_Init(&pid_ms7010_angle_br,PID_POSITION,PID_MS7010_ANGLE_BR,MS7010_ANGLE_BR_MAX,MS7010_ANGLE_BR_I_LIMIT);		
		
		//���̸�����̨ �ٶȻ� ��ʼ��
		PID_Init(&pid_car_follow_gimbal,PID_POSITION,PID_CAR_FOLLOW_GIMBAL,CAR_FOLLOW_GIMBAL_MAX,CAR_FOLLOW_GIMBAL_I_LIMIT);		
}

//�����ת��ת���ڲ�ʱ ��������
int8_t dirt[4] = { 1, -1, 1, -1};
void chassis_vector_to_M3508_wheel_speed(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_speed[4])
{
	  fp32 wheel_rpm_ratio;
	
    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * M3508_RATIO * 1000;

    wheel_speed[0] = dirt[0] * sqrt(	pow(vy_set + wz_set * Radius * 0.707107f,2)
                       +	pow(vx_set - wz_set * Radius * 0.707107f,2)
                       ) * wheel_rpm_ratio ;
    wheel_speed[1] = dirt[1] * sqrt(	pow(vy_set - wz_set * Radius * 0.707107f,2)
                       +	pow(vx_set - wz_set * Radius * 0.707107f,2)
                       ) * wheel_rpm_ratio ;
    wheel_speed[2] = dirt[2] * sqrt(	pow(vy_set - wz_set * Radius * 0.707107f,2)
                       +	pow(vx_set + wz_set * Radius * 0.707107f,2)
                       ) * wheel_rpm_ratio ;
    wheel_speed[3] = dirt[3] * sqrt(	pow(vy_set + wz_set * Radius * 0.707107f,2)
                       +	pow(vx_set + wz_set * Radius * 0.707107f,2) 
                       ) * wheel_rpm_ratio ;
		
}

fp64 atan_angle[4];
void chassis_vector_to_M7010_wheel_angle(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_angle[4])
{

	
		//7010Ŀ��Ƕȼ���
    if(!(vx_set == 0 && vy_set == 0 && wz_set == 0))//��ֹ����Ϊ��
    {
			//����atan2������Ľ���ǻ��ȣ���ת���ɽǶ� ���㹫ʽΪ ���� * 180.f / PI ���յõ��Ƕ�ֵ (0.707107f == ����2)
      atan_angle[0] = atan2((vx_set - wz_set * Radius * 0.707107f),(vy_set + wz_set * Radius * 0.707107f)) * 180.0f / PI;		
      atan_angle[1] = atan2((vx_set - wz_set * Radius * 0.707107f),(vy_set - wz_set * Radius * 0.707107f)) * 180.0f / PI;
      atan_angle[2] = atan2((vx_set + wz_set * Radius * 0.707107f),(vy_set + wz_set * Radius * 0.707107f)) * 180.0f / PI;
      atan_angle[3] = atan2((vx_set + wz_set * Radius * 0.707107f),(vy_set - wz_set * Radius * 0.707107f)) * 180.0f / PI;	
    }  
		
		// ��һȦ360��ת���ɱ���ֵ��һȦ0-32767 -> �Ƕ� * 32767 / 360 ����ת��Ϊ��Ҫת���ĽǶȶ�Ӧ�ı���ֵ���ټ���ƫ�ýǶ�,���յõ�Ŀ�����ֵ
		wheel_angle[0] = Angle_Limit(MS7010_FL_ANGLE + (fp32)(atan_angle[0] * 91.02f), 32767.f);
		wheel_angle[1] = Angle_Limit(MS7010_FR_ANGLE + (fp32)(atan_angle[1] * 91.02f), 32767.f);
		wheel_angle[2] = Angle_Limit(MS7010_BL_ANGLE + (fp32)(atan_angle[2] * 91.02f), 32767.f);
		wheel_angle[3] = Angle_Limit(MS7010_BR_ANGLE + (fp32)(atan_angle[3] * 91.02f), 32767.f);
		
		//�Ż� �ӻ� �������ת���ж�
	 if( ABS( (fp32)chassis_ms7010_measure[0].ecd - wheel_angle[0] ) > 8192 )
	 {	
			dirt[0] = -1;
			wheel_angle[0] = Angle_Limit( wheel_angle[0] - 16384, 32767 );
	 }
	 else
		 dirt[0] = 1;
		
   if( ABS( (fp32)chassis_ms7010_measure[1].ecd - wheel_angle[1] ) > 8192 )
	 {	
			dirt[1] = 1;
			wheel_angle[1] = Angle_Limit( wheel_angle[1] - 16384, 32767 );
	 }
	 else
		 dirt[1] = -1;

   if( ABS( (fp32)chassis_ms7010_measure[2].ecd - wheel_angle[2] ) > 8192 )
	 {	
			dirt[2] = -1;
			wheel_angle[2] = Angle_Limit( wheel_angle[2] - 16384, 32767 );
	 }
	 else
		 dirt[2] = 1;

   if( ABS( (fp32)chassis_ms7010_measure[3].ecd - wheel_angle[3] ) > 8192 )
	 {	
			dirt[3] = 1;
			wheel_angle[3] = Angle_Limit( wheel_angle[3] - 16384, 32767 );
	 }
	 else
		 dirt[3] = -1;
	 
}

void chassis_mode_switch(void) //ģʽѡ����
{	
		if(switch_is_mid(rc_ctrl.rc.s[RC_SW_RIGHT]) || switch_is_up(rc_ctrl.rc.s[RC_SW_RIGHT]))
		{
				if(switch_is_down(rc_ctrl.rc.s[RC_SW_LEFT]))		//��ದ�������� ң����ģʽ
						CHASSIS_MODE = CHASSIS_RC_GYROSCOPE;
				if(switch_is_mid(rc_ctrl.rc.s[RC_SW_LEFT]))			//��ದ�����м� ���̸�����̨
						CHASSIS_MODE = CHASSIS_RC_FOLLOW_GIMBAL;
				if(switch_is_up(rc_ctrl.rc.s[RC_SW_LEFT]))			//��ದ�������� PCģʽ
						CHASSIS_MODE = CHASSIS_PC_CONTROL;
		}
		else //�Ҳದ�������� ��̨����ģʽ
				CHASSIS_MODE = CHASSIS_ZERO_FORCE;
}

void chassis_target_calc(uint8_t Mode) //Target���㺯��
{
	
		if(Mode == CHASSIS_PC_CONTROL) //PCģʽ
		{
				;
		}
		else if(Mode == CHASSIS_RC_GYROSCOPE) //ң���� С����ģʽ
		{
				int16_t world_vx_set = - rc_ctrl.rc.ch[2] / 200.f;
				int16_t world_vy_set = 	 rc_ctrl.rc.ch[3] / 200.f;
		
				//�������̨�Ƕ�
				fp32 theta = (gimbal_measure[1].ecd - 6120) * Motor_Ecd_to_Rad;
			
				fp32 sin_yaw = arm_sin_f32(theta);
				fp32 cos_yaw = arm_cos_f32(theta);
			
				vx_set = cos_yaw * world_vx_set - sin_yaw * world_vy_set;
				vy_set = sin_yaw * world_vx_set + cos_yaw * world_vy_set;
			
				wz_set = -1;
			
		}
		else if(Mode == CHASSIS_RC_FOLLOW_GIMBAL) //���̸�����̨ģʽ
		{
				vx_set = 	-	rc_ctrl.rc.ch[2] / 200.f;
				vy_set = 	 	rc_ctrl.rc.ch[3] / 200.f;
			
				wz_set = PID_Calc_Ecd(&pid_car_follow_gimbal, gimbal_measure[0].ecd, 6120, 8191);
		}
		else if(Mode == CHASSIS_ZERO_FORCE) //����ģʽ
		{
				;
		}

}

void chassis_calc_cmd(uint8_t Mode) //����PID���㼰�������
{
		if(Mode == CHASSIS_PC_CONTROL) //PCģʽ
		{
				;
		}
		else if(Mode == CHASSIS_RC_GYROSCOPE) //ң���� ֱ�ӿ���ģʽ
		{
				//�˶��ֽ�
				chassis_vector_to_M7010_wheel_angle(vx_set, vy_set, wz_set, MS7010_ANGLE);
				chassis_vector_to_M3508_wheel_speed(vx_set, vy_set, wz_set, M3508_SPEED);
			
				//������� �ٶȻ� PID
				PID_CurrentLT1 = PID_Calc(&pid_car_lt1,	chassis_m3508_measure[0].speed_rpm,	M3508_SPEED[0]);
				PID_CurrentRT1 = PID_Calc(&pid_car_rt1,	chassis_m3508_measure[1].speed_rpm,	M3508_SPEED[1]);
				PID_CurrentLT2 = PID_Calc(&pid_car_lt2,	chassis_m3508_measure[2].speed_rpm,	M3508_SPEED[2]);
				PID_CurrentRT2 = PID_Calc(&pid_car_rt2,	chassis_m3508_measure[3].speed_rpm,	M3508_SPEED[3]);			

				//ת���� �ǶȻ����ٶȻ� PID			
				PID_MS7010_Speed_Current_FL = PID_Calc_Ecd(&pid_ms7010_angle_fl, chassis_ms7010_measure[0].ecd, MS7010_ANGLE[0], 32767);
				PID_MS7010_Current_FL = PID_Calc(&pid_ms7010_speed_fl, chassis_ms7010_measure[0].speed_rpm, PID_MS7010_Speed_Current_FL);
			
				PID_MS7010_Speed_Current_FR = PID_Calc_Ecd(&pid_ms7010_angle_fr, chassis_ms7010_measure[1].ecd, MS7010_ANGLE[1], 32767);
				PID_MS7010_Current_FR = PID_Calc(&pid_ms7010_speed_fr, chassis_ms7010_measure[1].speed_rpm, PID_MS7010_Speed_Current_FR);
			
				PID_MS7010_Speed_Current_BL = PID_Calc_Ecd(&pid_ms7010_angle_bl, chassis_ms7010_measure[2].ecd, MS7010_ANGLE[2], 32767);
				PID_MS7010_Current_BL = PID_Calc(&pid_ms7010_speed_bl, chassis_ms7010_measure[2].speed_rpm, PID_MS7010_Speed_Current_BL);			
			
				PID_MS7010_Speed_Current_BR = PID_Calc_Ecd(&pid_ms7010_angle_br, chassis_ms7010_measure[3].ecd, MS7010_ANGLE[3], 32767);
				PID_MS7010_Current_BR = PID_Calc(&pid_ms7010_speed_br, chassis_ms7010_measure[3].speed_rpm, PID_MS7010_Speed_Current_BR);				

				CAN_CMD_CHASSIS_MS7010(&hcan2, PID_MS7010_Current_FL, PID_MS7010_Current_FR, PID_MS7010_Current_BL, PID_MS7010_Current_BR);
				CAN_CMD_CHASSIS_M3508(&hcan2, PID_CurrentLT1, PID_CurrentRT1, PID_CurrentLT2, PID_CurrentRT2);

		}
		else if(Mode == CHASSIS_RC_FOLLOW_GIMBAL) //���̸�����̨ģʽ
		{				
				//�˶��ֽ�
				chassis_vector_to_M7010_wheel_angle(vx_set, vy_set, wz_set, MS7010_ANGLE);
				chassis_vector_to_M3508_wheel_speed(vx_set, vy_set, wz_set, M3508_SPEED);
			
				//������� �ٶȻ� PID
				PID_CurrentLT1	=	PID_Calc(&pid_car_lt1 ,chassis_m3508_measure[0].speed_rpm ,M3508_SPEED[0]);
				PID_CurrentRT1	= PID_Calc(&pid_car_rt1	,chassis_m3508_measure[1].speed_rpm	,M3508_SPEED[1]);
				PID_CurrentLT2	= PID_Calc(&pid_car_lt2 ,chassis_m3508_measure[2].speed_rpm ,M3508_SPEED[2]);
				PID_CurrentRT2 	= PID_Calc(&pid_car_rt2	,chassis_m3508_measure[3].speed_rpm	,M3508_SPEED[3]);			

				//ת���� �ǶȻ����ٶȻ� PID
				PID_MS7010_Speed_Current_FL = PID_Calc_Ecd(&pid_ms7010_angle_fl, chassis_ms7010_measure[0].ecd, MS7010_ANGLE[0], 32767);
				PID_MS7010_Current_FL = PID_Calc(&pid_ms7010_speed_fl, chassis_ms7010_measure[0].speed_rpm, PID_MS7010_Speed_Current_FL);
			
				PID_MS7010_Speed_Current_FR = PID_Calc_Ecd(&pid_ms7010_angle_fr, chassis_ms7010_measure[1].ecd, MS7010_ANGLE[1], 32767);
				PID_MS7010_Current_FR = PID_Calc(&pid_ms7010_speed_fr, chassis_ms7010_measure[1].speed_rpm, PID_MS7010_Speed_Current_FR);
			
				PID_MS7010_Speed_Current_BL = PID_Calc_Ecd(&pid_ms7010_angle_bl, chassis_ms7010_measure[2].ecd, MS7010_ANGLE[2], 32767);
				PID_MS7010_Current_BL = PID_Calc(&pid_ms7010_speed_bl, chassis_ms7010_measure[2].speed_rpm, PID_MS7010_Speed_Current_BL);			
			
				PID_MS7010_Speed_Current_BR = PID_Calc_Ecd(&pid_ms7010_angle_br, chassis_ms7010_measure[3].ecd, MS7010_ANGLE[3], 32767);

				CAN_CMD_CHASSIS_MS7010(&hcan2, PID_MS7010_Current_FL, PID_MS7010_Current_FR, PID_MS7010_Speed_Current_BL, PID_MS7010_Speed_Current_BR);
				CAN_CMD_CHASSIS_M3508(&hcan2, PID_CurrentLT1, PID_CurrentRT1, PID_CurrentLT2, PID_CurrentRT2);
		
		}
		else if(Mode == CHASSIS_ZERO_FORCE) //����ģʽ
		{
				CAN_CMD_CHASSIS_M3508(&hcan2, 0, 0, 0, 0);
				CAN_CMD_CHASSIS_MS7010(&hcan2, 0, 0, 0, 0);
		}
		
}

//���Ƕȷ�Χ������ 0 - 32767
float Angle_Limit (float angle ,float max)
{
		if(angle > max)
			angle -= max;
		if(angle < 0)
			angle += max; 
		return angle;
}
