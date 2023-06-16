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

#define WHEEL_PERIMETER 			 376.98f	//车轮周长 (轮子直径 * PI 再转换成mm)
#define M3508_RATIO 	 				 19				//电机减速比
#define Radius 								 60				//轮径mm

//四个驱动电机朝向前方时 转向电机初始编码值
#define MS7010_FL_ANGLE 23434		//装上去的时候Y轴正向对应的编码值(偏置角度)
#define MS7010_FR_ANGLE 26985
#define MS7010_BL_ANGLE 344
#define MS7010_BR_ANGLE 13310

//底盘控制模式
uint8_t CHASSIS_MODE; 

void Chassis_Task(void const * argument)
{
		vTaskDelay(CHASSIS_TASK_INIT_TIME);			
	
		//初始化底盘模式为 底盘无力模式
		CHASSIS_MODE = CHASSIS_ZERO_FORCE;
	
		pid_chassis_all_init(); //pid参数初始化
	
		while(1)
		{
				chassis_mode_switch();  //根据遥控器 选择底盘控制模式
				chassis_target_calc(CHASSIS_MODE); //根据不同模式 计算控制目标值
				chassis_calc_cmd(CHASSIS_MODE); //PID计算并输出
	
				vTaskDelay(CHASSIS_CONTROL_TIME);			
		
		}

}

void pid_chassis_all_init(void)
{
		//驱动电机 M3508 初始化
		PID_Init(&pid_car_lt1,PID_POSITION,PID_CAR_LT1,CAR_LT1_MAX,CAR_LT1_I_LIMIT);
		PID_Init(&pid_car_lt2,PID_POSITION,PID_CAR_LT2,CAR_LT2_MAX,CAR_LT2_I_LIMIT);	
		PID_Init(&pid_car_rt1,PID_POSITION,PID_CAR_RT1,CAR_RT1_MAX,CAR_RT1_I_LIMIT);	
		PID_Init(&pid_car_rt2,PID_POSITION,PID_CAR_RT2,CAR_RT2_MAX,CAR_RT2_I_LIMIT);	
		
		//转向电机 MS7010 速度环 初始化
		PID_Init(&pid_ms7010_speed_fl,PID_POSITION,PID_MS7010_SPEED_FL,MS7010_SPEED_FL_MAX,MS7010_SPEED_FL_I_LIMIT);
		PID_Init(&pid_ms7010_speed_fr,PID_POSITION,PID_MS7010_SPEED_FR,MS7010_SPEED_FR_MAX,MS7010_SPEED_FR_I_LIMIT);
		PID_Init(&pid_ms7010_speed_bl,PID_POSITION,PID_MS7010_SPEED_BL,MS7010_SPEED_BL_MAX,MS7010_SPEED_BL_I_LIMIT);
		PID_Init(&pid_ms7010_speed_br,PID_POSITION,PID_MS7010_SPEED_BR,MS7010_SPEED_BR_MAX,MS7010_SPEED_BR_I_LIMIT);
	
		//转向电机 MS7010 角度环 初始化
		PID_Init(&pid_ms7010_angle_fl,PID_POSITION,PID_MS7010_ANGLE_FL,MS7010_ANGLE_FL_MAX,MS7010_ANGLE_FL_I_LIMIT);	
		PID_Init(&pid_ms7010_angle_fr,PID_POSITION,PID_MS7010_ANGLE_FR,MS7010_ANGLE_FR_MAX,MS7010_ANGLE_FR_I_LIMIT);		
		PID_Init(&pid_ms7010_angle_bl,PID_POSITION,PID_MS7010_ANGLE_BL,MS7010_ANGLE_BL_MAX,MS7010_ANGLE_BL_I_LIMIT);	
		PID_Init(&pid_ms7010_angle_br,PID_POSITION,PID_MS7010_ANGLE_BR,MS7010_ANGLE_BR_MAX,MS7010_ANGLE_BR_I_LIMIT);		
		
		//底盘跟随云台 速度环 初始化
		PID_Init(&pid_car_follow_gimbal,PID_POSITION,PID_CAR_FOLLOW_GIMBAL,CAR_FOLLOW_GIMBAL_MAX,CAR_FOLLOW_GIMBAL_I_LIMIT);		
}

//将电机转子转向内侧时 修正方向
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

	
		//7010目标角度计算
    if(!(vx_set == 0 && vy_set == 0 && wz_set == 0))//防止除数为零
    {
			//由于atan2算出来的结果是弧度，需转换成角度 计算公式为 弧度 * 180.f / PI 最终得到角度值 (0.707107f == 根号2)
      atan_angle[0] = atan2((vx_set - wz_set * Radius * 0.707107f),(vy_set + wz_set * Radius * 0.707107f)) * 180.0f / PI;		
      atan_angle[1] = atan2((vx_set - wz_set * Radius * 0.707107f),(vy_set - wz_set * Radius * 0.707107f)) * 180.0f / PI;
      atan_angle[2] = atan2((vx_set + wz_set * Radius * 0.707107f),(vy_set + wz_set * Radius * 0.707107f)) * 180.0f / PI;
      atan_angle[3] = atan2((vx_set + wz_set * Radius * 0.707107f),(vy_set - wz_set * Radius * 0.707107f)) * 180.0f / PI;	
    }  
		
		// 将一圈360°转换成编码值的一圈0-32767 -> 角度 * 32767 / 360 最终转换为需要转动的角度对应的编码值，再加上偏置角度,最终得到目标编码值
		wheel_angle[0] = Angle_Limit(MS7010_FL_ANGLE + (fp32)(atan_angle[0] * 91.02f), 32767.f);
		wheel_angle[1] = Angle_Limit(MS7010_FR_ANGLE + (fp32)(atan_angle[1] * 91.02f), 32767.f);
		wheel_angle[2] = Angle_Limit(MS7010_BL_ANGLE + (fp32)(atan_angle[2] * 91.02f), 32767.f);
		wheel_angle[3] = Angle_Limit(MS7010_BR_ANGLE + (fp32)(atan_angle[3] * 91.02f), 32767.f);
		
		//优弧 劣弧 驱动电机转向判断
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

void chassis_mode_switch(void) //模式选择函数
{	
		if(switch_is_mid(rc_ctrl.rc.s[RC_SW_RIGHT]) || switch_is_up(rc_ctrl.rc.s[RC_SW_RIGHT]))
		{
				if(switch_is_down(rc_ctrl.rc.s[RC_SW_LEFT]))		//左侧拨杆在下面 遥控器模式
						CHASSIS_MODE = CHASSIS_RC_GYROSCOPE;
				if(switch_is_mid(rc_ctrl.rc.s[RC_SW_LEFT]))			//左侧拨杆在中间 底盘跟随云台
						CHASSIS_MODE = CHASSIS_RC_FOLLOW_GIMBAL;
				if(switch_is_up(rc_ctrl.rc.s[RC_SW_LEFT]))			//左侧拨杆在上面 PC模式
						CHASSIS_MODE = CHASSIS_PC_CONTROL;
		}
		else //右侧拨杆在下面 云台无力模式
				CHASSIS_MODE = CHASSIS_ZERO_FORCE;
}

void chassis_target_calc(uint8_t Mode) //Target计算函数
{
	
		if(Mode == CHASSIS_PC_CONTROL) //PC模式
		{
				;
		}
		else if(Mode == CHASSIS_RC_GYROSCOPE) //遥控器 小陀螺模式
		{
				int16_t world_vx_set = - rc_ctrl.rc.ch[2] / 200.f;
				int16_t world_vy_set = 	 rc_ctrl.rc.ch[3] / 200.f;
		
				//跟随的云台角度
				fp32 theta = (gimbal_measure[1].ecd - 6120) * Motor_Ecd_to_Rad;
			
				fp32 sin_yaw = arm_sin_f32(theta);
				fp32 cos_yaw = arm_cos_f32(theta);
			
				vx_set = cos_yaw * world_vx_set - sin_yaw * world_vy_set;
				vy_set = sin_yaw * world_vx_set + cos_yaw * world_vy_set;
			
				wz_set = -1;
			
		}
		else if(Mode == CHASSIS_RC_FOLLOW_GIMBAL) //底盘跟随云台模式
		{
				vx_set = 	-	rc_ctrl.rc.ch[2] / 200.f;
				vy_set = 	 	rc_ctrl.rc.ch[3] / 200.f;
			
				wz_set = PID_Calc_Ecd(&pid_car_follow_gimbal, gimbal_measure[0].ecd, 6120, 8191);
		}
		else if(Mode == CHASSIS_ZERO_FORCE) //无力模式
		{
				;
		}

}

void chassis_calc_cmd(uint8_t Mode) //底盘PID计算及输出后函数
{
		if(Mode == CHASSIS_PC_CONTROL) //PC模式
		{
				;
		}
		else if(Mode == CHASSIS_RC_GYROSCOPE) //遥控器 直接控制模式
		{
				//运动分解
				chassis_vector_to_M7010_wheel_angle(vx_set, vy_set, wz_set, MS7010_ANGLE);
				chassis_vector_to_M3508_wheel_speed(vx_set, vy_set, wz_set, M3508_SPEED);
			
				//驱动电机 速度环 PID
				PID_CurrentLT1 = PID_Calc(&pid_car_lt1,	chassis_m3508_measure[0].speed_rpm,	M3508_SPEED[0]);
				PID_CurrentRT1 = PID_Calc(&pid_car_rt1,	chassis_m3508_measure[1].speed_rpm,	M3508_SPEED[1]);
				PID_CurrentLT2 = PID_Calc(&pid_car_lt2,	chassis_m3508_measure[2].speed_rpm,	M3508_SPEED[2]);
				PID_CurrentRT2 = PID_Calc(&pid_car_rt2,	chassis_m3508_measure[3].speed_rpm,	M3508_SPEED[3]);			

				//转向电机 角度环串速度环 PID			
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
		else if(Mode == CHASSIS_RC_FOLLOW_GIMBAL) //底盘跟随云台模式
		{				
				//运动分解
				chassis_vector_to_M7010_wheel_angle(vx_set, vy_set, wz_set, MS7010_ANGLE);
				chassis_vector_to_M3508_wheel_speed(vx_set, vy_set, wz_set, M3508_SPEED);
			
				//驱动电机 速度环 PID
				PID_CurrentLT1	=	PID_Calc(&pid_car_lt1 ,chassis_m3508_measure[0].speed_rpm ,M3508_SPEED[0]);
				PID_CurrentRT1	= PID_Calc(&pid_car_rt1	,chassis_m3508_measure[1].speed_rpm	,M3508_SPEED[1]);
				PID_CurrentLT2	= PID_Calc(&pid_car_lt2 ,chassis_m3508_measure[2].speed_rpm ,M3508_SPEED[2]);
				PID_CurrentRT2 	= PID_Calc(&pid_car_rt2	,chassis_m3508_measure[3].speed_rpm	,M3508_SPEED[3]);			

				//转向电机 角度环串速度环 PID
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
		else if(Mode == CHASSIS_ZERO_FORCE) //无力模式
		{
				CAN_CMD_CHASSIS_M3508(&hcan2, 0, 0, 0, 0);
				CAN_CMD_CHASSIS_MS7010(&hcan2, 0, 0, 0, 0);
		}
		
}

//将角度范围控制在 0 - 32767
float Angle_Limit (float angle ,float max)
{
		if(angle > max)
			angle -= max;
		if(angle < 0)
			angle += max; 
		return angle;
}
