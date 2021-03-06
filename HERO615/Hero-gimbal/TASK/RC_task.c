#include "stm32f4xx.h"    
#include "rc_task.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "math.h"
#include "vision_task.h"        
#include "bsp_uart.h"
#include "shoot_task.h"
#include "can_receive.h"

int vision_switch_flag=0;//视觉开关的标识符，用来实现拨一下开关视觉，再播一下开关视觉的功能 
static int deadline_judge(uint8_t a);
float rc_cali(int max,int min, int RC_actual, int RC_max, int RC_mid, int RC_min,int deadline);
int F_flag=0;//判断F是否开启
int frie_first_flag=0;//判断第一次发单
int MOUSE_pre_left_cnt=0;	
KEY_CONTROL KEY_MODE=KEY_OFF;
void control_mode_judge(void)
{
	if(rc_ctrl.rc.ch[0]!=0||rc_ctrl.rc.ch[1]!=0||rc_ctrl.rc.ch[2]!=0||rc_ctrl.rc.ch[3]!=0||rc_ctrl.rc.ch[4]!=0)
		KEY_MODE=KEY_OFF;
	if(KEY_board||MOUSE_x||MOUSE_y||MOUSE_z)
		KEY_MODE=KEY_ON;
}
int shoot_true=0,shoot_true_cnt=0;
void remote_control_data(void)
{
//	if(!(KEY_board||MOUSE_x||MOUSE_y||MOUSE_z)==0)
//		return;
	if(switch_is_down(SW_R)&&vision_switch_flag==0) //视觉模式选择
	{
		if(vision_mode==VISION_OFF)
		{
			vision_mode=VISION_ON;
			gimbal_set_mode=GIMBAL_RELATIVE_ANGLE;
//			gimbal_set_mode=GIMBAL_ABSOLUTE_ANGLE;
			vision_sent.yaw.target_angle=gimbal_y.CAN_actual_angle;
			vision_sent.pitch.target_angle=gimbal_p.IMU_actual_angle;
		    gimbal_y.target_angle=gimbal_y.CAN_actual_angle;
			gimbal_p.target_angle=gimbal_p.IMU_actual_angle;
		}
			
		else 
		{
			gimbal_set_mode=GIMBAL_ABSOLUTE_ANGLE;
		    gimbal_y.target_angle=gimbal_y.IMU_actual_angle;
			vision_mode=VISION_OFF;
		}
		vision_switch_flag=1;
	}
	else if(switch_is_up(SW_R) || switch_is_mid(SW_R)) vision_switch_flag=0;
	
	if(switch_is_up(SW_R)&&frie_first_flag==0)
	{
		rc_shoot.left_fric.target_speed = -SHOOT_FRIC_HIGH_SPEED;
		rc_shoot.right_fric.target_speed = SHOOT_FRIC_HIGH_SPEED;
		frie_first_flag=1;
	}

	if(switch_is_up(SW_L) && gimbal_set_mode!=GIMBAL_RELATIVE_ANGLE) //小陀螺模式
		gimbal_set_mode = GIMBAL_TOP_ANGLE; 

	if(switch_is_mid(SW_L) && gimbal_set_mode!=GIMBAL_RELATIVE_ANGLE) //随动
		gimbal_set_mode = GIMBAL_ABSOLUTE_ANGLE;  

	if(switch_is_down(SW_L)) //脱力状态
		gimbal_set_mode = GIMBAL_ZERO_FORCE;  
	
	//遥控器摇杆通道值映射到底盘和云台的控制量
	rc_sent.x_speed=rc_cali(X_SPEED_MAXX,X_SPEED_MINN,rc_ctrl.rc.ch[3],RC_MAXX,RC_MIDD,RC_MINN,DEADLINE);
	rc_sent.y_speed=rc_cali(Y_SPEED_MAXX,Y_SPEED_MINN,rc_ctrl.rc.ch[2],RC_MAXX,RC_MIDD,RC_MINN,DEADLINE);
	
	if(!(rc_ctrl.rc.ch[4])==0)
	rc_sent.r_speed=rc_cali(Z_SPEED_MAXX,Z_SPEED_MINN,rc_ctrl.rc.ch[4],RC_MAXX,RC_MIDD,RC_MINN,DEADLINE);
	
	rc_sent.yaw.target_angle=-1.0f*rc_cali(RC_YAW_ANGLE_MAXX,RC_YAW_ANGLE_MINN,rc_ctrl.rc.ch[0],RC_MAXX,RC_MIDD,RC_MINN,DEADLINE);
	rc_sent.pitch.target_angle=rc_cali(RC_PITCH_ANGLE_MAXX,RC_PITCH_ANGLE_MINN,rc_ctrl.rc.ch[1],RC_MAXX,RC_MIDD,RC_MINN,DEADLINE);
		
	rc_sent.yaw.target_speed=-1.0f*rc_cali(RC_YAW_ANGLE_MAXX,RC_YAW_ANGLE_MINN,rc_ctrl.rc.ch[0],RC_MAXX,RC_MIDD,RC_MINN,DEADLINE);
	rc_sent.pitch.target_angle=rc_cali(RC_PITCH_ANGLE_MAXX,RC_PITCH_ANGLE_MINN,rc_ctrl.rc.ch[1],RC_MAXX,RC_MIDD,RC_MINN,DEADLINE);
	
	if(rc_shoot.trigger.last_shoot_flag==0 && frie_first_flag==1)
	{
		if(switch_is_up(SW_R) && rc_shoot.trigger.last_shoot_flag==0)  
		{			
			rc_shoot.trigger.target_angle=SHOOT_NUM;
			rc_shoot.trigger.last_shoot_flag=1;
		}
	}
	
	if(switch_is_mid(SW_R))
	{
		rc_shoot.trigger.last_shoot_flag=0;
	}
	
}
int R_flag=0,R_cnt=0;

void key_control_data(void)
{	
//	if((KEY_board||MOUSE_x||MOUSE_y||MOUSE_z)==0)
//	return;
	if(R_flag==1)
	R_cnt++;
	if(R_cnt>=1000) {R_flag=0;R_cnt=0;}
	rc_sent.x_speed=0;
	rc_sent.y_speed=0;
	if(KEY_board & KEY_PRESSED_OFFSET_SHIFT)
	{
			if(KEY_board & KEY_PRESSED_OFFSET_W)
				rc_sent.x_speed=KEY_X_SPEED_MAXX;
			if(KEY_board & KEY_PRESSED_OFFSET_S)
				rc_sent.x_speed=KEY_X_SPEED_MINN;
			if(KEY_board & KEY_PRESSED_OFFSET_A)
				rc_sent.y_speed=KEY_Y_SPEED_MINN;
			if(KEY_board & KEY_PRESSED_OFFSET_D)
				rc_sent.y_speed=KEY_Y_SPEED_MAXX;
			rc_sent.yaw.target_angle=-limits_change(KEY_YAW_ANGLE_MAXX_RUN,KEY_YAW_ANGLE_MINN_RUN,MOUSE_x,KEY_MAXX,KEY_MINN);
			rc_sent.pitch.target_angle=-limits_change(KEY_PITCH_ANGLE_MAXX_RUN,KEY_PITCH_ANGLE_MINN_RUN,MOUSE_y,KEY_MAXX,KEY_MINN);
	}
	else
	{
			if(KEY_board & KEY_PRESSED_OFFSET_W)
				rc_sent.x_speed=90;
			if(KEY_board & KEY_PRESSED_OFFSET_S)
				rc_sent.x_speed=-90;
			if(KEY_board & KEY_PRESSED_OFFSET_A)
				rc_sent.y_speed=-80;
			if(KEY_board & KEY_PRESSED_OFFSET_D)
				rc_sent.y_speed=80;
			rc_sent.yaw.target_angle=-limits_change(KEY_YAW_ANGLE_MAXX_ON,KEY_YAW_ANGLE_MINN_ON,MOUSE_x,KEY_MAXX,KEY_MINN);
			rc_sent.pitch.target_angle=-limits_change(KEY_PITCH_ANGLE_MAXX_ON,KEY_PITCH_ANGLE_MINN_ON,MOUSE_y,KEY_MAXX,KEY_MINN);
	}
	if(F_flag==1 && gimbal_set_mode != GIMBAL_TOP_ANGLE ) gimbal_set_mode=GIMBAL_RELATIVE_ANGLE;
	else if(F_flag==0 && gimbal_set_mode != GIMBAL_TOP_ANGLE) gimbal_set_mode=GIMBAL_ABSOLUTE_ANGLE;
	if(KEY_board & KEY_PRESSED_OFFSET_Q)
	{
		if(gimbal_set_mode==GIMBAL_RELATIVE_ANGLE  || F_flag==1)
		{
			F_flag=0;
		 gimbal_y.target_angle=gimbal_y.IMU_actual_angle;
		}
		gimbal_set_mode=GIMBAL_ABSOLUTE_ANGLE;

	}
	if(KEY_board & KEY_PRESSED_OFFSET_E)
		gimbal_set_mode = GIMBAL_TOP_ANGLE;
	if(KEY_board & KEY_PRESSED_OFFSET_F)
	{
		gimbal_set_mode=GIMBAL_RELATIVE_ANGLE;
		F_flag=1;
		gimbal_y.target_angle=gimbal_y.CAN_total_angle;
		
	}

	if((MOUSE_pre_left==1 || shoot_vision_flag==1 ) && rc_shoot.trigger.last_shoot_flag==0) 
    {			
		if(frie_first_flag==0)
		{
			frie_first_flag=1;
		    rc_shoot.left_fric.target_speed = -SHOOT_FRIC_HIGH_SPEED;
		    rc_shoot.right_fric.target_speed = SHOOT_FRIC_HIGH_SPEED;
		}
		else
		{
			rc_shoot.trigger.target_angle=SHOOT_NUM;
	  	rc_shoot.trigger.last_shoot_flag=1;
			shoot_true=0;
		}
	}
	if(MOUSE_pre_left==0) 
	{
//		if(rc_shoot.trigger.last_shoot_flag==1)
//		{
//			if(shoot_flag==1) shoot_true=1;
//		}
		MOUSE_pre_left_cnt++;
		if(MOUSE_pre_left_cnt>=100)
		{rc_shoot.trigger.last_shoot_flag=0;MOUSE_pre_left_cnt=0;}
//		if(shoot_true==1)
//		{
//			rc_shoot.trigger.target_angle=SHOOT_NUM;
//	  	rc_shoot.trigger.last_shoot_flag=1;
//			shoot_true=0;
//		}
	}

   	if(MOUSE_pre_right==1) 
    {			
        vision_mode=VISION_ON;
		gimbal_set_mode=GIMBAL_RELATIVE_ANGLE;
		gimbal_y.target_angle=gimbal_y.CAN_total_angle;
	    vision_pitch=gimbal_y.IMU_actual_angle;
	}
	else 
	{
		//gimbal_set_mode=GIMBAL_ABSOLUTE_ANGLE;
//		gimbal_y.target_angle=gimbal_y.IMU_actual_angle;
	    vision_mode=VISION_OFF;
	}
//	rc_sent.yaw.target_angle=limits_change(KEY_YAW_ANGLE_MAXX_ON,KEY_YAW_ANGLE_MINN_ON,MOUSE_x,KEY_MAXX,KEY_MINN);
//    if(deadline_judge(MOUSE_y)==0)
//	rc_sent.pitch.target_angle=-limits_change(KEY_PITCH_ANGLE_MAXX_ON,KEY_PITCH_ANGLE_MINN_ON,MOUSE_y,KEY_MAXX,KEY_MINN);
//	    
	if(KEY_PRESSED_OFFSET_C&KEY_board)
	{
		
			VISION_RESET_FLAG=ON_RESET;
	}
}
/**
	* @brief       幅度判断函数
	* @param[a]		想要判断的数值
	* @retvel      1（不超限幅），0（超限幅）
*/
static int deadline_judge(uint8_t a)
{
	if(abs(a-RC_MIDD)<=DEADLINE) return 1;
	else return 0;
}

/**
	* @brief       			遥控器映射到实际速度或者实际角度
	* @param[max]		    需要映射的最大值
	* @param[min]		    需要映射的最小值
	* @param[RCactual]	遥控器通道实际值
	* @param[RCmax]		  遥控器通道最大值
	* @param[RCmid]		  遥控器通道中值
	* @param[RCmin]		  遥控器通道最小值
	* @param[deadline]	遥控器死区
	* @retvel      1（不超限幅），0（超限幅）
*/
float rc_cali(int max,int min, int RC_actual, int RC_max, int RC_mid, int RC_min, int deadline)
{
	float value;
	if((RC_actual-RC_mid)>=deadline)
	{
		value=(float)(RC_actual-RC_mid-deadline)/(RC_max-RC_min);
		value*=max;
	}
	else if((RC_actual-RC_mid)<-deadline)
	{
		value=(float)(RC_actual-RC_mid+deadline)/(RC_max-RC_min);
		value*=max;
	}
	else
		value=0;
	return value;
}




