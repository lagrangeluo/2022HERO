#include "tim3_cnt_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "RC_task.h"
#include "tim.h"
#include "bsp_imu.h"
#include "bsp_uart.h"
#include "can_receive.h"
#include "sent_task.h"

int time_count=0;
int IMU_cnt=0,start_flag=0,loop=0;
//0.1ms
/*
	* @ brief       TIM3主循环中断函数
	* @ param				none
	* @ retvel      none
*/
void TIM3_CNT_TASK()
{
	 if(IMU_cnt>3) 
	 {
		 start_flag=1;
		
	 }
	 else
	 {
		  canTX_chassis(0,0,0,0);
	  	canTX_mode(CHASSIS_REMOTE_CLOSE);
	 }
		 time_count++;

		 INS_task();
	
	  if(time_count%20==0)
	 {
		 control_mode_judge();
		 //DMA_Send();	 
	 }
	
	 if(time_count%7==0&&start_flag==1)
		 {
			 Gimbal_Task();
//			 shoot_task();
			 DMA_Send();
			 remote_chassis();	
		 }
				
	 if(time_count%5==0)
	 {
//		 remote_chassis();
		 if(KEY_MODE==KEY_OFF)
		 remote_control_data();
		 else
		 key_control_data();	 
		 shoot_task();
	 }
	 
	 if(time_count%70==0)
	 {
		 if(gimbal_set_mode==GIMBAL_ABSOLUTE_ANGLE) //A
		 canTX_UI(gimbal_p.IMU_actual_angle*100,1);
		 else  if(gimbal_set_mode==GIMBAL_RELATIVE_ANGLE) //F
		 canTX_UI(gimbal_p.IMU_actual_angle*100,2);
		 else  if(gimbal_set_mode==GIMBAL_TOP_ANGLE) //T
		 canTX_UI(gimbal_p.IMU_actual_angle*100,3);
	 }
	 if(time_count>=1000) 
	 {
		 time_count=0;
		 loop++;
		 if(start_flag==0)
			IMU_cnt++;
	 }

}
