#include "vision_task.h"
#include "remote_control.h"
#include "gimbal_task.h"

VISION_t vision_mode=VISION_OFF;
VISION_GET_t vision_sent;
float vision_yaw,vision_pitch;


void Vision_Task(void)
{
	if(vision_mode==VISION_OFF)
	{
		gimbal_y.add_angle=rc_sent.yaw.target_angle;
		gimbal_y.target_speed=rc_sent.yaw.target_speed;
		
		gimbal_p.add_angle= rc_sent.pitch.target_angle;
		gimbal_p.target_speed=rc_sent.pitch.target_speed;
	}
	else if(vision_mode==VISION_ON)
	{
		gimbal_y.add_angle=vision_sent.yaw.target_angle;
		gimbal_y.target_speed=vision_sent.yaw.target_speed;
		
		gimbal_p.add_angle=vision_sent.pitch.target_angle;
		gimbal_p.target_speed=vision_sent.pitch.target_speed;
	}
}


