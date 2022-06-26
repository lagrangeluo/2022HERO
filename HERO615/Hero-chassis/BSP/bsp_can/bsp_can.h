#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "can.h"
#include "chassis_move.h"
#include "trigger_turn.h"
#include "yaw_turn.h"

#define MODE_SWITCH        1
#define MODE_NO_SWITCH     0

	
#define CAN_3508Motor1_ID         0x201
#define CAN_3508Motor2_ID         0x202
#define CAN_3508Motor3_ID         0x203
#define CAN_3508Motor4_ID         0x204
#define CAN_GIMBAL_Y_ID           0x20B
#define SHOOT_MOTOR_TRIGGER_ID    0x208


#define LOOP_BACK_ID              0x003
#define TRIGGER_CONTROL_ID        0x005
#define YAW_CONTROL_ID            0x006
#define GIMBAL_CONTROL_ID         0x007
#define UI_ID                     0x010
#define MODE_RECEIVE_ID           0X009
	
uint8_t bsp_can_init(void);
uint8_t Can_Tx_Message(CAN_HandleTypeDef *hcan,uint8_t *mdata);
void canTX_gimbal_2(int16_t vx,int16_t vy,int16_t shoot_flag_t);
void send_gimbal_data_2(void);
extern float pich_angle;
extern int mode_now;
extern char M_DAta[5];

#endif

	
	

