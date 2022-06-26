#include "sent_task.h"
#include "can.h"
#include "usart.h"
uint8_t canTX_chassis(int16_t x,int16_t y,int8_t z,int8_t deviation)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x007;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=x>>8;
	data[1]=x&0xff;
	data[2]=y>>8;
	data[3]=y&0xff;
	data[4]=z>>8;
	data[5]=z&0xff;
	data[6]=deviation>>8;
	data[7]=deviation&0xff;
	HAL_CAN_AddTxMessage(&hcan2, &canFrame, data, &temp);
	
	return temp;	
}                                                                                                                   

uint8_t canTX_mode(uint8_t mode)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x009;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=mode;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	data[4]=0;
	data[5]=0;
	data[6]=0;
	data[7]=0;
	HAL_CAN_AddTxMessage(&hcan2, &canFrame, data, &temp);
	
	return temp;	
}  

uint8_t canTX_UI(int pitch,int mode)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x010;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=pitch>>8;
	data[1]=pitch&0xff;
	data[2]=mode>>8;
	data[3]=mode&0xff;
	data[4]=0;
	data[5]=0;
	data[6]=0;
	data[7]=0;
	HAL_CAN_AddTxMessage(&hcan2, &canFrame, data, &temp);
	
	return temp;	
}   


uint8_t canTX_pitch(int16_t pitch)              //pitch采用3508电机驱动方案
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x1ff;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=pitch>>8;
	data[1]=pitch&0xff;
	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data,&temp);
	
	return temp;
}


uint8_t canTX_yaw(int16_t yaw,int16_t yaw_current)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x006;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=yaw>>8;
	data[1]=yaw&0xff;
	data[2]=yaw_current>>8;
	data[3]=yaw_current&0xff;
	data[4]=0;
	data[5]=0;
	data[6]=0;
	data[7]=0;
	HAL_CAN_AddTxMessage(&hcan2, &canFrame, data,&temp);
	
	return temp;
}


uint8_t canTX_fric(int16_t left,int16_t right)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x200;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=left>>8;
	data[1]=left&0xff;
	data[2]=right>>8;
	data[3]=right&0xff;
//	data[4]=0;
//	data[5]=0;
	data[6]=0;
	data[7]=0;
	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data,&temp);
	
	return temp;
}

uint8_t canTX_trigger(uint8_t trigger)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x005;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;

	data[0]=trigger;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	data[4]=0;
	data[5]=0;
	data[6]=0;
	data[7]=0;
	HAL_CAN_AddTxMessage(&hcan2, &canFrame, data,&temp);
	
	return temp;
}
