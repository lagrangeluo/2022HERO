#ifndef __BSP_UART_H
#define __BSP_UART_H


#define DMA_REC_LEN    6


void uart_init(void);
void DMA_Send(void);

typedef enum
{
	ON_RESET=0,
	OFF=1,
}VISION_RESET; //������־λ

typedef enum
{
	ON_FAR=0,
	OFF_FAR=1,
}VISION_FAR_SHOOT; //ǰ��վ������־λ

extern VISION_RESET VISION_RESET_FLAG;
extern VISION_FAR_SHOOT VISION_RAR_FLAG;
extern int shoot_vision_flag;
#endif