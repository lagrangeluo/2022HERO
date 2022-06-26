#ifndef __sent_task_H
#define __sent_task_H

#include "stm32f4xx.h"

uint8_t canTX_chassis(int16_t x,int16_t y,int8_t z,int8_t deviation);
uint8_t canTX_fric(int16_t left,int16_t right);
uint8_t canTX_trigger(uint8_t trigger);
uint8_t canTX_pitch(int16_t pitch);
uint8_t canTX_yaw(int16_t yaw,int16_t yaw_current);
uint8_t canTX_mode(uint8_t mode);

uint8_t canTX_UI(int pitch,int mode);
#endif
