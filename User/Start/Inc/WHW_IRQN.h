#ifndef __WHW_IRQN_H
#define __WHW_IRQN_H

#include "All_Init.h"
#include "Chassis_Task.h"
#include "Vision.h"
#include "stm32f4xx_hal_can.h"
extern void BSP_TIM_IRQHandler(TIM_HandleTypeDef *htim);
extern void BSP_UART_IRQHandler(UART_HandleTypeDef *huart);
void DWT_DelayUs(uint32_t us);
extern float dt_pc ; 

#endif
