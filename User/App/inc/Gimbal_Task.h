#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "MY_define.h"
#include "RUI_ROOT_INIT.h"
#include "Motors.h"
#include "IMU_Task.h"
#include "All_Init.h"
#include "VT13.h"
#include "WHW_IRQN.h"
#include "controller.h"
static float Gimbal_Clamp(float val, float max, float min);
static float Gimbal_NormalizeAngle(float angle);
uint8_t MOTOR_PID_Gimbal_INIT(MOTOR_Typdef *MOTOR);
static void Gimbal_PID_Calculate(MOTOR_Typdef *MOTOR, IMU_Data_t *IMU,
                                  float target_yaw, float target_pitch);
uint8_t gimbal_task(CONTAL_Typedef *CONTAL,RUI_ROOT_STATUS_Typedef  *Root,MOTOR_Typdef  *MOTOR,IMU_Data_t  *IMU);
//void Gimbal_Set_Target_RC(CONTAL_Typedef *CONTAL,DBUS_Typedef *DBUS,IMU_Data_t *IMU);
//void Gimbal_Set_Target_Follow(CONTAL_Typedef *CONTAL,DBUS_Typedef *DBUS,IMU_Data_t  *IMU);
void Gimbal_set_target_VT13(CONTAL_Typedef *CONTAL,VT13_Typedef *VT13,IMU_Data_t *IMU);
void Gimbal_Set_target_DBUS(CONTAL_Typedef *CONTAL,DBUS_Typedef *DBUS,IMU_Data_t *IMU);
#endif
