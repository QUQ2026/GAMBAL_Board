#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "DBUS.h"
#include "MY_define.h"
#include "RUI_ROOT_INIT.h"
#include "Motors.h"
#include "Power_Ctrl.h"
static float NormalizeAngle(float angle);
static float Clamp(float val, float limit);

static void ApplyGimbalTransform(CONTAL_Typedef *CONTAL,
                                 DBUS_Typedef   *DBUS,
                                 float           gimbal_deg);
static void OmniResolve(CONTAL_Typedef *CONTAL);
uint8_t Motor_PID_Chassis_Init(MOTOR_Typdef *MOTOR);
uint8_t Chassis_AIM_INIT(RUI_ROOT_STATUS_Typedef *Root, MOTOR_Typdef *MOTOR);
uint8_t chassis_task(CONTAL_Typedef *CONTAL,
                   RUI_ROOT_STATUS_Typedef *Root,
                   User_Data_T *User_data,
                   model_t *model,
                   CAP_RXDATA *CAP_GET,
                   MOTOR_Typdef *MOTOR);
void Chassis_Normal(CONTAL_Typedef *CONTAL, DBUS_Typedef *DBUS, MOTOR_Typdef *MOTOR);
void Chassis_Gyroscope(CONTAL_Typedef *CONTAL, DBUS_Typedef *DBUS, IMU_Data_t *IMU);
void Chassis_Follow_Gimbal(CONTAL_Typedef *CONTAL, DBUS_Typedef *DBUS, IMU_Data_t *IMU);

#endif
