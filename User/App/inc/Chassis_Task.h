#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "DBUS.h"
#include "MY_define.h"
#include "RUI_ROOT_INIT.h"
#include "Motors.h"
#include "Power_Ctrl.h"
#include "All_Init.h"
#include "VT13.h"

union gmTOch_typdef
{
    struct {
        int16_t vx:11;
        int16_t vy:11;
        int16_t vr:11;
        uint16_t key_q:1;
        uint16_t key_e:1;
        uint16_t key_r:1;
        uint16_t key_z:1;
        uint16_t key_x:1;
        uint16_t key_c:1;
        uint16_t key_v:1;
        uint16_t key_shift:1;
        uint16_t key_ctrl:1;
        uint16_t key_f:1;
        uint16_t key_g:1;
        uint16_t key_b:1;
        uint16_t romoteOnLine	:2;

        uint16_t S1:2;
        uint16_t S2:2;

        uint16_t supUSe :1;
        uint16_t pitch:1;
        uint16_t fire_wheel:1;
        uint16_t shoot:1;
        uint16_t vision:1;

        //		uint16_t DBUS_state:1;
    }dataNeaten;
    //CAN���͵�����
    //	uint8_t sendData[8];
    uint8_t getData[8];
};
union chTOgm_typdef
{
    struct {
        int16_t pitch;
        int16_t yaw;
        float time;
    }dataNeaten_angle;
    struct{
        uint64_t heat_last:16;
        uint64_t huanchongnengliang:8;
        uint64_t nowSpeed:8;
        uint64_t target:1;
        uint64_t visionMod:3;
        uint64_t visionState:1;
        uint64_t judgeState:1;
        uint64_t :0;
    }dataNeaten_another;

    uint8_t sendData[4];
    //	uint8_t getData[8];
};
struct CanCommunit_typedef
{
    union chTOgm_typdef chTOgm;
    union gmTOch_typdef gmTOch;
};

typedef union {
    float    yaw_abs;       // 云台绝对yaw角度（度，来自 IMU->YawTotalAngle）
    uint8_t  data[4];       // CAN发送用
} YawFrame_t;

extern YawFrame_t YawFrame;
static float NormalizeAngle(float angle);
static float Clamp(float val, float limit);
static void ApplyGimbalTransform(CONTAL_Typedef *CONTAL,DBUS_Typedef *DBUS,float  gimbal_deg);
static void ApplyGimbal_Transform(CONTAL_Typedef *CONTAL,VT13_Typedef VT13,float gimbal_deg);
static void OmniResolve(CONTAL_Typedef *CONTAL);
static void MecanumResolve(CONTAL_Typedef *CONTAL);
uint8_t Motor_PID_Chassis_Init(MOTOR_Typdef *MOTOR);
uint8_t Chassis_AIM_INIT(RUI_ROOT_STATUS_Typedef *Root, MOTOR_Typdef *MOTOR);
uint8_t chassis_task(CONTAL_Typedef *CONTAL,RUI_ROOT_STATUS_Typedef *Root,User_Data_T *User_data,model_t *model,CAP_RXDATA *CAP_GET,MOTOR_Typdef *MOTOR);
void Chassis_Normal(CONTAL_Typedef *CONTAL, DBUS_Typedef *DBUS, MOTOR_Typdef *MOTOR);
void Chassis_Gyroscope_VT13(CONTAL_Typedef *CONTAL, VT13_Typedef *VT13, IMU_Data_t *IMU);
void Chassis_Gyroscope_DBUS(CONTAL_Typedef *CONTAL, DBUS_Typedef *DBUS, IMU_Data_t *IMU) ;
void Chassis_Gyroscope(CONTAL_Typedef *CONTAL, VT13_Typedef *VT13, IMU_Data_t *IMU);
void Chassis_Follow_Gimbal(CONTAL_Typedef *CONTAL, VT13_Typedef *VT13, IMU_Data_t *IMU);
void Chassis_auto_changeMode(CONTAL_Typedef *CONTAL, IMU_Data_t *IMU,VT13_Typedef *VT13);
void Chassis_Auto_changeMode_DBUS(CONTAL_Typedef *CONTAL, IMU_Data_t *IMU,DBUS_Typedef *DBUS);
uint8_t ChassisTXResolve(User_Data_T *User_data);
uint8_t ChassisRXResolve(uint8_t * data,DBUS_Typedef *DBUS,RUI_ROOT_STATUS_Typedef *Root);
#endif
