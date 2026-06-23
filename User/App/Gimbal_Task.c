#include  "Gimbal_Task.h"
//云台pitch轴缺一个重力补偿,先限位来解决


static float Gimbal_NormalizeAngle(float angle)//角度归一化
{
    angle = fmodf(angle, 360.0f);
    if (angle >  180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}
static float Gimbal_Clamp(float val, float max, float min)//云台限幅
{
    if (val >  max) return max;
    if (val <  min) return min;
    return val;
}

uint8_t MOTOR_PID_Gimbal_INIT(MOTOR_Typdef *MOTOR)
{

    //Yaw轴PID
    float PID_P_Yaw[3] = { 1, 0, 0 };
    float PID_S_Yaw[3] = { 20, 0, 0 };

    PID_Init(&MOTOR->DJI_6020_Yaw.PID_P,
             YAW_MOTOR_MAX_OUT, YAW_MOTOR_MAX_IOUT,
             PID_P_Yaw, 0, 0, 0, 0, 0,
             Integral_Limit | ErrorHandle);

    PID_Init(&MOTOR->DJI_6020_Yaw.PID_S,
             YAW_MOTOR_MAX_OUT, YAW_MOTOR_MAX_IOUT,
             PID_S_Yaw, 0, 0, 0, 0, 0,
             Integral_Limit | ErrorHandle);


    //pitch轴PID
    float PID_P_Pitch[3] = { 1, 0, 0 };
    float PID_S_Pitch[3] = { 10,0.1, 0};

    PID_Init(&MOTOR->DJI_6020_Pitch.PID_P,
             PITCH_MOTOR_MAX_OUT, PITCH_MOTOR_MAX_IOUT,
             PID_P_Pitch, 0, 0, 0, 0, 0,
             Integral_Limit | ErrorHandle);

    PID_Init(&MOTOR->DJI_6020_Pitch.PID_S,
             PITCH_MOTOR_MAX_OUT, PITCH_MOTOR_MAX_IOUT,
             PID_S_Pitch, 0, 0, 0, 0, 0,
             Integral_Limit | ErrorHandle);

    return RUI_DF_READY;
}



static void Gimbal_PID_Calculate(MOTOR_Typdef *MOTOR, IMU_Data_t *IMU,
                                  float target_yaw, float target_pitch)
{
    float yaw_angle_err = Gimbal_NormalizeAngle(target_yaw - IMU->yaw);
    PID_Calculate(&MOTOR->DJI_6020_Yaw.PID_P,
                  IMU->yaw,          // 当前反馈（绝对角度）
                  target_yaw);       // 目标角度

    /* 外环输出作为内环目标，补偿底盘旋转：
      小陀螺时底盘转速会叠加到云台上，
      用 CONTAL->CG.YAW_SPEED（底盘yaw角速度）前馈补偿 */
    PID_Calculate(&MOTOR->DJI_6020_Yaw.PID_S,
                  (float)MOTOR->DJI_6020_Yaw.DATA.Speed_now,
                  MOTOR->DJI_6020_Yaw.PID_P.Output);

    PID_Calculate(&MOTOR->DJI_6020_Pitch.PID_P,
                  IMU->pitch,        // 当前反馈
                  target_pitch);     // 目标角度

    PID_Calculate(&MOTOR->DJI_6020_Pitch.PID_S,
                  (float)MOTOR->DJI_6020_Pitch.DATA.Speed_now,
                  MOTOR->DJI_6020_Pitch.PID_P.Output);
    DJI_Current_Ctrl(&hcan1,
                     0x1FF,
                     (int16_t)MOTOR->DJI_6020_Pitch.PID_S.Output,
                     (int16_t)MOTOR->DJI_6020_Yaw.PID_S.Output,
                     0,
                     0);
}



uint8_t gimbal_task(CONTAL_Typedef          *CONTAL,
                    RUI_ROOT_STATUS_Typedef  *Root,
                    MOTOR_Typdef            *MOTOR,
                    IMU_Data_t              *IMU)
{
    static uint8_t PID_INIT = RUI_DF_ERROR;
    //首次运行：初始化 PID
    if (PID_INIT != RUI_DF_READY)
    {
        PID_INIT = MOTOR_PID_Gimbal_INIT(MOTOR);
        return RUI_DF_ERROR;
    }

    // 遥控离线：云台锁定在当前位置（目标=当前）
    if (Root->RM_DBUS == RUI_DF_OFFLINE)
    {
        CONTAL->HEAD.Yaw   = IMU->yaw;
        CONTAL->HEAD.Pitch = IMU->pitch;
    }

    //Pitch 限位（防止撞车身）
    CONTAL->HEAD.Pitch = Gimbal_Clamp(CONTAL->HEAD.Pitch,
                                      PITCH_ANGLE_MAX,
                                      PITCH_ANGLE_MIN);

    /* 底盘跟随用的相对角度（给 Chassis_Task 用）
      = 云台相对底盘偏转角（编码器值转换为角度）
      底盘会读这个值来决定跟随方向 */
    CONTAL->CG.RELATIVE_ANGLE =-(int16_t)(CONTAL->CG.YAW_INIT_ANGLE - MOTOR->DJI_6020_Yaw.DATA.Angle_now);

    /* 底盘yaw角速度（给底盘前馈补偿用）*/
    CONTAL->CG.YAW_SPEED = MOTOR->DJI_6020_Yaw.DATA.Speed_now;

    /* 串级 PID 计算 + 发送 */
    Gimbal_PID_Calculate(MOTOR, IMU,
                         CONTAL->HEAD.Yaw,
                         CONTAL->HEAD.Pitch);

    return RUI_DF_READY;
}


//VT13版
void Gimbal_set_target_VT13(CONTAL_Typedef *CONTAL,VT13_Typedef *VT13,IMU_Data_t *IMU)
{
    CONTAL->HEAD.Yaw += VT13->Remote.Channel[3] * (YAW_RC_SPEED / 1024.0f);
    CONTAL->HEAD.Yaw  = Gimbal_NormalizeAngle(CONTAL->HEAD.Yaw);
    CONTAL->HEAD.Pitch += VT13->Remote.Channel[2] * (PITCH_RC_SPEED / 1024.0f);
    CONTAL->HEAD.Pitch  = Gimbal_Clamp(CONTAL->HEAD.Pitch,PITCH_ANGLE_MAX,PITCH_ANGLE_MIN);
}

void Gimbal_Set_target_DBUS(CONTAL_Typedef *CONTAL,DBUS_Typedef *DBUS,IMU_Data_t *IMU) {
    CONTAL->HEAD.Yaw+=DBUS->Remote.CH3*(YAW_RC_SPEED / 660.0f);
    CONTAL->HEAD.Yaw  = Gimbal_NormalizeAngle(CONTAL->HEAD.Yaw);
    CONTAL->HEAD.Pitch += DBUS->Remote.CH2 * (PITCH_RC_SPEED / 660.0f);
    CONTAL->HEAD.Pitch  = Gimbal_Clamp(CONTAL->HEAD.Pitch,PITCH_ANGLE_MAX,PITCH_ANGLE_MIN);
}


//
//
// void Gimbal_Set_Target_RC(CONTAL_Typedef *CONTAL,DBUS_Typedef *DBUS,IMU_Data_t *IMU)
// {
//     /* Yaw：左摇杆左右（CH2）累加
//      摇杆最大值 660 时，每个周期累加 YAW_RC_SPEED 度
//       实际转速 = YAW_RC_SPEED * (CH2/660) * 任务频率(Hz) */
//     CONTAL->HEAD.Yaw += DBUS->Remote.CH2 * (YAW_RC_SPEED / REMOTE_SCALE);
//
//     CONTAL->HEAD.Pitch += DBUS->Remote.CH1 * (PITCH_RC_SPEED / REMOTE_SCALE);
//
//     CONTAL->HEAD.Yaw = Gimbal_NormalizeAngle(CONTAL->HEAD.Yaw);
//     // Pitch 限位
//     CONTAL->HEAD.Pitch = Gimbal_Clamp(CONTAL->HEAD.Pitch,
//                                       PITCH_ANGLE_MAX,
//                                       PITCH_ANGLE_MIN);
// }
//
//
//  /*模式3：底盘跟随云台
// Yaw 目标锁定为 0（正前方），Pitch 由摇杆控制
//    此时底盘会用 PID 去追 CONTAL->CG.RELATIVE_ANGLE=0，
//    即让云台相对底盘的偏角为零， 底盘跟上云台*/
//
// void Gimbal_Set_Target_Follow(CONTAL_Typedef *CONTAL,
//                                DBUS_Typedef   *DBUS,
//                                IMU_Data_t     *IMU)
// {
//     /* Yaw：保持当前 IMU 方向不变（云台锁住，底盘来跟）
//       实际上目标就是当前 yaw，不累加 */
//     /* 如果需要摇杆微调方向，取消下面注释：
//       CONTAL->HEAD.Yaw += DBUS->Remote.CH2 * (YAW_RC_SPEED / REMOTE_SCALE);
//      *CONTAL->HEAD.Yaw = Gimbal_NormalizeAngle(CONTAL->HEAD.Yaw);
//      */
//
//     //Pitch：摇杆累加
//     CONTAL->HEAD.Pitch += DBUS->Remote.CH1 * (PITCH_RC_SPEED / REMOTE_SCALE);
//     CONTAL->HEAD.Pitch  = Gimbal_Clamp(CONTAL->HEAD.Pitch,
//                                        PITCH_ANGLE_MAX,
//                                        PITCH_ANGLE_MIN);
// }