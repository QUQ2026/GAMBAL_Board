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

    float PID_S_Yaw[3] = { 15.0f,  0.0f,     0.0f };
    float PID_P_Yaw[3] = { 15.0f,  0.00001f, 0.0f };

    PID_Init(&MOTOR->m_dm4310_y_t.PID_S, 10000.0f, 0.0f,
             PID_S_Yaw, 0.0f, 0.0f, 0.0f, 0.0f, 0,
             Integral_Limit);
    PID_Init(&MOTOR->m_dm4310_y_t.PID_P, 2000.0f, 50.0f,
             PID_P_Yaw, 0.0f, 0.0f, 0.0f, 0.0f, 0,
             Integral_Limit);

    float PID_S_Pitch[3] = { 15.0f, 0.0f, 0.0f };
    float PID_P_Pitch[3] = { 15.0f, 0.0f, 0.0f };

    PID_Init(&MOTOR->m_dm4310_p_t.PID_S, 10000.0f, 0.0f,
             PID_S_Pitch, 0.0f, 0.0f, 0.0f, 0.0f, 0,
             Integral_Limit);
    PID_Init(&MOTOR->m_dm4310_p_t.PID_P, 1000.0f, 50.0f,
             PID_P_Pitch, 0.0f, 0.0f, 0.0f, 0.0f, 0,
             Integral_Limit);

    return RUI_DF_READY;
}


// static void Gimbal_PID_Calculate(MOTOR_Typdef *MOTOR, IMU_Data_t *IMU,
//                                   float target_yaw, float target_pitch)
// {
//     float yaw_angle_err = Gimbal_NormalizeAngle(target_yaw - IMU->yaw);
//     PID_Calculate(&MOTOR->m_dm4310_y_t.PID_P,IMU->yaw,          // 当前反馈（绝对角度）
//         target_yaw);       // 目标角度
//
//     float v_des = MOTOR->m_dm4310_y_t.PID_P.Output* (3.14159265f / 180.0f) / 0.001f;
//
//     if (v_des >  YAW_V_MAX_OUTPUT) v_des =  YAW_V_MAX_OUTPUT;
//     if (v_des < -YAW_V_MAX_OUTPUT) v_des = -YAW_V_MAX_OUTPUT;
//     mit_ctrl(&hcan1, YAW_CAN_ID,
//              0.0f, v_des,
//             0.0f, YAW_KD_DAMP,
//             0.0f);
//     /* 外环输出作为内环目标，补偿底盘旋转：
//       小陀螺时底盘转速会叠加到云台上，
//       用 CONTAL->CG.YAW_SPEED（底盘yaw角速度）前馈补偿 */
//     // PID_Calculate(&MOTOR->m_dm4310_y_t.PID_S,
//     //               (float)MOTOR->m_dm4310_y_t.DATA.Speed_now,
//     //               MOTOR->m_dm4310_y_t.PID_P.Output);
//
//     PID_Calculate(&MOTOR->m_dm4310_p_t.PID_P,
//                   IMU->pitch,        // 当前反馈
//                   target_pitch);     // 目标角度
//
//     // PID_Calculate(&MOTOR->m_dm4310_p_t.PID_S,
//     //               (float)MOTOR->m_dm4310_p_t.DATA.Speed_now,
//     //               MOTOR->m_dm4310_p_t.PID_P.Output);
//
// }
uint8_t Gimbal_AIM_INIT(CONTAL_Typedef *CONTAL)
{
    for (int i = 0; i < 10; i++)
    {
        motor_mode(&hcan1, YAW_CAN_ID,   0, DM_CMD_MOTOR_MODE);  // 0xFC 使能Yaw
        osDelay(1);
        motor_mode(&hcan1, PITCH_CAN_ID, 0, DM_CMD_MOTOR_MODE);  // 0xFC 使能Pitch
        osDelay(1);
    }
    //Pitch 软限位（防止撞车身）,根据实际机械结构调整
    CONTAL->HEAD.Pitch_MIN = -20.0f;
    CONTAL->HEAD.Pitch_MAX =  20.0f;

    return RUI_DF_READY;
}


//VT13版
void Gimbal_set_target_VT13(CONTAL_Typedef *CONTAL,VT13_Typedef *VT13,IMU_Data_t *IMU)
{
    CONTAL->HEAD.Yaw += VT13->Remote.Channel[3] * (YAW_RC_SPEED / 1024.0f);
   // CONTAL->HEAD.Yaw  = Gimbal_NormalizeAngle(CONTAL->HEAD.Yaw);
    CONTAL->HEAD.Pitch += VT13->Remote.Channel[2] * (PITCH_RC_SPEED / 1024.0f);
    CONTAL->HEAD.Pitch  = Gimbal_Clamp(CONTAL->HEAD.Pitch,PITCH_ANGLE_MAX,PITCH_ANGLE_MIN);
}

void Gimbal_Set_target_DBUS(CONTAL_Typedef *CONTAL,DBUS_Typedef *DBUS,IMU_Data_t *IMU) {
    CONTAL->HEAD.Yaw+=DBUS->Remote.CH3*(YAW_RC_SPEED / 660.0f);
    CONTAL->HEAD.Yaw  = Gimbal_NormalizeAngle(CONTAL->HEAD.Yaw);
    CONTAL->HEAD.Pitch += DBUS->Remote.CH2 * (PITCH_RC_SPEED / 660.0f);
    CONTAL->HEAD.Pitch  = Gimbal_Clamp(CONTAL->HEAD.Pitch,PITCH_ANGLE_MAX,PITCH_ANGLE_MIN);
}

void Gimbal_Set_Target_RC(CONTAL_Typedef *CONTAL,DBUS_Typedef *DBUS, IMU_Data_t *IMU)
{
    //Yaw：直接累加，不归一化，保持和 YawTotalAngle 同一坐标系 ,以下累加的符号可能反了
    CONTAL->HEAD.Yaw -= (float)DBUS->Remote.CH3 * (YAW_RC_SPEED / 660.0f);
    CONTAL->HEAD.Pitch -= (float)DBUS->Remote.CH2 * (PITCH_RC_SPEED / 660.0f);
    CONTAL->HEAD.Pitch  = Gimbal_Clamp(CONTAL->HEAD.Pitch,CONTAL->HEAD.Pitch_MAX,CONTAL->HEAD.Pitch_MIN);

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

uint8_t gimbal_task(CONTAL_Typedef *CONTAL,RUI_ROOT_STATUS_Typedef *Root,MOTOR_Typdef  *MOTOR,IMU_Data_t *IMU)
{
    static uint8_t PID_INIT  = RUI_DF_ERROR;
    static uint8_t AIM_INIT  = RUI_DF_ERROR;

    if (PID_INIT != RUI_DF_READY)
    {
        PID_INIT = MOTOR_PID_Gimbal_INIT(MOTOR);
        return RUI_DF_ERROR;
    }

    if (AIM_INIT != RUI_DF_READY)
    {
        AIM_INIT = Gimbal_AIM_INIT(CONTAL);
        return RUI_DF_ERROR;
    }

   //VT13   的离线检测
    // if (Root->RM_DBUS == RUI_DF_OFFLINE)
    // {
    //     CONTAL->HEAD.Yaw   = IMU->YawTotalAngle;
    //     CONTAL->HEAD.Pitch = IMU->pitch;
    // }

    CONTAL->HEAD.Pitch = Gimbal_Clamp(CONTAL->HEAD.Pitch,CONTAL->HEAD.Pitch_MAX,CONTAL->HEAD.Pitch_MIN);
   //数据给底盘跟随
   // CONTAL->CG.RELATIVE_ANGLE =(int16_t)(MOTOR->m_dm4310_y_t.DATA.pos * (180.0f / 3.14159265f)* 8192.0f / 360.0f);
    //CONTAL->CG.YAW_SPEED = MOTOR->m_dm4310_y_t.DATA.vel;  // rad/s
    CONTAL->CG.RELATIVE_ANGLE =-(int16_t)(CONTAL->CG.YAW_INIT_ANGLE - MOTOR->m_dm4310_y_t.DATA.Angle_now);
    CONTAL->CG.YAW_SPEED = MOTOR->m_dm4310_y_t.DATA.Speed_now;
    PID_Calculate(&MOTOR->m_dm4310_y_t.PID_P,IMU->YawTotalAngle,CONTAL->HEAD.Yaw);

    PID_Calculate(&MOTOR->m_dm4310_y_t.PID_S,QEKF_INS.Gyro[2] * 50.0f,MOTOR->m_dm4310_y_t.PID_P.Output);

    PID_Calculate(&MOTOR->m_dm4310_p_t.PID_P,IMU->pitch,CONTAL->HEAD.Pitch);

    PID_Calculate(&MOTOR->m_dm4310_p_t.PID_S,IMU->gyro[1] * 50.0f,MOTOR->m_dm4310_p_t.PID_P.Output);

    DM_Motor_Send(&hcan1, 0x3FE,MOTOR->m_dm4310_y_t.PID_S.Output,MOTOR->m_dm4310_p_t.PID_S.Output,0, 0);

    return RUI_DF_READY;
}

uint8_t GimbalTXResolve_DBUS(CONTAL_Typedef          *CONTAL,
                              DBUS_Typedef            *DBUS,
                              RUI_ROOT_STATUS_Typedef *Root,
                              IMU_Data_t              *IMU)
{
    /* ---- 帧1：遥控控制量 ---- */
    CanCommunit_t.gmTOch.dataNeaten.vx = DBUS->Remote.CH1;
    CanCommunit_t.gmTOch.dataNeaten.vy = DBUS->Remote.CH0;
    CanCommunit_t.gmTOch.dataNeaten.vr = DBUS->Remote.Dial;

    CanCommunit_t.gmTOch.dataNeaten.S1 = DBUS->Remote.S1;
    CanCommunit_t.gmTOch.dataNeaten.S2 = DBUS->Remote.S2;

    CanCommunit_t.gmTOch.dataNeaten.romoteOnLine = Root->RM_DBUS;
    CanCommunit_t.gmTOch.dataNeaten.supUSe       = CONTAL->BOTTOM.CAP;

    CanCommunit_t.gmTOch.dataNeaten.pitch      = CONTAL->HEAD.Pitch > 5.0f ? 1 : 0;
    CanCommunit_t.gmTOch.dataNeaten.fire_wheel = CONTAL->SHOOT.Shoot_Speed;
   // CanCommunit_t.gmTOch.dataNeaten.shoot      = CONTAL->SHOOT.Shoot_State;
    CanCommunit_t.gmTOch.dataNeaten.vision     = 0;

    CanCommunit_t.gmTOch.dataNeaten.key_q     = DBUS->KeyBoard.Q;
    CanCommunit_t.gmTOch.dataNeaten.key_e     = DBUS->KeyBoard.E;
    CanCommunit_t.gmTOch.dataNeaten.key_c     = DBUS->KeyBoard.C;
    CanCommunit_t.gmTOch.dataNeaten.key_shift = DBUS->KeyBoard.Shift;
    CanCommunit_t.gmTOch.dataNeaten.key_ctrl  = DBUS->KeyBoard.Ctrl;
    CanCommunit_t.gmTOch.dataNeaten.key_f     = DBUS->KeyBoard.F;
    CanCommunit_t.gmTOch.dataNeaten.key_g     = DBUS->KeyBoard.G;

    canx_send_data(&hcan1, GIMBAL_kong, CanCommunit_t.gmTOch.getData);

    /* ---- 帧2：yaw绝对角度（float拆4字节）----
     * 用 YawTotalAngle（累计角度）而不是 yaw（±180°）
     * 底盘板坐标变换用这个值 */
    YawFrame.yaw_abs = IMU->YawTotalAngle;
    canx_send_data(&hcan1, GIMBAL_kong+1, YawFrame.data);

    return 1;
}