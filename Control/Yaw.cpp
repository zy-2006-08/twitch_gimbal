#include "Yaw.h"
#include "SMC.h"
#include "RM.h"
#include "PID.h"
#include "DM.h"


#define YAW_ST 6600
UpDown_check_class UD_Yaw_Back(0);
SMC Yaw(28,50, 0, 0.01, 30000, 0.9, 1, 1),
    Yaw_Zm(30, 55, 0, 0.01, 30000, 0.9, 1, 1),
    Yaw_Back(15, 50, 0, 0.001, 16000, 0.9, 1, 1);
PID_class yaw_angle(20, 0, 0, 450, 0, 10, 500),
          yaw_speed(0.02, 0, 0, 10, 0, 0, 10),
          yaw_back_angle(30,0,10,450,0,10,500),
          yaw_back_speed(0.035,0,0,10,0,0,10),
          yaw_angle_zm(30,0,0,450,0,0,500),
          yaw_speed_zm(0.025,0,0,5,0,0,5);
static YAW yaw_instance;
YAW *yaw = &yaw_instance;
extern MOTOR_RM M6020_YAW;
extern MOTOR_DM DM_YAW;
extern float Zm_Yaw_Vel, Zm_Yaw_Acc;
static float error_yaw;
// 底盘小陀螺前馈的侧面补偿系数 (2026-10-12)
//   原前馈只按 cos (云台朝前/朝后) 调力度, 实测开摩擦轮小陀螺时 yaw 的峰谷
//   落在 Chassis_Spin_Rear_Weight 的上升/下降段 (云台朝侧面), 前后增益调不掉。
//   这里额外加一项 gain * sin, 可在 Ozone 里在线改, 不用重新烧录。
//   0 = 和原来完全一样。先试 +0.0005 / -0.0005, 哪个方向 yaw 摆幅变小就往哪边加。
volatile float g_spin_ff_sin_gain = 0.0f;

// ================= 底盘小陀螺扰动自适应抵消 (2026-10-12) =================
// 【为什么】手调前后增益 + sin 增益只能一次动一个方向, 找不到 cos/sin 同时最优的那一点。
//   这里让程序自己学: 每次调用看 yaw 偏差 e 和云台相对底盘的相位, 把两个系数往 e 变小的方向挪。
//   学到的力矩 = 方向 x (a*cos + b*sin), 只和"云台朝底盘哪边"有关, 不改原来的 SMC 和前馈。
// 【怎么用】Ozone 里:
//   1. 开摩擦轮转小陀螺, 把 g_spin_adapt_learn 改成 1, 看 g_spin_adapt_a / _b 慢慢变化, yaw 摆幅变小。
//   2. 几秒后 a/b 基本不动了, 就是学好了。把 learn 改回 0 = 停止学习但继续用学到的值。
//   3. g_spin_adapt_apply 改 0 = 不用学到的值(和原来一样), 方便对比。
//   4. 学好的 a/b 断电会丢: 把 Ozone 里的数抄到下面 g_spin_adapt_a / _b 的初始值里重新烧录。
// 【安全】a/b 各自限幅 g_spin_adapt_amp_max; 偏差 > g_spin_adapt_err_gate 时不学(换板/打舵时不乱学)。
//   如果开了学习以后 a/b 一直往一个方向涨到限幅、yaw 反而更晃, 把 g_spin_adapt_sign 改成 -1。
volatile uint8_t g_spin_adapt_learn = 0;        // 1 = 学习, 0 = 不学(上电默认不学, 行为和原来一样)
volatile uint8_t g_spin_adapt_apply = 1;        // 1 = 输出学到的力矩, 0 = 不输出
volatile uint8_t g_spin_adapt_auto_learn = 0;   // 1 = 自瞄模式下也学(目标在动, 会更吵), 默认只在手动模式学
volatile float g_spin_adapt_mu = 0.0005f;       // 学习速度 (Nm / 度 / 次), 太慢就加大, 来回晃就减小
volatile float g_spin_adapt_sign = 1.0f;        // 更新方向, 学反了改 -1
volatile float g_spin_adapt_phase_rad = 0.0f;   // 相位超前补偿(弧度), 一般不用动
volatile float g_spin_adapt_amp_max = 0.5f;     // a/b 各自限幅 (Nm)
volatile float g_spin_adapt_err_gate = 3.0f;    // 偏差超过它(度)不学
volatile float g_spin_adapt_min_speed = 100.0f; // 底盘转速低于它不学(底盘发来的单位, 正常最大约 520)
volatile float g_spin_adapt_a = -0.19f;           // 学到的 cos 系数 (Nm), 学好后抄进这里
volatile float g_spin_adapt_b = -0.043f;           // 学到的 sin 系数 (Nm), 学好后抄进这里
volatile float g_spin_adapt_torque = 0.0f;      // 观察用: 本次输出的自适应力矩 (Nm)
volatile float g_spin_adapt_err = 0.0f;         // 观察用: 本次 yaw 偏差 (度)
static const YAW::StateFunc kYawStateFuncTable[static_cast<u8>(YAW::State::COUNT)] = {
    &YAW::stateNORMAL,
    &YAW::stateBACKING,
    &YAW::stateAUTO_ZM,
    &YAW::stateZERO,
};
//转力矩
float gm6020to_torq(float u)
{
    float A = u / (16384.0f / 3.0f);
    float nm = A * 0.741f * 4.0f;
    return nm;
}
//Yaw轴模式切换
void YAW::yaw_mode_deal(u8 GIMBAL_088_State,u8 YK_Mode,u8 zm_request)
{
    if (GIMBAL_088_State == BMI088_OK &&
        (YK_Mode == CONTROL_MODE || YK_Mode == ONLY_GIMBAL || YK_Mode == SHOOT_MODE ||
         YK_Mode == PLAYER_MODE || YK_Mode == XTL_MODE) &&
        !zm_request)
    {
        currentMode = Mode::GYRO;
        YAW_Mode = GYRO_MODE;
    }
    else if (zm_request)
    {
        currentMode = Mode::AUTO;
        YAW_Mode = AUTO_MODE;
    }
    else
    {
        currentMode = Mode::PROTECT;
        YAW_Mode = PROTECT_MODE;
    }
}

void YAW::set_SMCref(f Target_Angle)
{
    Yaw.ref = Target_Angle;
    Yaw_Zm.ref = Target_Angle;
    Yaw_Back.ref = Target_Angle;
}
void YAW::set_Yaw_Angle(f Target_Angle)
{
    this->Target_Angle = Target_Angle;
}

void YAW::set_ChassisSpinState(f speed, bool enabled, f phase_sin, f phase_cos)
{
    // The chassis uses joystick-speed units (currently roughly 0..520).  A
    // bounded value prevents a malformed CAN frame from producing a large
    // feed-forward torque while still allowing the full configured range.
    if (speed > 1000.0f)
        speed = 1000.0f;
    else if (speed < -1000.0f)
        speed = -1000.0f;

    if (phase_sin > 1.0f)
        phase_sin = 1.0f;
    else if (phase_sin < -1.0f)
        phase_sin = -1.0f;
    if (phase_cos > 1.0f)
        phase_cos = 1.0f;
    else if (phase_cos < -1.0f)
        phase_cos = -1.0f;

    chassis_spin_speed_target = enabled ? speed : 0.0f;
    chassis_spin_phase_sin_target = enabled ? phase_sin : 0.0f;
    chassis_spin_phase_cos_target = enabled ? phase_cos : 1.0f;
    chassis_spin_enabled = enabled;
    chassis_spin_update_tick = HAL_GetTick();

    if (!enabled)
    {
        chassis_spin_speed_filtered = 0.0f;
        Chassis_Spin_FF_Torque = 0.0f;
        Chassis_Spin_Rear_Weight = 0.0f;
    }
}

f YAW::chassisSpinFeedforward(void)
{
    const uint32_t now = HAL_GetTick();
    const bool timed_out = (uint32_t)(now - chassis_spin_update_tick) > CHASSIS_SPIN_FF_TIMEOUT_MS;
    if (!chassis_spin_enabled || timed_out)
    {
        chassis_spin_speed_filtered = 0.0f;
        Chassis_Spin_FF_Torque = 0.0f;
        Chassis_Spin_Rear_Weight = 0.0f;
        return 0.0f;
    }

    // Smooth the command so entering/leaving XTL does not create a torque
    // step that would disturb the vision loop.
    chassis_spin_speed_filtered += 0.05f * (chassis_spin_speed_target - chassis_spin_speed_filtered);
    chassis_spin_phase_sin_filtered += 0.25f * (chassis_spin_phase_sin_target - chassis_spin_phase_sin_filtered);
    chassis_spin_phase_cos_filtered += 0.25f * (chassis_spin_phase_cos_target - chassis_spin_phase_cos_filtered);

    const f phase_norm = sqrtf(chassis_spin_phase_sin_filtered * chassis_spin_phase_sin_filtered +
                               chassis_spin_phase_cos_filtered * chassis_spin_phase_cos_filtered);
    f normalized_cos = (phase_norm > 0.1f) ? chassis_spin_phase_cos_filtered / phase_norm : 1.0f;
    if (normalized_cos > 1.0f)
        normalized_cos = 1.0f;
    else if (normalized_cos < -1.0f)
        normalized_cos = -1.0f;

    // Smoothly interpolate the gain: front centre -> 0, rear centre -> 1.
    // This matches the observed once-per-revolution mechanical disturbance
    // without changing the normal auto-aim controller gains.
    Chassis_Spin_Rear_Weight = 0.5f * (1.0f - normalized_cos);
    const f phase_gain = CHASSIS_SPIN_FF_FRONT_GAIN +
        (CHASSIS_SPIN_FF_REAR_GAIN - CHASSIS_SPIN_FF_FRONT_GAIN) * Chassis_Spin_Rear_Weight;

    // 侧面补偿: sin = +1/-1 对应云台朝底盘左/右侧, 见 g_spin_ff_sin_gain 注释。
    const f normalized_sin = (phase_norm > 0.1f) ? chassis_spin_phase_sin_filtered / phase_norm : 0.0f;

    f torque = CHASSIS_SPIN_FF_SIGN * (phase_gain + g_spin_ff_sin_gain * normalized_sin) * chassis_spin_speed_filtered;

    // ---- 自适应抵消 (见文件开头 g_spin_adapt_* 注释) ----
    {
        // 相位超前: 把 (cos, sin) 转过 phase_rad
        const f ph = g_spin_adapt_phase_rad;
        const f cph = cosf(ph), sph = sinf(ph);
        const f rc = normalized_cos * cph - normalized_sin * sph;
        const f rs = normalized_sin * cph + normalized_cos * sph;
        // 反转小陀螺时扰动方向也反, 用转速符号把两个方向统一成一套 a/b
        const f spd = chassis_spin_speed_filtered;
        const f dir = (spd >= 0.0f) ? 1.0f : -1.0f;
        const f abs_spd = fabsf(spd);
        // 进出小陀螺时跟着转速渐入, 不让学到的力矩一下全加上
        f ramp = abs_spd / 200.0f;
        if (ramp > 1.0f) ramp = 1.0f;

        // e > 0 时 SMC 输出负力矩, 所以前馈要往 -e 方向调: a -= mu*e*(dir*rc)
        const f e = GIMBAL_088.realAngle.yaw - Target_Angle;
        g_spin_adapt_err = e;
        const bool state_ok = (currentState == State::NORMAL) ||
                              (g_spin_adapt_auto_learn && currentState == State::AUTO_ZM);
        if (g_spin_adapt_learn && state_ok && abs_spd >= g_spin_adapt_min_speed &&
            fabsf(e) < g_spin_adapt_err_gate)
        {
            f ec = e;
            if (ec > 1.5f) ec = 1.5f;
            else if (ec < -1.5f) ec = -1.5f;
            const f step = g_spin_adapt_sign * g_spin_adapt_mu * ec * ramp;
            f a = g_spin_adapt_a - step * dir * rc;
            f b = g_spin_adapt_b - step * dir * rs;
            const f lim = g_spin_adapt_amp_max;
            if (a > lim) a = lim; else if (a < -lim) a = -lim;
            if (b > lim) b = lim; else if (b < -lim) b = -lim;
            g_spin_adapt_a = a;
            g_spin_adapt_b = b;
        }
        const f u_adapt = g_spin_adapt_apply ? ramp * dir * (g_spin_adapt_a * rc + g_spin_adapt_b * rs) : 0.0f;
        g_spin_adapt_torque = u_adapt;
        torque += u_adapt;
    }

    if (torque > CHASSIS_SPIN_FF_MAX_TORQUE)
        torque = CHASSIS_SPIN_FF_MAX_TORQUE;
    else if (torque < -CHASSIS_SPIN_FF_MAX_TORQUE)
        torque = -CHASSIS_SPIN_FF_MAX_TORQUE;
    Chassis_Spin_FF_Torque = torque;
    return Chassis_Spin_FF_Torque;
}

YAW::StateFunc YAW::stateFuncOf(State state)
{
    const u8 index = static_cast<u8>(state);
    if (index >= (sizeof(kYawStateFuncTable) / sizeof(kYawStateFuncTable[0])))
    {
        return &YAW::stateZERO;
    }
    return kYawStateFuncTable[index];
}

void YAW::switchState(State newState)
{
    currentState = newState;
    currentStateFunc = stateFuncOf(newState);
    if (currentStateFunc == &YAW::stateZERO && newState != State::ZERO)
    {
        currentState = State::ZERO;
    }
}

f YAW::runCurrentState(u8 jianshu_flag)
{
    if (currentStateFunc == nullptr)
    {
        switchState(State::ZERO);
    }
    return (this->*currentStateFunc)(jianshu_flag);
}

float YAW::stateNORMAL(u8 jianshu_flag)
{
    const f yaw_delta = jianshu_flag ?
        (f)LIMIT(YK.shubiao.x, -400, 400) / 1200.0f :
        (f)YK.yaogan.ch2 / 6600.0f;
    Target_Angle -= yaw_delta;

    if (UD_Yaw_Back.updata(YK.Pressed_Check(KEY_PRESSED_R)) == UpDown_check_rising)
    {
        Target_Angle += 180.0f;
        set_SMCref(Target_Angle);
        Communicate_Send_Flag_1 |= (0x0001 << 1);
        Back_Flag = 1;
        switchState(State::BACKING);
        return stateBACKING(jianshu_flag);
    }

    Back_Flag = 0;
    set_SMCref(Target_Angle);
    Yaw.SMC_Tick(GIMBAL_088.realAngle.yaw, GIMBAL_088.Anglespeed.Deal_yaw);
    Yaw_Out = gm6020to_torq(Yaw.u);
    return Yaw_Out;
}

float YAW::stateAUTO_ZM(u8 jianshu_flag)
{
    (void)jianshu_flag;
    set_SMCref(Target_Angle);
    Yaw_Zm.SMC_AngleSpeed(Target_Angle, Zm_Yaw_Vel, Zm_Yaw_Acc, GIMBAL_088.realAngle.yaw, GIMBAL_088.Anglespeed.Deal_yaw);
    Yaw_Out = gm6020to_torq(Yaw_Zm.u);
    return Yaw_Out;
}

float YAW::stateBACKING(u8 jianshu_flag)
{
    set_SMCref(Target_Angle);
    if (fabsf(Target_Angle - GIMBAL_088.realAngle.yaw) > 5.0f)
    {
        Yaw_Back.SMC_Tick(GIMBAL_088.realAngle.yaw, GIMBAL_088.Anglespeed.Deal_yaw);
        Yaw_Out = gm6020to_torq(Yaw_Back.u);
        return Yaw_Out;
    }

    Communicate_Send_Flag_1 &= ~(0x0001 << 1);
    Back_Flag = 0;
    switchState(State::NORMAL);
    return stateNORMAL(jianshu_flag);
}

float YAW::stateZERO(u8 jianshu_flag)
{
    (void)jianshu_flag;
    Target_Angle = GIMBAL_088.realAngle.yaw;
    set_SMCref(Target_Angle);
    Back_Flag = 0;
    Communicate_Send_Flag_1 &= ~(0x0001 << 1);
    Yaw_Out = 0.0f;
    return Yaw_Out;
}
float YAW::Yaw_Out_Interface(u8 jianshu_flag)
{
    // 同步全局 YAW_Mode 到状态机
    if (YAW_Mode == GYRO_MODE)
        currentMode = Mode::GYRO;
    else if (YAW_Mode == AUTO_MODE)
        currentMode = Mode::AUTO;
    else
        currentMode = Mode::PROTECT;

    State nextState = State::ZERO;

    switch (currentMode)
    {
        case Mode::GYRO:
            nextState = Back_Flag ? State::BACKING : State::NORMAL;
            break;
        case Mode::AUTO:
            Back_Flag = 0;
            Communicate_Send_Flag_1 &= ~(0x0001 << 1);
            nextState = State::AUTO_ZM;
            break;
        case Mode::PROTECT:
        default:
            nextState = State::ZERO;
            break;
    }
    if (currentState != nextState)
    {
        switchState(nextState);
    }
    const f state_output = runCurrentState(jianshu_flag);
    if (currentMode == Mode::PROTECT)
    {
        // Do not retain a stale compensation command while the gimbal is
        // disabled.  This also makes the first frame after re-enabling safe.
        chassis_spin_speed_filtered = 0.0f;
        Chassis_Spin_FF_Torque = 0.0f;
        Chassis_Spin_Rear_Weight = 0.0f;
        Yaw_Out = 0.0f;
    }
    else
    {
        Yaw_Out = state_output + chassisSpinFeedforward();
    }
    return Yaw_Out;
}
