#include "Shoot.h"
#include "PID.h"
#include "RM.h"
#include "communication.h"
#include "RM_Lib.h"
extern MOTOR_RM M3508_MCL_Right,
    M3508_MCL_Left,
    M2006_BP;
extern USER_CAN  CAN_1;
extern u16 remain_heat, shooter_id1_17mm_cooling_limit, shooter_id1_17mm_cooling_heat;
PID_class M3508_MCL_Left_Speed(20, 0, 0, 15000, 0, 0, 15000),
    M3508_MCL_Right_Speed(20, 0, 0, 15000, 0, 0, 15000);
PID_class M2006_BP_Speed(13, 0, 0, 16000, 0, 0, 16000),
    M2006_BP_Angle(0.4, 0, 0, 16000, 0, 10000, 16000),
    M2006_BP_Con_Speed(13, 0, 0, 16000, 0, 0, 16000),
    M2006_BP_Con_Angle(0.45, 0, 0, 16000, 0, 0, 16000);
UpDown_check_class UD_BP_ON(0), UD_BP_ALIGN(0), UD_BP_PREFAB_RELEASE(0), UD_BP_DF(0);
UpDown_check_class UD_PR_BP(0),UD_BP_FIRE(0);
MCL *mcl = new MCL();
BP *bp = new BP();
static u8 buff_flag = 1;
extern u8 buff_mode;
static uint16_t test_flag_1,test_flag_2;

// ================= AutoFire: level-triggered + rate limit (TIM7 1kHz tick) =================
// 2026-09-27. 原来视觉开火只认 SuperPower.mode==2 的上升沿(UD_BP_FIRE):
//   目标静止时视觉持续发 2, 只有第一帧是上升沿 -> 只打一发就停;
//   目标转起来 fire 随板子进出容差窗口反复翻转 -> 每翻一次打一发, 射频不受控.
// 现在: mode==2 表示“允许开火”(电平), 电控按固定间隔出弹.
//
// 计时基准: bp_fire_tick_1ms, 在 TIM7 中断里的 BP_time_out() 中每次 +1.
//   TIM7: APB1 定时器时钟 84MHz / (Prescaler 84) / (Period 1000) = 精确 1kHz.
//   不用 HAL_GetTick(): 它依赖 SysTick 中断, 被高优先级中断抢占时会掉拍.
//   不用调用次数: BP_deal 在 CAN2 接收回调里调用, 频率随总线负载变化.
//
// AUTOFIRE_INTERVAL_TICKS: 两发最小间隔, 单位 TIM7 tick(1ms). 40 = 25Hz 上限.
//   拨盘物理上未必转得这么快: 一发 BOPAN_ANGLE=36864 计数 = 电机 4.5 圈,
//   25Hz 需约 6750rpm, 接近 M2006 空载极限. 下面的“拨盘跟上”门控会把实际射频
//   自动压到拨盘能达到的速度, 不会因为跟不上而把目标角无限累加.
// 热量限制暂未加(按要求先不管热量), 上场前必须补上.
#define AUTOFIRE_INTERVAL_TICKS  40u

// ================= 临时测试: 定量连发 (2026-10-04) =================
// 目的: 验证"拨盘推 N 格 = 打出 N 颗弹丸"。
// 用法(SHOOT_MODE): ch1>600 开摩擦轮 -> ch0 往上推过 600 一次(上升沿) ->
//   拨盘按自瞄同样的方式(AutoFire_Shoot, 同一个位置环)每 BP_BURST_TEST_INTERVAL_MS 推一格,
//   推满 BP_BURST_TEST_SHOTS 格自动停。中途松开 ch0 也会继续打完; 关摩擦轮/报错立即中止。
// 测试期间 ch0>600 不再走原来的遥控连发(Continuous_Target_Angle -= Shot_SP_1*15)。
// 测完把 BP_BURST_TEST 改成 0 恢复原逻辑。
#define BP_BURST_TEST               0
#define BP_BURST_TEST_SHOTS         50u
#define BP_BURST_TEST_INTERVAL_MS   200u   // 200ms = 中速自瞄每块装甲板的间隔; 改 40 测最高射频

volatile uint32_t bp_fire_tick_1ms = 0;   // TIM7 1kHz, 在 BP_time_out() 中递增
static uint32_t autofire_last_tick = 0;
// 拨盘每推进一格 +1, 由 Mini_PC_SendData() 回传视觉(声明见 communication.h)
volatile uint16_t bp_shot_count = 0;

// ===== Ozone scope mirrors (debug only) =====
// bp 是 new 出来的指针, Ozone Data Sampling 只能画固定地址的全局变量, 所以每个拨盘控制周期
// (CAN2 收到 M2006 反馈 -> BP_deal) 把关心的量拷一份到这里。单位: 电机编码器累计计数,
// 一格 BOPAN_ANGLE = 36864。目标往负方向走: 每出一发目标 -36864。
//   err = target - real, 负 = 还没走到, 正 = 冲过了目标(过冲)。
volatile float dbg_bp_target_one = 0;   // bp->One_Target_Angle        单发/自瞄 位置环目标
volatile float dbg_bp_target_con = 0;   // bp->Continuous_Target_Angle 连发 位置环目标
volatile float dbg_bp_real       = 0;   // M2006_BP.mang_inf           实际累计角度
volatile float dbg_bp_err        = 0;   // 当前所用状态的 target - real
volatile float dbg_bp_speed      = 0;   // M2006_BP.sp                 实际转速 rpm
volatile float dbg_bp_speed_ref  = 0;   // 当前所用状态的角度环输出(速度环目标)
volatile float dbg_bp_out        = 0;   // 速度环输出(发给电机的电流值)
volatile float dbg_bp_state      = 0;   // 0=ZERO 1=ONE_SHOT 2=CONTINUOUS
volatile float dbg_bp_phase      = 0;   // real 对一格取余(0~36863), 看每次停位是否一致
volatile float dbg_bp_shot       = 0;   // bp_shot_count, 每推一格 +1, 做时间标记
volatile float dbg_bp_fire_cmd   = 0;   // 视觉开火信号 mode==2 且自瞄开启

#if BP_BURST_TEST
static UpDown_check_class UD_BP_BURST(0);
static uint32_t burst_last_tick = 0;
volatile uint16_t dbg_bp_burst_left  = 0;   // 本轮还剩几格没推, 0 = 空闲
volatile uint16_t dbg_bp_burst_done  = 0;   // 本轮已下令推了几格
volatile float    dbg_bp_burst_start = 0;   // 本轮开始时 mang_inf
volatile float    dbg_bp_burst_turns = 0;   // 本轮拨盘实际转过的格数 = (开始角 - 当前角) / 36864
#endif

// 返回 1 表示本周期应该出一发. 只判断条件, 不改拨盘目标.
static u8 AutoFire_Ready(void)
{
    const uint32_t now = bp_fire_tick_1ms;
    if ((uint32_t)(now - autofire_last_tick) < AUTOFIRE_INTERVAL_TICKS) return 0;

    // 拨盘跟上门控: 目标角比当前角超前不超过一发才允许再加一发.
    // 目标往负方向减, 所以超前量 = 当前角 - 目标角.
    const double lag = M2006_BP.mang_inf - bp->One_Target_Angle;
    if (lag > (double)BOPAN_ANGLE) return 0;
    return 1;
}

// ================= 拨弹 -> 出膛 延迟测量 (debug, 2026-10-04) =================
// 每次下令推一格时记下时刻(BP_LatencyMark), 摩擦轮检测到下一颗弹丸时(BP_LatencyHit)
// 算出时间差, 单位 ms, 直接在 Ozone Data Sampling 表里读:
//   dbg_shot_lat_ms   最近一颗的延迟
//   dbg_shot_lat_avg  平均值 (= 视觉 laser_latency 要额外补的"拨弹到出膛"时间)
//   dbg_shot_lat_min / max  最小 / 最大
//   dbg_shot_lat_n    已统计颗数
// 下令后 BP_LAT_TIMEOUT_MS 内没测到弹丸算丢失(dbg_shot_lat_miss++), 不计入平均。
// 清零: Ozone 里把 dbg_shot_lat_n 改成 0, 下一颗会重置统计。
#define BP_LAT_QUEUE      8u
#define BP_LAT_TIMEOUT_MS 300u
static uint32_t bp_lat_q[BP_LAT_QUEUE];
static uint8_t  bp_lat_head = 0, bp_lat_tail = 0;
volatile float    dbg_shot_lat_ms  = 0;
volatile float    dbg_shot_lat_avg = 0;
volatile float    dbg_shot_lat_min = 0;
volatile float    dbg_shot_lat_max = 0;
volatile uint32_t dbg_shot_lat_n   = 0;
volatile uint32_t dbg_shot_lat_miss = 0;
static double     bp_lat_sum = 0;

static void BP_LatencyMark(void)
{
    const uint8_t next = (uint8_t)((bp_lat_head + 1u) % BP_LAT_QUEUE);
    if (next == bp_lat_tail) { bp_lat_tail = (uint8_t)((bp_lat_tail + 1u) % BP_LAT_QUEUE); dbg_shot_lat_miss++; }
    bp_lat_q[bp_lat_head] = bp_fire_tick_1ms;
    bp_lat_head = next;
}

static void BP_LatencyHit(uint32_t now)
{
    // 丢掉超时没出膛的旧记录
    while (bp_lat_tail != bp_lat_head && (uint32_t)(now - bp_lat_q[bp_lat_tail]) > BP_LAT_TIMEOUT_MS)
    {
        bp_lat_tail = (uint8_t)((bp_lat_tail + 1u) % BP_LAT_QUEUE);
        dbg_shot_lat_miss++;
    }
    if (bp_lat_tail == bp_lat_head) return;   // 没有对应的下令(比如手动拨弹), 不统计
    const float lat = (float)(uint32_t)(now - bp_lat_q[bp_lat_tail]);
    bp_lat_tail = (uint8_t)((bp_lat_tail + 1u) % BP_LAT_QUEUE);

    if (dbg_shot_lat_n == 0) { bp_lat_sum = 0; dbg_shot_lat_min = lat; dbg_shot_lat_max = lat; }
    dbg_shot_lat_ms = lat;
    bp_lat_sum += lat;
    dbg_shot_lat_n++;
    dbg_shot_lat_avg = (float)(bp_lat_sum / (double)dbg_shot_lat_n);
    if (lat < dbg_shot_lat_min) dbg_shot_lat_min = lat;
    if (lat > dbg_shot_lat_max) dbg_shot_lat_max = lat;
}

// 出一发: 在上一发目标的基础上再推进一格(不重新取当前角, 避免丢掉未走完的行程).
static void AutoFire_Shoot(void)
{
    // 目标角如果落后当前角(例如刚从别的模式切过来), 先对齐到当前角
    if (bp->One_Target_Angle > M2006_BP.mang_inf) bp->One_Target_Angle = M2006_BP.mang_inf;
    bp->One_Target_Angle -= BOPAN_ANGLE;
    bp->Continuous_Target_Angle = bp->One_Target_Angle;
    // Continuous_shooting_flag 是 BP 的 private 成员, 由调用方(BP 成员函数内)清零
    autofire_last_tick = bp_fire_tick_1ms;
    bp_shot_count++;
    BP_LatencyMark();
}

// ================= 摩擦轮掉速测弹 (debug, 2026-10-04) =================
// 没有裁判系统测速模块时, 用摩擦轮掉速判断"真的有一颗弹丸从摩擦轮中间挤过去了"。
// 每次两个摩擦轮都收到新反馈(约 1kHz)调用一次。
//   掉速 drop = |目标转速| - |实际转速|, 取左右两轮中较大的一个。
//   drop 超过 MCL_SHOT_DROP_ON 记一颗; 回到 MCL_SHOT_DROP_OFF 以下才允许记下一颗(迟滞, 防一颗算两次)。
//   只在摩擦轮已稳定(目标转速到位且 |目标| > MCL_SHOT_MIN_SPEED)时检测, 启停斜坡期间不计。
// 阈值是初值, 需要按 Ozone 里 dbg_mcl_drop 的实际波形调: 一颗弹丸的掉速峰值应明显高于 ON,
// 空转时的波动应低于 OFF。
#define MCL_SHOT_DROP_ON     300.0f   // rpm, 掉速超过这个算一颗
#define MCL_SHOT_DROP_OFF    120.0f   // rpm, 恢复到这个以下才重新武装
#define MCL_SHOT_MIN_SPEED   3000.0f  // rpm, 摩擦轮目标低于这个不检测

volatile uint16_t mcl_shot_count   = 0;   // 摩擦轮检测到的弹丸数(真实出弹)
volatile float    dbg_mcl_r_sp     = 0;   // 右摩擦轮实际转速 rpm
volatile float    dbg_mcl_l_sp     = 0;   // 左摩擦轮实际转速 rpm
volatile float    dbg_mcl_drop     = 0;   // 本周期掉速(两轮取大) rpm
volatile float    dbg_mcl_drop_max = 0;   // 上一颗弹丸的掉速峰值, 用来定阈值
volatile float    dbg_mcl_shot     = 0;   // mcl_shot_count 的 float 镜像, 方便和 dbg_bp_shot 同图
volatile uint32_t dbg_mcl_shot_tick = 0;  // 最近一颗弹丸被检测到的 TIM7 tick(ms)

static void MCL_ShotDetect(float target_r, float target_l)
{
    static bool armed = true;
    static float peak = 0.0f;

    const float r_sp = (float)M3508_MCL_Right.sp;
    const float l_sp = (float)M3508_MCL_Left.sp;
    dbg_mcl_r_sp = r_sp;
    dbg_mcl_l_sp = l_sp;

    const float tr = fabsf(target_r), tl = fabsf(target_l);
    const float full = fabsf(mcl->MCL_Speed + mcl->MCL_Change);
    // 斜坡还没到位或摩擦轮没开: 不检测, 状态复位
    if (mcl->Mode == 0 || tr < MCL_SHOT_MIN_SPEED || tl < MCL_SHOT_MIN_SPEED ||
        tr < full - 1.0f || tl < full - 1.0f)
    {
        dbg_mcl_drop = 0.0f;
        armed = true;
        peak = 0.0f;
        return;
    }

    const float drop_r = tr - fabsf(r_sp);
    const float drop_l = tl - fabsf(l_sp);
    const float drop = (drop_r > drop_l) ? drop_r : drop_l;
    dbg_mcl_drop = drop;

    if (armed)
    {
        if (drop > MCL_SHOT_DROP_ON)
        {
            armed = false;
            peak = drop;
            mcl_shot_count++;
            dbg_mcl_shot = (float)mcl_shot_count;
            dbg_mcl_shot_tick = bp_fire_tick_1ms;
            BP_LatencyHit(bp_fire_tick_1ms);
        }
    }
    else
    {
        if (drop > peak) peak = drop;
        if (drop < MCL_SHOT_DROP_OFF)
        {
            armed = true;
            dbg_mcl_drop_max = peak;
        }
    }
}

static float ramp_towards(float current, float target, float max_delta)
{
    float delta = target - current;
    if (delta > max_delta)
        return current + max_delta;
    if (delta < -max_delta)
        return current - max_delta;
    return target;
}

void slope(float *rec, float target, float slow_Inc)
{
    if (abs(*rec) - abs(target) < 0)
    {
    }
    if (abs(*rec) - abs(target) > 0)
        slow_Inc *= 2.5f;

    if (abs(*rec - target) < slow_Inc)
        *rec = target;
    else
    {
        if ((*rec) > target)
            (*rec) = target;
        if ((*rec) < target)
            (*rec) += slow_Inc;
    }
}
void MCL::MCL_while_layer(u8 YK_Mode)
{
    if (YK_Mode == SHOOT_MODE)
    {
        bp->Mode = GYRO_MODE;
        bp->currentMode = BP::FsmMode::GYRO;
        if (YK.yaogan.ch1 > P_YG)
        {
            mcl->Mode = MCL_ON;
        }
        else
        {
            mcl->Mode = MCL_OFF;
        }
    }
    else if (YK_Mode == PLAYER_MODE)
    {
        bp->Mode = AUTO_MODE;
        bp->currentMode = BP::FsmMode::AUTO;
        mcl->Mode = MCL_ON;
    }
    else if (YK_Mode == PROTECT_MODE)
    {
        bp->Mode = PROTECT_MODE;
        bp->currentMode = BP::FsmMode::PROTECT;
        mcl->Mode = MCL_OFF;
        bp->One_Target_Angle = bp->Continuous_Target_Angle = M2006_BP.mang_inf;
    }
    else
    {
        bp->Mode = PROTECT_MODE;
        bp->currentMode = BP::FsmMode::PROTECT;
        mcl->Mode = MCL_OFF;
    }
}

f *MCL::MCL_deal(u8 YK_Mode)
{
    if (M3508_MCL_Right.update() == HAL_OK)
    {
        mcl->Update_Flag |= 0x1;
        test_flag_1++;
    }
    if (M3508_MCL_Left.update() == HAL_OK)
    {
        mcl->Update_Flag |= 0x2;
        test_flag_2++;
    }
    if ((mcl->Update_Flag & 0x3) == 0x3)
    {
        const float desired_speed = mcl->Mode ? (mcl->MCL_Speed + mcl->MCL_Change) : 0.0f;
        const uint32_t now = bp_fire_tick_1ms;   // TIM7 1kHz, 不用 HAL_GetTick
        uint32_t dt_ms = 0;

        if (mcl->Last_Ramp_Tick != 0)
        {
            dt_ms = now - mcl->Last_Ramp_Tick;
            if (dt_ms > 20)
                dt_ms = 20;
        }
        mcl->Last_Ramp_Tick = now;

        const float ramp_speed = fabsf(mcl->MCL_Speed + mcl->MCL_Change);
        const float accel_step = (ramp_speed / 500.0f) * dt_ms;
        const float decel_step = (ramp_speed / 100.0f) * dt_ms;
        const float right_step = (fabsf(desired_speed) > fabsf(mcl->Target_R)) ? accel_step : decel_step;
        const float left_target = -desired_speed;
        const float left_step = (fabsf(left_target) > fabsf(mcl->Target_L)) ? accel_step : decel_step;

        mcl->Target_R = ramp_towards(mcl->Target_R, desired_speed, right_step);
        mcl->Target_L = ramp_towards(mcl->Target_L, left_target, left_step);

        if (mcl->Mode)
        {
            Charge_ON;
        }
        else
        {
            Charge_OFF;
        }

        M3508_MCL_Right_Speed.PID_new_update(mcl->Target_R, M3508_MCL_Right.sp);
        mcl->PID_OUT[0] = M3508_MCL_Right_Speed.OUT_PID;
        M3508_MCL_Left_Speed.PID_new_update(mcl->Target_L, M3508_MCL_Left.sp);
        mcl->PID_OUT[1] = M3508_MCL_Left_Speed.OUT_PID;
        mcl->Update_Flag = 0;
        MCL_ShotDetect(mcl->Target_R, mcl->Target_L);
    }
    return mcl->PID_OUT;
}

static const BP::StateFunc kBPStateFuncTable[static_cast<u8>(BP::FsmState::COUNT)] = {
    &BP::stateZERO,
    &BP::stateOneShot,
    &BP::stateContinuous,
};

static BP::StateFunc stateFuncOf(BP::FsmState state)
{
    const u8 index = static_cast<u8>(state);
    if (index >= (sizeof(kBPStateFuncTable) / sizeof(kBPStateFuncTable[0])))
    {
        return &BP::stateZERO;
    }
    return kBPStateFuncTable[index];
}

void BP::switchState(FsmState newState)
{
    currentState = newState;
    currentStateFunc = stateFuncOf(newState);
    if (currentStateFunc == &BP::stateZERO && newState != FsmState::ZERO)
    {
        currentState = FsmState::ZERO;
    }
}

static f runCurrentState(BP *self)
{
    if (self->currentStateFunc == nullptr)
    {
        self->switchState(BP::FsmState::ZERO);
    }
    return (self->*self->currentStateFunc)();
}

// ZERO: 无射击
f BP::stateZERO(void)
{
    PID_OUT = 0;
    return PID_OUT;
}

// ONE_SHOT: 单发
f BP::stateOneShot(void)
{
    M2006_BP_Angle.PID_new_update(One_Target_Angle, M2006_BP.mang_inf);
    M2006_BP_Speed.PID_new_update(M2006_BP_Angle.OUT_PID, M2006_BP.sp);
    PID_OUT = M2006_BP_Speed.OUT_PID;
    return PID_OUT;
}

// CONTINUOUS: 连发
f BP::stateContinuous(void)
{
    M2006_BP_Con_Angle.PID_new_update(Continuous_Target_Angle, M2006_BP.mang_inf);
    M2006_BP_Con_Speed.PID_new_update(M2006_BP_Con_Angle.OUT_PID, M2006_BP.sp);
    PID_OUT = M2006_BP_Con_Speed.OUT_PID;
    return PID_OUT;
}

// 状态机入口：处理目标角度 + 根据 Mode 决定 State + PID 输出
f BP::BP_Out_Interface(u8 YK_Mode, u8 jianshu_flag)
{
    // 目标角度更新（原 BP_deal 逻辑）
    bool burst_owns = false;
#if BP_BURST_TEST
    if (YK_Mode == SHOOT_MODE)
    {
        const bool ch0_up = (YK.yaogan.ch0 > 600);
        const bool trigger = (UD_BP_BURST.updata(ch0_up) == UpDown_check_rising);
        if (trigger && dbg_bp_burst_left == 0 && mcl->Mode == 1 && Error_flag == 0)
        {
            dbg_bp_burst_left  = BP_BURST_TEST_SHOTS;
            dbg_bp_burst_done  = 0;
            dbg_bp_burst_start = (float)M2006_BP.mang_inf;
            burst_last_tick    = bp_fire_tick_1ms - BP_BURST_TEST_INTERVAL_MS;   // 第一格立即推
        }
        // 关摩擦轮或报错: 立即中止
        if (dbg_bp_burst_left > 0 && (mcl->Mode != 1 || Error_flag != 0))
            dbg_bp_burst_left = 0;

        if (dbg_bp_burst_left > 0 &&
            (uint32_t)(bp_fire_tick_1ms - burst_last_tick) >= BP_BURST_TEST_INTERVAL_MS &&
            !bp->Blockage && !bp->Blockage_To_Daed)
        {
            // 和 AutoFire_Ready 同样的"拨盘跟上"门控: 上一格没走完不推下一格
            const double lag = M2006_BP.mang_inf - bp->One_Target_Angle;
            if (lag <= (double)BOPAN_ANGLE)
            {
                AutoFire_Shoot();                    // 和自瞄完全同一条出弹路径(含 bp_shot_count++)
                burst_last_tick = bp_fire_tick_1ms;
                dbg_bp_burst_left--;
                dbg_bp_burst_done++;
            }
        }
        // 测试进行中, 或 ch0 推在上面: 不走原遥控连发, 拨盘保持单发位置环
        burst_owns = (dbg_bp_burst_left > 0) || ch0_up;
        if (burst_owns) Continuous_shooting_flag = 0;
    }
    else
    {
        (void)UD_BP_BURST.updata(false);
        dbg_bp_burst_left = 0;
    }
#endif
    if (burst_owns)
    {
        // 定量连发测试接管拨盘, 其余火控本周期不处理
    }
    else if (YK_Mode == SHOOT_MODE)
    {
        // 视觉控制和拨盘开火是两条独立链路：云台由 app_gimbal.cpp 按 mode 1/2 持续跟随；
        // 这里只决定拨盘是否出弹，不能影响 yaw/pitch 跟随。ch1>600 由 MCL_while_layer() 开摩擦轮。
        //
        // ★2026-10-04 开自瞄后, 发不发弹只由视觉决定, 遥控单发/遥控连发全部屏蔽:
        //   ch0 < -600: 视觉单发火控 —— 视觉每来一次开火命令(mode==2 上升沿), 电控推一格
        //   ch0 >  600: 视觉连续火控 —— 视觉 mode==2 期间, 按 AUTOFIRE_INTERVAL_TICKS(40ms=25Hz) 连续推
        //   ch0 居中   : 不开火
        // 没开自瞄时才走遥控单发(ch0<-600 上升沿)/遥控连发(ch0>600)。
        // 原来开自瞄 + ch0>600 时, 遥控连发(CON_ON, 每周期推 Shot_SP_1*15, 不看视觉、不计数)
        // 会和视觉火控叠在一起推拨盘: 实测视觉下令 35 格, 摩擦轮测到 73 颗。
        const u8 zimiao_on = request.zimiao_status ? 1 : 0;
        const u8 vision_fire_signal = zimiao_on && SuperPower.mode == 2;
        // 安全门控（摩擦轮/错误/堵弹）
        const u8 fire_safe = mcl->Mode == 1 && Error_flag == 0
                             && !bp->Blockage && !bp->Blockage_To_Daed;
        // 边沿检测每周期都要更新, 否则切换自瞄开关/拨杆的第一帧会误判出一个上升沿
        const u8 remote_one = (UD_BP_ON.updata(bp->ONE_ON) == UpDown_check_rising);
        const u8 vision_fire_rising =
            (UD_BP_FIRE.updata(vision_fire_signal) == UpDown_check_rising);
        static u8 vision_one_pending = 0;   // 单发火控: 上升沿到了但拨盘还没准备好, 先记着

        if (zimiao_on)
        {
            const u8 vision_one_mode = YK.yaogan.ch0 < -600;
            const u8 vision_con_mode = YK.yaogan.ch0 > 600;
            if (vision_one_mode && vision_fire_rising) vision_one_pending = 1;
            // 视觉这次命令已经结束 / 拨杆不在单发档 / 不安全: 作废, 不补打
            if (!vision_one_mode || !vision_fire_signal || !fire_safe) vision_one_pending = 0;

            const u8 want_shot = vision_fire_signal && fire_safe &&
                                 (vision_con_mode || (vision_one_mode && vision_one_pending));
            if (want_shot && AutoFire_Ready())
            {
                AutoFire_Shoot();
                vision_one_pending = 0;
            }
            // 开自瞄时拨盘只用单发位置环, 视觉没让开火就停在当前目标, 不推。
            // 刚从遥控连发切过来时, 连发目标可能还超前一截, 先拉回当前角, 避免多推。
            if (Continuous_shooting_flag)
            {
                bp->One_Target_Angle = M2006_BP.mang_inf;
                bp->Continuous_Target_Angle = M2006_BP.mang_inf;
            }
            Continuous_shooting_flag = 0;
        }
        else if (remote_one && mcl->Mode == 1 && Error_flag == 0)
        {
            bp->One_Target_Angle = M2006_BP.mang_inf;
            if (abs((int32_t)M2006_BP.mang_inf % (int32_t)BOPAN_ANGLE) < 10000)
                bp->Blockage_Compensation = 0;
            else
                bp->Blockage_Compensation = 1;
            if (bp->Blockage || bp->Blockage_To_Daed)
            {
            }
            else
            {
                bp->One_Target_Angle -= BOPAN_ANGLE;
                bp->energy.C_Shoot++;
                bp_shot_count++;
                bp->Continuous_shooting_flag = 0;
                bp->Continuous_Target_Angle = bp->One_Target_Angle;
                bp->Blockage = 0;
                bp->Blockage_To_Daed = 0;
            }
        }
        else if ((bp->CON_ON && mcl->Mode == 1) && Error_flag == 0 && buff_mode != 1)
        {
            bp->Continuous_shooting_flag = 1;
            bp->Continuous_Target_Angle -= Shot_SP_1 * 15;
            bp->energy.real_inf = M2006_BP.mang_inf;
            bp->energy.total_real += bp->energy.real_inf - bp->energy.last_inf;
            bp->energy.C_Shoot = bp->energy.total_real / BOPAN_ANGLE;
            bp->energy.last_inf = M2006_BP.mang_inf;
            bp->One_Target_Angle = bp->Continuous_Target_Angle;
        }
        else
        {
            if (bp->Continuous_shooting_flag)
                bp->One_Target_Angle = M2006_BP.mang_inf;
            bp->Continuous_shooting_flag = 0;
            bp->Continuous_Target_Angle = M2006_BP.mang_inf;
        }
    }
    else if (YK_Mode == PLAYER_MODE)
    {
        remain_heat = shooter_id1_17mm_cooling_limit - shooter_id1_17mm_cooling_heat;
        const u8 vision_fire_p = request.zimiao_status && SuperPower.mode == 2 && mcl->Mode && Error_flag == 0
                                 && !bp->Blockage && !bp->Blockage_To_Daed;
        const u8 mouse_one = (UD_BP_DF.updata(YK.shubiao.press_l) == UpDown_check_rising);
        if (vision_fire_p && AutoFire_Ready())
        {
            AutoFire_Shoot();
            Continuous_shooting_flag = 0;
            bp->delay_2ms = 0;
        }
        else if (vision_fire_p)
        {
        }
        else if (mouse_one && (remain_heat > 30 || YK.Pressed_Check(KEY_PRESSED_CTRL)))
        {
            bp->One_Target_Angle = M2006_BP.mang_inf;
            bp->One_Target_Angle -= BOPAN_ANGLE;
            bp->Continuous_Target_Angle = bp->One_Target_Angle;
            bp->Continuous_shooting_flag = 0;
            bp->delay_2ms = 0;
        }
        else if ((((YK.shubiao.press_l) && bp->delay_2ms > 150 && !request.zimiao_status) || (request.zimiao_status && YK.shubiao.press_l && SuperPower.mode == 2 && mcl->Mode && bp->delay_2ms > 150))&& buff_mode == 0)
        {
            bp->Continuous_shooting_flag = 1;
#if BP_TEST_FLAG
            bp->Continuous_Target_Angle += Shot_SP_1 * 7;
#else
            if (remain_heat < 20)
            {
                bp->Continuous_Target_Angle = bp->One_Target_Angle = M2006_BP.mang_inf;
                bp->Prefabricate_Flag = 0;
            }
            else if (remain_heat >= 20 && remain_heat < 60)
            {
                bp->Continuous_Target_Angle -= Shot_SP_1 * 5;
                bp->Prefabricate_Flag = 1;
            }
            else if (remain_heat >= 60 && remain_heat < 120)
            {
                bp->Continuous_Target_Angle -= Shot_SP_1 * 10;
                bp->Prefabricate_Flag = 1;
            }
            else if (remain_heat >= 120 && remain_heat < 150)
            {
                bp->Continuous_Target_Angle -= Shot_SP_1 * 15;
                bp->Prefabricate_Flag = 1;
            }
            else if (remain_heat >= 150 && remain_heat < 190)
            {
                bp->Continuous_Target_Angle -= Shot_SP_1 * 18;
                bp->Prefabricate_Flag = 1;
            }
            else if (remain_heat >= 190 && remain_heat < 220)
            {
                bp->Continuous_Target_Angle -= Shot_SP_1 * 20;
                bp->Prefabricate_Flag = 1;
            }
            else if (remain_heat >= 220)
            {
                bp->Continuous_Target_Angle -= Shot_SP_1 * 20;
                bp->Prefabricate_Flag = 1;
            }
#endif
        }
        else
        {
            if (bp->Continuous_shooting_flag)
                bp->One_Target_Angle = M2006_BP.mang_inf;
            bp->Continuous_shooting_flag = 0;
            bp->Continuous_Target_Angle = M2006_BP.mang_inf;
        }
    }
    (void)jianshu_flag;

    // 状态机：根据 Mode 决定 State
    FsmState nextState = FsmState::ZERO;

    switch (currentMode)
    {
        case FsmMode::GYRO:
            nextState = Continuous_shooting_flag ? FsmState::CONTINUOUS : FsmState::ONE_SHOT;
            break;
        case FsmMode::AUTO:
            nextState = Continuous_shooting_flag ? FsmState::CONTINUOUS : FsmState::ONE_SHOT;
            break;
        case FsmMode::PROTECT:
        default:
            nextState = FsmState::ZERO;
            break;
    }

    if (currentState != nextState)
    {
        switchState(nextState);
    }
    const f out = runCurrentState(this);

    // debug mirrors for Ozone (see top of file)
    const bool use_con = (currentState == FsmState::CONTINUOUS);
    const double real = (double)M2006_BP.mang_inf;
    const double tgt = use_con ? Continuous_Target_Angle : One_Target_Angle;
    dbg_bp_target_one = (float)One_Target_Angle;
    dbg_bp_target_con = (float)Continuous_Target_Angle;
    dbg_bp_real       = (float)real;
    dbg_bp_err        = (float)(tgt - real);
    dbg_bp_speed      = (float)M2006_BP.sp;
    dbg_bp_speed_ref  = use_con ? M2006_BP_Con_Angle.OUT_PID : M2006_BP_Angle.OUT_PID;
    dbg_bp_out        = out;
    dbg_bp_state      = (float)static_cast<u8>(currentState);
    {
        int32_t ph = (int32_t)M2006_BP.mang_inf % (int32_t)BOPAN_ANGLE;
        if (ph < 0) ph += (int32_t)BOPAN_ANGLE;
        dbg_bp_phase = (float)ph;
    }
    dbg_bp_shot       = (float)bp_shot_count;
    dbg_bp_fire_cmd   = (request.zimiao_status && SuperPower.mode == 2) ? 1.0f : 0.0f;
#if BP_BURST_TEST
    dbg_bp_burst_turns = (float)(((double)dbg_bp_burst_start - real) / (double)BOPAN_ANGLE);
#endif
    return out;
}

void BP::Start_Recovery(double current_angle, double recovery_angle, u8 use_continuous_pid)
{
    this->Recovery_State = RECOVERY_ACTIVE;
    this->Recovery_Use_ContinuousPid = use_continuous_pid;
    this->Recovery_Target_Angle = current_angle + recovery_angle;
    this->Recovery_Last_Error = this->Recovery_Target_Angle - current_angle;
    this->One_Target_Angle = current_angle;
    this->Continuous_Target_Angle = current_angle;
    this->Continuous_shooting_flag = 0;
    this->Error_flag = 1;
}

void BP::Stop_Recovery(double current_angle)
{
    this->Recovery_State = RECOVERY_IDLE;
    this->Recovery_Use_ContinuousPid = 0;
    this->Recovery_Target_Angle = current_angle;
    this->Recovery_Last_Error = 0;
    this->One_Target_Angle = current_angle;
    this->Continuous_Target_Angle = current_angle;
    this->Continuous_shooting_flag = 0;
    this->BP_Error = 0;
    this->Error_flag = 0;
}

void BP::BP_while_layer()
{
    if (bp->Mode == GYRO_MODE || bp->Mode == AUTO_MODE)
    {
        if (YK.yaogan.ch0 < N_YG)
        {
            bp->State = ONE_ON_MASK;
        }
        else if (YK.yaogan.ch0 > P_YG)
        {
            bp->State = CON_ON_MASK;
        }
        else
        {
            bp->State = OFF_MASK;
        }
    }
    else
    {
        bp->State = OFF_MASK;
    }
    bp->ONE_ON = (bp->State & ONE_ON_MASK) ? 1 : 0;
    bp->CON_ON = (bp->State & CON_ON_MASK) ? 1 : 0;
}

void BP::BP_Energy()
{
    bp->test_num++;
    bp->energy.Energy_Shoot = bp->energy.C_Shoot * 10.0f - bp->energy.Energy_re / 10.0f;
    bp->energy.Energy_real += bp->energy.Energy_Shoot;
    if (bp->energy.Energy_real < 0)
    {
        bp->energy.Energy_real = 0;
        bp->energy.mode = GYRO_MODE;
    }
    else if (bp->energy.Energy_real > bp->energy.Energy_Clipping)
    {
        bp->energy.mode = PROTECT_MODE;
        bp->energy.Energy_real = bp->energy.Energy_Clipping;
    }
    else
        bp->energy.mode = GYRO_MODE;
    bp->energy.C_Shoot = 0;
}

void BP::BP_time_out(void)
{
    // TIM7 1kHz 计数, 不用 HAL_GetTick. 最小取 1: 原逻辑用 Last_Blockage_Recovery_Tick==0
    // 表示“从未恢复过”, 防止上电第 0 个 tick 触发恢复时被误判.
    const uint32_t now_tick = (bp_fire_tick_1ms == 0) ? 1u : bp_fire_tick_1ms;
    const u8 recovery_ready = (bp->Last_Blockage_Recovery_Tick == 0) ||
                              ((now_tick - bp->Last_Blockage_Recovery_Tick) >= BP_RECOVERY_MIN_INTERVAL_MS);
    const u8 use_continuous_pid = bp->Continuous_shooting_flag;
    const u8 one_pid_overload = (M2006_BP_Speed.OUT_PID > 12000 || M2006_BP_Speed.OUT_PID < -12000);
    const u8 con_pid_overload = (M2006_BP_Con_Speed.OUT_PID > 12000 || M2006_BP_Con_Speed.OUT_PID < -12000);

    if (!use_continuous_pid && one_pid_overload)
    {
        if (++bp->BP_Error > 200)
        {
            if (recovery_ready)
            {
                Error_flag = 1;
                bp->One_Target_Angle = bp->Continuous_Target_Angle = M2006_BP.mang_inf;
                BP_ERROR_ONE;
                bp->Last_Blockage_Recovery_Tick = now_tick;
                Error_flag = 0;
            }
            bp->BP_Error = 0;
        }
    }
    else if (use_continuous_pid && con_pid_overload)
    {
        if (++bp->BP_Error > 200)
        {
            if (recovery_ready)
            {
                Error_flag = 1;
                bp->Continuous_shooting_flag = 1;
                bp->Continuous_Target_Angle = bp->One_Target_Angle = M2006_BP.mang_inf;
                BP_ERROR_CON;
                bp->Last_Blockage_Recovery_Tick = now_tick;
                Error_flag = 0;
            }
            bp->BP_Error = 0;
        }
    }
    else
        bp->BP_Error = 0;

    bp->delay_2ms++;
    bp_fire_tick_1ms++;   // TIM7 1kHz 计时基准, 供 AutoFire 射频限制
}

f BP::BP_deal(u8 YK_Mode, u8 jianshu_flag)
{
    return BP_Out_Interface(YK_Mode, jianshu_flag);
}
