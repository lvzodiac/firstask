#include "ControlTask.h"

#include "config.h"
#include "Gm6020.h"
#include "Pid.h"

#include "main.h"
#include "can.h"
#include "tim.h"
#include "iwdg.h"

#include <cmath>

// Ozone 连续采样用的目标转速，单位 rpm。
volatile float g_refVelRpm = 0.0f;
// 速度环的摩擦前馈，单位为电流码值；两种控制模式共用。
volatile float g_speedFeedforward = 0.0f;
// Ozone position traces, in degrees; updated in position mode.
volatile float g_refPosDeg = 0.0f;
volatile float g_fdbPosDeg = 0.0f;

namespace {

constexpr float kPi      = 3.14159265358979323846f;
constexpr float kTwoPi   = 2.0f * kPi;
constexpr float kRad2Deg = 180.0f / kPi;

// 控制对象
Gm6020 g_motor(MOTOR_ID);

// PID 控制器
Pid g_speedPid(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
               SPEED_PID_I_MAX, SPEED_PID_OUT_MAX, SPEED_PID_OUT_MIN);
Pid g_posPid(POS_PID_KP, POS_PID_KI, POS_PID_KD,
             POS_PID_I_MAX, POS_PID_OUT_MAX, POS_PID_OUT_MIN);

// 1kHz 节拍计数。CAN 中断只写 g_lastFeedbackTick，两个变量均为 32 位。
uint32_t g_tick = 0;
volatile uint32_t g_lastFeedbackTick = 0;
volatile bool g_feedbackReceived = false;

// 位置模式：上电位置 home（用于围绕 home 做正弦摆动）
float    g_posHomeDeg = 0.0f;
bool     g_homeLatched = false;

// 喂狗
void FeedWatchdog(void) {
#if WATCHDOG_ENABLE
    HAL_IWDG_Refresh(&hiwdg);
#endif
}

// 发送单电机电流控制帧。GM6020 的电流给定范围是 -16384~16384。
void SendCurrentCmd(void) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t  tx_data[8] = {0};
    uint32_t mailbox;

    g_motor.encode(tx_data);

    tx_header.StdId = g_motor.txId();
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    (void)HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &mailbox);
}

bool FeedbackIsFresh(void) {
    if (!g_feedbackReceived) return false;
    return (g_tick - g_lastFeedbackTick) <= CAN_FEEDBACK_TIMEOUT_TICKS;
}

// 根据目标转速补偿摩擦，tanh 让补偿在零速附近平滑改变方向。
float FrictionFeedforward(float ref_vel_rpm) {
#if FRICTION_FF_ENABLE
    const float sign_scale = (FRICTION_FF_SIGN_SCALE_RPM > 0.0f)
                           ? FRICTION_FF_SIGN_SCALE_RPM : 1.0f;
    const float sign = std::tanh(ref_vel_rpm / sign_scale);
    return FRICTION_FF_COULOMB * sign
         + FRICTION_FF_VISCOUS * ref_vel_rpm;
#else
    (void)ref_vel_rpm;
    return 0.0f;
#endif
}

// 两种模式共用速度环：PID 加摩擦前馈，再限制总电流。
float SpeedLoopOutput(float ref_vel_rpm, float act_vel_rpm) {
    float out = g_speedPid.calc(ref_vel_rpm, act_vel_rpm);
    const float feedforward = FrictionFeedforward(ref_vel_rpm);
    g_speedFeedforward = feedforward;
    out += feedforward;
    if (out > SPEED_PID_OUT_MAX) out = SPEED_PID_OUT_MAX;
    if (out < SPEED_PID_OUT_MIN) out = SPEED_PID_OUT_MIN;
    if (out > 16384.0f) out = 16384.0f;
    if (out < -16384.0f) out = -16384.0f;
    return out;
}

} // namespace

extern "C" void ControlTaskInit(void) {
    // 1. 配置 CAN 过滤器：全零掩码放行消息到 FIFO0，再由回调匹配电机 ID。
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank           = 0;
    filter.SlaveStartFilterBank  = 14;  // CAN1 使用 0~13，CAN2 使用 14~27
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &filter);

    // 2. 启动 CAN 并开启接收中断
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    g_tick = 0;
    g_lastFeedbackTick = 0;
    g_feedbackReceived = false;
    g_refVelRpm = 0.0f;
    g_speedFeedforward = 0.0f;
    g_refPosDeg = 0.0f;
    g_fdbPosDeg = 0.0f;
    g_speedPid.reset();
    g_posPid.reset();

    // 3. 先喂一次狗，再启动 1kHz 控制定时器；此喂狗方式不能检测主循环卡死。
    FeedWatchdog();
    HAL_TIM_Base_Start_IT(&htim6);
}

extern "C" void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
    if (std_id == g_motor.rxId()) {
        g_motor.decode(data);
        g_lastFeedbackTick = g_tick;
        g_feedbackReceived = true;
    }
}

extern "C" void MainTask(void) {
    FeedWatchdog();

    g_tick++;
    float t = static_cast<float>(g_tick) * CONTROL_PERIOD_S;

    // 反馈超时时清空 PID 并尝试发送零电流；CAN 断线时不保证指令能送达。
    if (!FeedbackIsFresh()) {
        g_refVelRpm = 0.0f;
        g_speedFeedforward = 0.0f;
        g_speedPid.reset();
        g_posPid.reset();
        g_motor.setCurrent(0);
        SendCurrentCmd();
        return;
    }

    if (CONTROL_MODE == 0) {
        // ---- 速度正弦跟随：v(t) = A * sin(2*pi*f*t) ----
        float phase = kTwoPi * SPEED_SINE_FREQ_HZ * t;
        float ref_vel_rpm = SPEED_SINE_AMP_RPM * std::sinf(phase);
        g_refVelRpm = ref_vel_rpm;

        float out = SpeedLoopOutput(ref_vel_rpm, g_motor.velRpm());
        g_motor.setCurrent(static_cast<int16_t>(out));
    } else {
        // ---- 位置正弦跟随（串级：位置环 -> 速度环）----
        // 上电时锁定当前位置作为 home，此后围绕 home 做正弦摆动
        if (!g_homeLatched) {
            g_posHomeDeg  = g_motor.angle() * kRad2Deg;
            g_homeLatched = true;
        }

        // p(t) = home + A * sin(2*pi*f*t)
        float ref_pos_deg = g_posHomeDeg + POS_SINE_AMP_DEG * std::sinf(kTwoPi * POS_SINE_FREQ_HZ * t);
        float fdb_pos_deg = g_motor.angle() * kRad2Deg;
        g_refPosDeg = ref_pos_deg;
        g_fdbPosDeg = fdb_pos_deg;

        // 外环：位置环输出期望速度（rpm）
        float ref_vel_rpm = g_posPid.calc(ref_pos_deg, fdb_pos_deg);
        g_refVelRpm = ref_vel_rpm;

        // 内环：速度 PID 加摩擦前馈，输出限幅后的电流码值
        float out = SpeedLoopOutput(ref_vel_rpm, g_motor.velRpm());
        g_motor.setCurrent(static_cast<int16_t>(out));
    }

    SendCurrentCmd();
}

// ---------------------------------------------------------------------------
// HAL 弱回调实现（覆盖 HAL 库中的弱定义）
// ---------------------------------------------------------------------------
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        MainTask();
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hcan != &hcan1 || HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }
    CanFeedbackCallback(rx_header.StdId, rx_data);
}
