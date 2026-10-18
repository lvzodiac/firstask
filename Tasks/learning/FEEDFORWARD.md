# 速度环的摩擦前馈

当前前馈参考用户提供的代码，已替换原来的加速度前馈，不再使用 UP/DOWN 系数。
PID 参数、控制模式、正弦目标曲线、CAN 和看门狗逻辑保持原样。

## 计算过程

```cpp
const float sign = std::tanh(ref_vel_rpm / sign_scale);
float feedforward = FRICTION_FF_COULOMB * sign
                  + FRICTION_FF_VISCOUS * ref_vel_rpm;
```

- `ref_vel_rpm`：速度环目标速度，不是实际速度。
- `sign_scale`：零速附近的平滑范围，单位 rpm。
- `tanh(x)`：输出在 -1 到 1 之间的平滑函数，x=0 时输出0。
- 第一项补偿基本摩擦，第二项补偿随速度增加的摩擦。

`SpeedLoopOutput()` 先计算速度 PID，再叠加摩擦前馈，限制总输出，返回电流码值。
速度模式使用正弦目标速度；位置模式使用位置 PID 输出的目标速度。
两种模式共用这个函数，反馈超时时会在调用前返回并尝试发送零电流。

## 参数

config.h 中的初值沿用参考代码，尚未在本机电机上验证：

```cpp
#define FRICTION_FF_ENABLE               1
#define FRICTION_FF_COULOMB              400.0f
#define FRICTION_FF_VISCOUS              1.0f
#define FRICTION_FF_SIGN_SCALE_RPM       5.0f
```

`ENABLE=0` 关闭前馈；两项补偿系数均取非负数，方向由目标速度决定。
平滑范围须大于0；代码对非正数使用1 rpm兜底，避免除零。
平滑范围越小，过零切换越陡；越大，零速附近的补偿越缓。

| 目标速度 | 前馈电流码值，约 |
| --- | --- |
| -100 rpm | -500 |
| -5 rpm | -310 |
| 0 rpm | 0 |
| +5 rpm | +310 |
| +100 rpm | +500 |

这与加速度前馈不同：目标速度到达峰值时仍有摩擦补偿，目标速度为0时补偿为0。
它没有完整描述静摩擦，不能保证消除过零停顿，需要实测比较。
保持 PID 和目标曲线不变，每次只调一项，观察多个完整周期。

## Ozone 观察

- `g_refVelRpm`：速度环目标速度，rpm。
- `g_motor.raw_vel_rpm_`：实际速度，rpm。
- `g_speedFeedforward`：当前摩擦前馈，电流码值；关闭前馈或反馈超时时为0。
- `g_motor.voltage_`：叠加并限幅后保存的电流指令，名字是旧命名，不是实际电流反馈。
- `g_refPosDeg` / `g_fdbPosDeg`：位置模式的目标和实际角度，单位度，基准保持原样。

速度与电流使用不同纵轴或分开显示，时间对齐，重点检查实际速度停在零的区间。
重新编译、下载后，让 Ozone 加载与板上固件匹配的 build/project.elf。
