# GM6020 正弦速度与位置跟随

基于 STM32F407 的 GM6020 电机控制工程，通过 CAN 发送电流指令，实现正弦速度跟随和正弦位置跟随。
应用控制代码位于 `Tasks/`，包含 CAN 数据编解码、PID、摩擦前馈及反馈超时保护，并启用独立看门狗。

## 控制方式

- 速度模式：正弦目标转速 -> 速度 PID + 摩擦前馈 -> 电流指令。
- 位置模式：正弦目标角度 -> 位置 PID -> 目标转速 -> 速度 PID + 摩擦前馈 -> 电流指令。
- 摩擦前馈：`C * tanh(v_ref / scale) + B * v_ref`，根据速度环目标速度平滑改变方向。

## 配置

参数集中在 `Tasks/config.h`。当前提交配置为：

| 项目 | 配置 |
| --- | --- |
| 控制模式 | `CONTROL_MODE = 0`，速度模式；设为 1 使用位置模式 |
| 电机 ID | 1 |
| 控制频率 | 1000 Hz，由 TIM6 中断调用 |
| 速度轨迹 | 幅值 100 rpm，频率 0.25 Hz |
| 位置轨迹 | 围绕首次有效位置摆动，幅值 180 度，频率 0.25 Hz |
| 速度 PID | Kp=15，Ki=0.2，Kd=0，积分限幅 550 |
| 位置 PID | Kp=8，Ki=0，Kd=0，输出限幅 +/-300 rpm |
| 摩擦前馈 | 开启，C=400，B=1，scale=5 rpm |
| CAN 反馈超时 | 100 ms |
| 看门狗 | 开启喂狗 |

PID 使用离散计算，积分和微分系数已包含采样周期的影响，不能直接与另一种 PID 实现的参数互换。

## 硬件与 CAN

- MCU：STM32F407IG，CAN1，波特率 1 Mbps。
- 电机：GM6020，独立供电，CAN_H/CAN_L 按硬件手册接线并配置终端电阻。
- ID 1 反馈报文为 `0x205`，电流指令组为 `0x1FE`，占用前两个字节。
- 电流码值范围为 -16384 到 +16384。源码中的旧成员名 `voltage_` 保存的是电流指令。

## 编译

安装 CMake 3.22 或更新版本、Ninja 和 Arm GNU Toolchain，将 `arm-none-eabi-gcc` 等工具加入 PATH。

```sh
cmake --preset Debug
cmake --build --preset Debug --parallel 4
```

生成 `build/project.elf`、`build/Debug/project.hex` 和 `build/Debug/project.bin`。
通过支持 SWD 的调试器下载到对应开发板。Ozone 应加载与板上程序一致的 ELF。
本机 Ozone 设置、编译产物、备份源码和参考 PDF 不包含在仓库中。

## 调试与学习

- `g_refVelRpm`：速度环目标转速，rpm。
- `g_speedFeedforward`：摩擦前馈，电流码值。
- `g_refPosDeg` / `g_fdbPosDeg`：位置模式目标和实际累计角度，单位度。
- [CAN、PID 与代码讲义](Tasks/learning/README.md)
- [摩擦前馈说明](Tasks/learning/FEEDFORWARD.md)
- [早期逐行讲解](Tasks/learning/LINE_BY_LINE.md)：后续代码有新增，行号以当前源码为准。

## 验证状态与保护边界

当前工程通过 Debug 编译；项目已有实机调试记录，当前摩擦前馈参数仍需在实际负载上验证。
反馈超时会清空 PID 并尝试发送零电流，CAN 断线时不能保证指令送达；零电流也不意味着立即停转。
看门狗在控制中断内刷新，不能检测主循环卡死但中断仍正常执行的情况。
`WATCHDOG_ENABLE = 0` 会停止喂狗，不会关闭已经启动的看门狗。

`Drivers/` 中的 ST 和 Arm 代码保留原有许可证，分别见对应目录内的 `LICENSE.txt`。
