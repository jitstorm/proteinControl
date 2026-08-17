# 当前三轴步进电机逻辑说明

本文按当前源码整理三轴步进控制的实际行为，描述的是程序逻辑，不代表已完成接线、方向、行程或实机运动验证。

## 1. 总览

当前三轴为独立的 DMA 脉冲轴：

| 逻辑轴 / 旧名称 | STEP 输出 | 定时器 | DMA | 方向输出 | 旧串口命令 |
| --- | --- | --- | --- | --- | --- |
| X / PU1 | PB11 | TIM5 | DMA2 Channel 2 | `HC595Data[1]` bit4（DIR1） | `0x1B` |
| Y / PU2 | PB10 | TIM6 | DMA2 Channel 3 | `HC595Data[2]` bit5（DIR2/Q5） | `0x1C` |
| Z / PU3 | PB13 | TIM7 | DMA2 Channel 4 | `HC595Data[1]` bit6（DIR3/Q6） | `0x1E` |

上电初始化顺序中会初始化三条底层轴，再初始化 `RobotArm` 管理层；初始化本身不会输出 STEP。主循环持续调用 `RobotArm_Task()` 和 `RobotArmProtocol_Task()`，因此 RobotArm 动作是非阻塞状态机，而不是在串口处理函数中等待跑完。

## 2. 脉冲如何产生

三轴的共同设计是：定时器更新事件触发 DMA，DMA 将预先准备的值写入 `GPIOB->BSRR`。

- DMA 缓冲交替写“置位 STEP”和“复位 STEP”。
- 一个 STEP 上升沿计为一步，因此一整步由两个 DMA 边沿组成：高、低。
- DMA 以分段方式输出，段完成中断只负责统计已完成步数、计算下一段速度并装载下一段；不会逐步软件翻转 GPIO。
- 任意轴停止时均关闭对应定时器和 DMA，并将 STEP 引脚拉低。

X、Y 的分段长度为 128 步（256 个边沿）；Z 的分段长度也是 128 步（256 个边沿）。X、Y 的底层驱动使用分段梯形速度规划。Z 在第一段以起始速度运行，中间段直接切换到目标速度，最后一段回到起始速度，因此它是分段加速/匀速/减速，而非逐段按加速度公式细化的完整梯形曲线。

## 3. 单轴启动与停止

RobotArm 驱动适配层统一以 `RobotArmDriver_Start(axis, direction, steps, speed)` 启动：

1. 拒绝非法轴、零步数和该轴忙碌的请求；速度为 0 时最低按 1 steps/s 处理。
2. X 先更新 DIR1，再调用 PB11 的 DMA 梯形运动接口。
3. Y 由 `Stepper2_Start()` 处理：拒绝零步/忙碌，速度限制在 1～50000 steps/s；先停止旧输出、写 DIR2、延时 2 us，再以起始 500 steps/s、目标请求速度、加速度 100000 启动。
4. Z 由 `PU3_Stepper_Start()` 处理：拒绝零步/忙碌；先确保已停止、写 DIR3，随后以起始 500 steps/s、目标请求速度、加速度参数 100000 启动。
5. `RobotArmDriver_Stop()` 可分别立即停止一条轴；`RobotArm_Stop()` 则停止全部三轴，并使被中断轴的坐标失效。

方向位都通过 `HC595Data` 阴影数组的按位修改后调用 `ShiftRegister_WriteAll()` 输出，避免覆盖同一片 74HC595 的其他控制位。逻辑正负方向与真实机构正反方向尚需实机确认。

## 4. 两条控制入口

### 4.1 旧 V1 串口直驱（调试/兼容路径）

帧格式中的步数为大端 16 位 `D0:D1`，方向取 `D2`，最大速度为 `D3 * 200 steps/s`；三条命令均会先原样回显请求数据。

| 命令 | 作用 | 当前特殊行为 |
| --- | --- | --- |
| `0x1B` | 直接启动 X / PU1 | 写 DIR1 后启动 PB11；使 X 的 RobotArm 坐标失效。 |
| `0x1C` | 直接启动 Y / PU2 | 先关闭 `0x20` 的传感器停止配置；若 Y 正在跑先停止，再写 DIR2、等待 2 us、启动 PB10；使 Y 坐标失效。 |
| `0x1E` | 直接启动 Z / PU3 | 调用 `PU3_Stepper_Start()`；使 Z 坐标失效。 |
| `0x1F` | 查询 Y 状态 | 回应 `D0=0` 表示运行中，`D0=1` 表示已停止或完成。 |
| `0x20` | Y / PU2 传感器停止运动 | 仅传感器参数合法时启动；启动步骤与 `0x1C` 相同，并使 Y 坐标失效。 |

旧直驱不会向 RobotArm 管理层报告精确完成步数。因此，只要使用过某轴的旧命令，该轴的 `position_valid` 会被清零；之后不能把旧坐标用于 RobotArm 的绝对运动，直到重新建立可信坐标。

### 4.2 RobotArm V2 管理层（XYZ 逻辑坐标路径）

V2 命令由 `RobotArmProtocol_HandleFrame()` 接收，接受动作后先回 ACK；真实完成、停止或失败由随后携带原请求 SEQ 的 EVENT 表示，ACK 不是电机已完成的证据。

可用动作逻辑包括：单轴相对/绝对运动、按 X → Y → Z 的 `MoveTo`、按“必要时先抬 Z → X → Y → 最终 Z”的 `MoveToSafe`、单轴 Home、按 Z → Y → X 的 HomeAll，以及 STOP。

- 相对/绝对单轴移动均由 `RobotArm_Task()` 轮询底层轴忙碌状态；只有底层脉冲完成后才提交该轴的新逻辑坐标。
- `MoveTo` 串行执行 X、Y、Z，不做三轴同时插补。
- `MoveToSafe` 只有在安全 Z 配置有效时才允许。
- `STOP` 停止三轴、结束组合状态机；中断的轴坐标变为不可信。STOP 的 ACK 也不等于原运动完成，原运动应等待其 `STOPPED EVENT`。

## 5. 当前默认安全配置

当前 `robot_arm_config.h` 的默认值决定了以下现状：

- `ROBOT_ARM_DEBUG_ASSUME_HOME=1`：RobotArm 初始化时将 XYZ 标记为已 Home 且坐标有效，初始逻辑坐标均为 0。这是临时调试假设，不是传感器确认的回零。
- X/Y/Z 软件行程限位均为关闭状态，当前没有真实最大行程值。
- 正式 Home 的三轴配置默认均关闭；只有显式定义 `ROBOT_ARM_LOGIC_TEST` 时，才启用 100 步、1 秒的短行程逻辑测试参数。
- SafeMove 默认关闭，安全 Z 默认值 0 只是占位值，不能作为生产坐标。
- 三轴 RobotArm 默认速度均为 1000 steps/s。

因此，当前源码虽具备三轴 DMA 驱动和 V2 状态机，但真实方向、Home 传感器绑定、限位、实际行程和 Safe Z 均尚未由默认生产配置启用或标定。没有这些实机证据时，不应把 V2 的初始零点或任何逻辑坐标当作真实机械位置。

## 6. 关键源码位置

- 启动初始化与主循环调度：`User/main.c`。
- 三条 DMA 步进轴、方向输出、DMA 中断：`User/step_dma.c`、`User/step_dma.h`。
- XYZ 到底层轴的适配、统一启动/停止/完成步数查询：`User/robot_arm_driver.c`。
- 坐标、Home、单轴/组合动作和 STOP 状态机：`User/robot_arm.c`。
- V2 的 ACK、EVENT、STATUS 生命周期：`User/robot_arm_protocol.c`。
- 旧 V1 `0x1B/0x1C/0x1E/0x1F/0x20` 命令：`User/protocol.c`。
