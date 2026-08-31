# RobotArm MCU 改造与交接总结

## 1. 文档范围

本文总结 STM32F103 工程中 RobotArm Phase 1～10.6 的当前实现，用于维护、交接和后续实机冻结。内容只描述已经存在的代码行为；尚未实机确认的映射和参数会明确标记，不把计划项写成已完成功能。

## 2. 改造前后的架构

### 改造前

```text
V1 串口命令
  └─ handle_command()
      ├─ 直接改 74HC595 DIR
      ├─ 直接启动 PB11/PB10/PU3 DMA
      └─ 各入口自行处理停止或传感器
```

主要特征：

- 步进命令直接操作底层轴驱动，没有统一 XYZ 逻辑坐标。
- 不同入口分别控制方向、步数和传感器停止，调用者难以判断坐标是否仍可信。
- 没有统一的 Home、Hard Limit、Soft Limit、MoveTo、SafeMove、Stop 终态和异步事件生命周期。
- DMA 完成与“机械动作可信完成”之间没有统一管理层判据。

### 改造后

```text
USART1 RS485 字节流
  └─ V1/V2 流解析与独立接收队列
      ├─ V1 → 既有 parse_frame / handle_command 兼容路径
      └─ V2 → RobotArmProtocol
                 ├─ ACK / STATUS / EVENT 可靠发送队列
                 └─ RobotArm 管理层
                      ├─ XYZ 坐标与状态
                      ├─ Home / Limit / MoveTo / SafeMove 状态机
                      ├─ RobotArmSensor 集中传感器语义
                      └─ RobotArmDriver Adapter
                           └─ 既有 DMA / Timer / STEP / DIR 驱动
```

## 3. 为什么引入 RobotArm

RobotArm 的目的不是重写 DMA，而是在已经工作的三路步进驱动上增加统一的机械语义：

- 用 X/Y/Z 和统一正负方向描述机械运动。
- 集中维护 current、target、homed、position_valid、axis state 和 arm state。
- 只有取得可信的底层完成证据后才提交坐标。
- 把传感器、硬限位、软限位、Home、超时、停止和组合运动收口到同一管理层。
- 为 Android 提供稳定的 V2 ACK/STATUS/EVENT 合同，不要求 Android 理解 DMA 细节。
- 保留 V1 兼容路径，但显式使绕过管理层的旧直驱动作破坏坐标有效性。

## 4. XYZ Driver 映射

| 逻辑轴 | STEP/驱动入口 | Timer | DMA | DIR 管理 |
|---|---|---|---|---|
| X | PB11 / `stepdma_pb11_*` | TIM5 | DMA2 Channel 2 | 74HC595 shadow byte 1 bit 4，由 Adapter 集中设置 |
| Y | PB10 / `Stepper2_*` | TIM6 | DMA2 Channel 3 | `Stepper2_SetDirection/Start` 内部管理既有 DIR2 |
| Z | PB13 / `PU3_Stepper_*` | TIM7 | DMA2 Channel 4 | `PU3_Stepper_Start` 内部管理既有 DIR3 |

RobotArm 改造没有更换 DMA 算法、Timer/DMA 分配或 STEP 引脚。方向的机械一致性仍需 Phase 10.6 实机确认；若不一致，应只在 Driver Adapter 的集中映射处修正，不改变坐标系定义。

## 5. 传感器 S1～S3

| 编号 | 机械定义 | 逻辑作用 |
|---|---|---|
| S1 | X_HOME | X 轴物理零点/Home |
| S2 | Y_HOME | Y 轴物理零点/Home |
| S3 | Z_HOME | Z 轴物理零点/Home |

三路传感器通过集中 Sensor Adapter 转换成统一逻辑值：released=0、triggered=1。当前有效电平配置均为 1，但 HC165 采集层已经取反，最终电平仍必须通过实机逐路确认。若实测相反，只修改集中式有效电平配置，不在状态机内增加散落取反。

## 6. XYZ 坐标系

| 方向 | 机械定义 |
|---|---|
| X+ | 向外，离开 S1 |
| X- | 回 S1 |
| Y+ | 离开 S2 |
| Y- | 回 S2 |
| Z+ | 向下，离开 S3；坐标数值增大 |
| Z- | 向上回 S3；坐标数值减小 |

Z=0 定义为最高 Home 侧逻辑位置；Z 数值越大，机械位置越低。当前 Home Offset 均为 0，尚未做机械偏移标定。

## 7. position_valid 与 homed

这两个状态不可互相替代：

- `homed=1`：该轴在本次上电后曾成功完成 Home，表示拥有成功建立原点的历史。
- `position_valid=1`：当前逻辑坐标仍可信，可以用于绝对运动、MoveTo 和 SafeMove。
- 上电初始化时二者都为 0，current/target 数值虽然初始化为 0，但不能据此认为坐标可信。
- Home 开始时对应轴的 homed 和 position_valid 都清零；Home 成功后两者同时置 1。
- 绝对单轴、MoveTo 和 SafeMove 会检查 position_valid；当前相对运动入口没有显式检查该标志，而是以保存的 current 计算目标。V2 Android 合同已强制要求 valid=1 才能发送相对运动，后续维护不得把这一边界误认为 MCU 已具备同等保护。
- 普通运动的 Stop、Timeout、Driver Error 会使活动轴 position_valid 清零，但保留其既有 homed 历史；负向首次命中 S1～S3 时传感器反而重新确认 position_valid=1。
- Home 过程失败或停止会清除该 Home 轴的 homed 和 position_valid。
- ClearError 只清错误状态，不恢复 position_valid，也不伪造 homed。
- 旧 V1 直驱入口执行后会主动使对应轴 position_valid 失效。

## 8. Phase 7.5 运动完成判断

管理层不再把底层 Busy 变为 0 直接等同于成功。单轴动作只有同时满足以下条件才提交 current=target：

1. Driver Busy 已变为 0；
2. remaining steps 等于 0；
3. completed steps 大于或等于本次 command steps。

如果 Busy 已结束但步数证据不完整，动作按 Driver Error 失败，活动轴 position_valid 失效。这样可以区分正常走完、被停止、DMA 异常退出和未完成的动作。

## 9. 三轴 Home 状态机

每轴使用非阻塞的单阶段快速寻零流程：

1. CHECK_SENSOR：读取对应 Home 传感器。
2. 若命令开始时已 Active，不输出 STEP，直接将 current/target 设为 0，并置 homed=1、position_valid=1。
3. 若未 Active，SEEK_FAST 沿负方向寻找 Home 传感器。
4. 运行中首次 Active 时立即停止，禁止续装 DMA chunk，并将 current/target 设为 0、homed=1、position_valid=1。

Home 具备每轴 MAX_STEPS 和 TIMEOUT 约束；只有传感器未触发且寻边步数耗尽或总时间超时才会失败。

HomeAll 不是并行回零，固定顺序为 Z → Y → X。任何一轴失败都会终止 HomeAll，不继续启动后续轴。

## 10. Hard Limit

Hard Limit 使用集中式传感器逻辑并按运动方向判断：

- X- 已在 S1 零点时拒绝启动；从正坐标负向首次命中 S1 时立即归零。
- Y- 已在 S2 零点时拒绝启动；从正坐标负向首次命中 S2 时立即归零。
- Z- 已在 S3 零点时拒绝启动；从正坐标负向首次命中 S3 时立即归零。
- 从已触发零点向正方向离开是允许的。

S1～S3 Active 分别是 X/Y/Z 实际坐标为 0 的硬件证据。运行中首次命中时立即停止当前
DMA/STEP，禁止续装下一 chunk，并将 current_position=0、homed=1、position_valid=1。
Home 和目标为 0 的普通单轴、MOVE_TO、SafeMove 均以 COMPLETED/OK 收尾；只有已经在零点
仍请求继续负方向时才返回 LIMIT。

## 11. Soft Limit

Soft Limit 是每轴可配置的目标范围检查：

- 只有对应 `ROBOT_ARM_*_LIMIT_ENABLED=1` 时生效。
- 启动运动前检查目标是否位于 `[MIN_POSITION, MAX_POSITION]`。
- 超界返回 `ROBOT_ARM_ERR_LIMIT`，不产生 STEP。
- 它不替代 S1～S3 物理零点语义；运行期间仍持续检查是否已经在零点还要求继续负向。

当前生产配置三轴 Soft Limit 均禁用，MIN 为 0，MAX 为 INT32_MAX。这些不是已标定机械范围。

## 12. MoveTo

MoveTo 接受 XYZ 三轴绝对目标，要求：

- 三轴 position_valid 均为 1；
- 传感器快照已经就绪；
- 目标通过已启用的 Soft Limit、零点方向和姿态安全检查。

执行顺序固定为 X → Y → Z。每轴必须正常完成并提交坐标后才启动下一轴；目标未变化的轴直接跳过。不做三轴同步、不做插补。目标为 0 的轴首次命中 S1～S3 会作为正常完成继续下一轴；越零 LIMIT、Timeout、Stop 或其他 Error 才会终止组合任务。

当前生产态的姿态安全检查没有已标定碰撞区，默认通过；不能把这一点解释为已完成机械碰撞模型。

## 13. SafeMoveTo

SafeMoveTo 要求三轴 position_valid 均为 1，并要求 SafeMove 已启用且 SAFE_Z 落在声明的 Z 范围内。

当前坐标定义为 Z 数值越大越低：

- 如果 X/Y 都不变化，只执行最终 Z。
- 如果 X/Y 需要变化且 currentZ > SAFE_Z，先把 Z 抬到 SAFE_Z。
- 然后依次执行 X、Y。
- 最后执行目标 Z。
- 如果 currentZ <= SAFE_Z，则跳过 Raise Z。

X/Y 阶段会检查当前 Z 是否位于安全高度；不满足时返回 INTERLOCK。每一阶段正常完成后才允许启动下一阶段。

当前生产配置 `ROBOT_ARM_SAFE_MOVE_ENABLED=0`、`ROBOT_ARM_SAFE_Z_POSITION=0`，因此生产态会返回 CONFIG，不会使用未标定 SAFE_Z 驱动实机。

## 14. Stop、Timeout、Limit、Error 后的坐标策略

### 正常完成

- 只有完整步数证据成立后，活动轴 current 才提交为 target。
- position_valid 保持有效。

### Stop

- 立即停止所有活动或底层仍 Busy 的轴。
- 正在运动的轴 position_valid 清零；未参与运动的轴保持原状态。
- 正在 Home 的轴同时清除 homed。
- 已经正常完成并提交的前序组合轴保留其已提交坐标。
- 清除组合状态，绝不继续启动后续轴。

### Limit / Timeout / Driver Error

- 活动轴停止并失去 position_valid。
- current 不提交为未完成 target，仍保留最后一次可信提交值，但必须结合 position_valid=0 判断，不得继续把该数值当作真实位置。
- 普通运动保留 homed 历史；Home 失败清除当前 Home 轴 homed。
- arm_state 进入 ERROR，后续轴不启动。

### Interlock / Sensor / 配置错误

- 若在动作启动前发现，拒绝启动，不产生 STEP。
- 若在组合阶段切换时发现，组合任务进入 ERROR，不启动后续阶段。
- ClearError 不恢复坐标可信度；需要根据现场状态重新 Home。

## 15. RobotArm Driver Adapter 职责

Driver Adapter 是 RobotArm 与既有底层驱动之间唯一的硬件差异适配层，职责包括：

- 将逻辑轴 X/Y/Z 映射到 PB11、PB10、PB13 三套既有驱动。
- 将逻辑方向正/负映射到底层 DIR 电平或既有方向接口。
- 统一 Start、Stop、IsBusy、GetRemainingSteps、GetCompletedSteps。
- 隔离 RobotArm 核心与 GPIO、74HC595、Timer、DMA 通道细节。

它不负责 Home、坐标提交、限位状态机、组合运动顺序或协议。机械方向实测不一致时，应最小修改 Adapter 方向映射，不能改坐标定义或重写 DMA。

## 16. Protocol V2 架构

V2 使用固定 24B、Little Endian、CRC16-CCITT-FALSE 和 16 位 SEQ。RobotArmProtocol 将有效 V2 请求转换为管理层调用，并生成：

- ACK：同步表示接受或拒绝；
- STATUS_RSP：分页状态快照；
- EVENT：异步任务唯一最终结果。

发送侧拥有 4 帧静态保存队列和 ACK/STATUS/EVENT overflow 统计。EVENT 生命周期为 produced → queued → consumed：在 EVENT 被可靠保存和发送消费前，active CMD/SEQ 不清除，新运动请求不能覆盖旧任务。发送暂时失败时保留队首重试。

STOP 使用两套 SEQ：STOP ACK 使用 STOP 请求 SEQ，被中止任务的 STOPPED EVENT 使用原任务 SEQ。完成与 STOP 临界竞态只允许一个最终终态。

## 17. V1/V2 共存方式

V1 和 V2 共用 USART1 RS485 字节流：

- 第二字节为 `0xFE` 时解析为 V2 固定 24B。
- 其他 `0xAA` 起始帧按 V1 固定 10B 解析。
- V1/V2 各有独立的 4 帧接收队列。
- V1 继续进入旧 `parse_frame/handle_command`；V2 进入 RobotArmProtocol。
- 主循环限制每轮输入字节和处理帧数，避免通信任务长期阻塞其他业务。
- V2 发送队列空间不足时暂缓取出新 V2 请求，避免 ACK/EVENT 被提前消费后丢失。

V1 协议格式、8 位累加和校验语义和旧业务没有被 V2 重写。

## 18. 主要新增和修改文件

### RobotArm 核心

- `User/robot_arm.c`、`User/robot_arm.h`：XYZ 状态、坐标、Home、Limit、MoveTo、SafeMove、Stop 和完成判断。
- `User/robot_arm_config.h`：集中式传感器电平、Home、Soft Limit、Safe Z、默认速度和超时配置。
- `User/robot_arm_driver.c`、`User/robot_arm_driver.h`：XYZ Driver Adapter。
- `User/robot_arm_sensor.c`、`User/robot_arm_sensor.h`：S1～S3 快照和统一触发语义。

### V2 协议

- `User/protocol_v2.c`、`User/protocol_v2.h`：V1/V2 流解析、固定 24B 编解码、CRC 和接收队列。
- `User/robot_arm_protocol.c`、`User/robot_arm_protocol.h`：RobotArm CMD、ACK、STATUS、EVENT、SEQ 和可靠发送生命周期。

### 集成与兼容

- `User/main.c`：初始化和轮询 RobotArm/V2，接入 USART1 字节流。
- `User/protocol.c`：保留旧 V1 命令，并在旧步进直驱后使对应 RobotArm 坐标失效。
- `User/shift_register_input.c`：把 HC165 快照同步给 RobotArmSensor。
- `User/step_dma.c`、`User/step_dma.h`：为管理层补齐独立轴 Start/Stop/Busy/完成步数接口，保留既有资源映射。
- `Project/Test-standard-lib.uvprojx`：纳入新增模块。

### 测试与联调文档

- `Tests/robot_arm_logic_test.c`：Home、Limit、MoveTo、SafeMove、Stop、超时和坐标策略逻辑测试。
- `Tests/protocol_v2_logic_test.c`：固定帧、CRC、V1/V2 共存、队列和 SEQ 测试。
- `Tests/robot_arm_protocol_logic_test.c`：ACK/EVENT/STATUS、终态唯一性、队列恢复和 STOP 临界竞态测试。
- `docs/phase_10_6_hardware_validation.md`：实机协议、传感器、方向和机械参数标定清单。

## 19. 旧直驱入口与过渡方案

以下 V1 命令仍直接操作底层步进驱动，属于兼容/调试过渡路径，不属于 RobotArm V2 坐标运动合同：

| V1 CMD | 当前作用 | RobotArm 影响 |
|---:|---|---|
| `0x1B` | 直接驱动 X / PB11 | X position_valid 失效 |
| `0x1C` | 直接驱动 Y / PB10 | Y position_valid 失效 |
| `0x1E` | 直接驱动 Z / PU3 | Z position_valid 失效 |
| `0x1F` | 查询既有 PB10 运行状态 | 不建立 RobotArm 坐标 |
| `0x20` | PB10 带传感器停止直驱 | Y position_valid 失效 |

这些入口保留是为了旧设备兼容和板端调试，不应作为 Android 新 RobotArm 功能的常规控制通道。旧直驱与 V2 绝对坐标混用后，必须重新 Home 才能恢复可信坐标。

## 20. 当前仍未实机确认的内容

截至 Phase 10.6 清单建立时，以下内容只有静态检查或宿主机逻辑测试证据，没有完成目标板确认：

- USART1 RS485 上 V1/V2 24B 实际收发、连续帧、SEQ、CRC、ACK/STATUS/EVENT。
- S1～S3 实际 released/triggered 电平与集中取反配置。
- X+/X-/Y+/Y-/Z+/Z- 的真实机械方向。
- 每轴 Home fast/slow/backoff/max steps/timeout 和重复精度。
- 每轴机械安全行程、Soft Limit MIN/MAX 与安全余量。
- SAFE_Z 实际无碰撞高度。
- MoveToSafe 的真实 Raise Z → X → Y → Final Z 顺序及中途 Stop/Limit 行为。
- 9600 baud 下持续通信与真实机械运动并发稳定性。

因此当前逻辑测试通过不能替代 Keil 完整构建、逻辑分析仪测量和板端机械验证。

## 21. 当前生产禁用状态

非 `ROBOT_ARM_LOGIC_TEST` 生产构建当前保持：

- `ROBOT_ARM_X/Y/Z_HOME_ENABLED = 0`；
- 三轴 Home MAX_STEPS、TIMEOUT、FAST_SPEED、SLOW_SPEED、BACKOFF_STEPS 为 0；
- `ROBOT_ARM_X/Y/Z_LIMIT_ENABLED = 0`；
- `ROBOT_ARM_SAFE_MOVE_ENABLED = 0`；
- `ROBOT_ARM_SAFE_Z_POSITION = 0`，但该值尚未标定，不能启用使用。

这些禁用状态是上板前安全门禁，不是功能缺失。只有 Phase 10.6 对应参数取得实机记录后才能逐项启用。

## 22. 两个既有编译 warning

### hc165_index：2000 → 208

`stepMotorA` 使用位置初始化。按当前结构体字段顺序，常量 2000 实际落入 `uint8_t hc165_index`，编译转换后为 208。当前两片 HC165 的合法索引只有 0～1。

当前全工程搜索结果：

- 找到该非法初始写入；
- 找到后续传感器绑定对 hc165_index 的重新赋值；
- 尚未找到绑定前读取该初始值的路径。

现有历史无法证明 2000 原本应属于哪个字段，因此 Phase 10.6 仍不盲改。但正式实机版本冻结前必须确认原设计意图并消除该非法初始化。

### reback_steps

- `stepMotorA` 是静态全局对象，省略显式初始化后由 C 语言规则默认置 0。
- 当前全工程未发现 reback_steps 的实际读写。
- 当前没有证据表明它影响步进、Home 或 RobotArm，暂不修改。

## 23. 构建与验证状态

- RobotArm、SafeMove、V2 和可靠性逻辑测试已在宿主机通过。
- ARM Cortex-M3 目标语法检查已通过。
- 当前环境没有可用的 Keil UV4/ARMCC 工具链。
- Keil 完整工程构建尚未完成。
- 目标板 RS485、传感器、方向、Home、Soft Limit、SAFE_Z 和 MoveToSafe 实机验证尚未完成。

正式发布条件应同时包括：Keil 全工程无新增错误、两个既有 warning 得到明确处理、Phase 10.6 实机记录完整、最终机械参数经过复核并写入生产配置。
