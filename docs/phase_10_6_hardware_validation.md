# RobotArm Phase 10.6 实机联调与机械参数标定记录

> 状态：待连接并识别目标板后执行。本文只记录验证步骤和实测值，不修改生产参数。
>
> 安全原则：任何未标定值不得写入生产配置；任何测试开始前必须确认急停可用、机构周围无人、运动方向有足够机械余量。

## 1. 当前固件安全基线

| 项目 | 当前生产构建状态 | 结论 |
|---|---:|---|
| X/Y/Z Home | `ROBOT_ARM_*_HOME_ENABLED = 0` | 禁止自动 Home |
| X/Y/Z Soft Limit | `ROBOT_ARM_*_LIMIT_ENABLED = 0` | 未标定前保持关闭 |
| SafeMove | `ROBOT_ARM_SAFE_MOVE_ENABLED = 0` | 未标定 SAFE_Z 前保持关闭 |
| Position/Homed | 上电均为 0 | 不允许为了调试伪造 |
| V2 波特率 | 9600 baud | 不修改 |
| V2 帧长 | 固定 24B | 不修改 |

当前电脑串口枚举结果：仅发现 `COM1`，没有取得可证明它是目标板的设备信息。因此不得向 `COM1` 发送探测命令。连接目标板后补录：

| 项目 | 实测记录 |
|---|---|
| 目标板串口 | 待测 |
| USB/串口设备名称 | 待测 |
| RS485 转换器型号 | 待测 |
| 供电电压 | 待测 |
| 固件版本/提交 | 待测 |
| 测试日期/人员 | 待测 |

## 2. V2 协议核对

线格式固定为：

```text
AA FE CMD SEQ_L SEQ_H D0..D15 55 CRC_L CRC_H
```

- CRC：CRC16/CCITT-FALSE，多项式 `0x1021`，初值 `0xFFFF`，覆盖字节 0～21。
- SEQ：16 位小端，响应必须原样返回请求 SEQ。
- 本阶段允许请求：`ARM_STATUS(0x38)`、`ARM_CLEAR_ERROR(0x37)`。
- 本阶段禁止请求：Home、MoveAxis、MoveTo、MoveToSafe 及任何会产生 STEP 的命令。

以下四帧已按当前固件算法计算 CRC，可作为首次无 STEP 联调基准：

```text
STATUS page 0, SEQ=0x1001
AA FE 38 01 10 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 55 8F 51

STATUS page 1, SEQ=0x1002
AA FE 38 02 10 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 55 6D 33

STATUS page 2, SEQ=0x1003
AA FE 38 03 10 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 55 54 DC

CLEAR_ERROR, SEQ=0x1004
AA FE 37 04 10 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 55 2F F2
```

预期响应：

- STATUS：`CMD=0x72`，SEQ 与请求一致，每个请求只返回一帧 STATUS。
- CLEAR_ERROR：`CMD=0x70`，`D0=0x37`，`D1=ACK 状态`，`D2=结果码`。
- 每帧必须为 24B，帧头 `AA FE`、尾标记 `55`、CRC 正确。
- 协议阶段不得观察到 STEP 引脚脉冲。

### 2.1 STATUS page 0 解码

| 数据字节 | 含义 |
|---|---|
| D0 | 固定为 page 0 |
| D1 | arm_state |
| D2 | operation |
| D3 | error_code |
| D4/D5/D6 | X/Y/Z axis_state |
| D7 bit0/1/2 | X/Y/Z homed |
| D8 bit0/1/2 | X/Y/Z position_valid |
| D9 bit0/1/2 | S1/S2/S3 sensor_flags；bit3～bit7 保留 |
| D10 | last_move_end_reason |
| D11 | home_state |
| D12 | move_to_state |
| D13 | safe_move_state |

### 2.2 STATUS page 1/2 解码

| 页面 | D0～D3 | D4～D7 | D8～D11 | D12 |
|---|---|---|---|---|
| page 1 | current X, int32 LE | current Y, int32 LE | current Z, int32 LE | 1 |
| page 2 | target X, int32 LE | target Y, int32 LE | target Z, int32 LE | 2 |

### 2.3 协议实测记录

| 用例 | 请求 SEQ | RX 24B | CRC | 响应 CMD/SEQ | 数据正确 | STEP=0 | 结果/原始帧 |
|---|---:|---|---|---|---|---|---|
| STATUS page 0 | 0x1001 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| STATUS page 1 | 0x1002 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| STATUS page 2 | 0x1003 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| CLEAR_ERROR | 0x1004 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| 连续 STATUS 请求 | 连续递增 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| V1/V2 交替请求 | 分别记录 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |

协议阶段通过门槛：以上项目全部通过，逻辑分析仪确认 STEP 引脚无脉冲，才能进入传感器测试。

## 3. S1～S3 传感器实测

固定机械定义：

- S1 = X_HOME
- S2 = Y_HOME
- S3 = Z_HOME / Z upper limit

当前集中配置均为 `ROBOT_ARM_S1/S2/S3_TRIGGERED_LEVEL = 1`。HC165 采集层已经按位取反，最终必须以 STATUS page 0 的 D9 实测为准。

| 传感器 | D9 掩码 | released 实测电平 | released flag | triggered 实测电平 | triggered flag | 抖动/稳定性 | 结论 |
|---|---:|---:|---:|---:|---:|---|---|
| S1 X_HOME | 0x01 | 待测 | 必须 0 | 待测 | 必须 1 | 待测 | 待测 |
| S2 Y_HOME | 0x02 | 待测 | 必须 0 | 待测 | 必须 1 | 待测 | 待测 |
| S3 Z_HOME | 0x04 | 待测 | 必须 0 | 待测 | 必须 1 | 待测 | 待测 |

若结果相反，只允许调整 `robot_arm_config.h` 中对应的集中式 `ROBOT_ARM_S*_TRIGGERED_LEVEL`；禁止在 RobotArm 状态机中增加局部取反。

## 4. XYZ 方向验证

固定坐标定义不得改变：

| 逻辑方向 | 机械方向 | 归零/限位关系 | 实测结果 |
|---|---|---|---|
| X+ | 向外 | 离开 S1 | 待测 |
| X- | 向内 | 回 S1 | 待测 |
| Y+ | 离开 S2 | 离开 S2 | 待测 |
| Y- | 回 S2 | 回 S2 | 待测 |
| Z+ | 向下 | 离开 S3 | 待测 |
| Z- | 向上 | 回 S3 | 待测 |

重要限制：当前 V2 单轴运动在 `position_valid=0` 时会返回 `ROBOT_ARM_ERR_POSITION_UNKNOWN`。因此 Home 前不得通过伪造 position_valid 来点动。方向验证只能在以下条件之一满足后执行：

1. 已确认一种现有、可限步、可限速且轴映射明确的板端调试入口；或
2. 调试器受控调用现有 `RobotArmDriver_Start(axis, direction, steps, speed)`，并由现场人员确认轴、方向、步数、速度及急停。

在入口未确认前，本节保持阻塞，不发送任何探索性运动命令。若方向相反，只允许修改 `robot_arm_driver.c` 的集中方向映射，不改坐标定义、不改 DMA。

## 5. Home 参数标定

当前代码的真实配置结构是：每轴独立 `MAX_STEPS/TIMEOUT/OFFSET`，三轴共用 `FAST_SPEED/SLOW_SPEED/BACKOFF_STEPS`。不能把尚不存在的逐轴速度宏当作已实现参数。

### 5.1 原始测量记录

| 轴 | 最坏回零距离 steps | 可靠释放最小 backoff | Fast 候选 | Slow 候选 | 最长正常时间 ms | 重复次数 | 失败次数 |
|---|---:|---:|---:|---:|---:|---:|---:|
| Z | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| Y | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| X | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |

### 5.2 启用顺序

1. 只启用 Z Home，低速标定并重复验证。
2. Z 通过后只启用 Y Home。
3. Y 通过后只启用 X Home。
4. 三轴单独通过后，最后验证 `RobotArm_Home()` 的 Z → Y → X 顺序。

每次必须记录：homed、position_valid、current、传感器释放/触发、EVENT、耗时、是否 Timeout、是否 Limit 误判。

| 轴/轮次 | 起始位置 | 最终 current | homed | valid | 传感器边沿 | EVENT | 耗时 ms | 结果 |
|---|---:|---:|---:|---:|---|---|---:|---|
| Z-1 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| Z-2 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| Z-3 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| Y-1～N | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| X-1～N | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| HomeAll-1～N | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |

## 6. Soft Limit 标定

| 轴 | 机械安全最大位置 | 预留安全余量 | 软件 MIN | 软件 MAX 候选 | MAX-少量 | MAX | MAX+1 无 STEP | MIN-1 无 STEP |
|---|---:|---:|---:|---:|---|---|---|---|
| X | 待测 | 待测 | 0 | 待测 | 待测 | 待测 | 待测 | 待测 |
| Y | 待测 | 待测 | 0 | 待测 | 待测 | 待测 | 待测 | 待测 |
| Z | 待测 | 待测 | 0 | 待测 | 待测 | 待测 | 待测 | 待测 |

只有 Home 重复精度通过后才能执行。软件 MAX 必须小于机械安全最大位置，并保留实测安全余量。

## 7. SAFE_Z 标定

坐标定义：Z=0 最高，Z 数值越大位置越低。

| 项目 | 实测值/结果 |
|---|---|
| X/Y 全工作范围无碰撞的最低安全高度 | 待测 |
| `ROBOT_ARM_SAFE_Z_POSITION` 候选 | 待测 |
| currentZ > SAFE_Z 且 X 变化：RaiseZ → X → FinalZ | 待测 |
| currentZ > SAFE_Z 且 XY 变化：RaiseZ → X → Y → FinalZ | 待测 |
| currentZ <= SAFE_Z：跳过 RaiseZ | 待测 |

SAFE_Z 未取得实测值前，`ROBOT_ARM_SAFE_MOVE_ENABLED` 必须保持 0。

## 8. MoveToSafe 板端验证

| 顺序 | 用例 | 预期 | 实测 |
|---:|---|---|---|
| 1 | XYZ 全相同 | 无 STEP，完成一次 | 待测 |
| 2 | 只移动 Z | 仅 Z 运动 | 待测 |
| 3 | Safe 高度下小范围 X | 仅 X 运动 | 待测 |
| 4 | Safe 高度下小范围 Y | 仅 Y 运动 | 待测 |
| 5 | Z 较低且 X 变化 | RaiseZ → X → FinalZ | 待测 |
| 6 | Z 较低且 XY 变化 | RaiseZ → X → Y → FinalZ | 待测 |
| 7 | 运动途中 STOP | 停止且不启动后续轴 | 待测 |
| 8 | Z- 中触发 S3 | LIMIT，停止且不启动后续轴 | 待测 |
| 9 | X- 中触发 S1 | LIMIT，停止且不启动后续轴 | 待测 |
| 10 | Y- 中触发 S2 | LIMIT，停止且不启动后续轴 | 待测 |

## 9. 最终机械参数表

只有来源为“实机确认”的值才能进入最终列。

| 参数 | 当前生产值 | 实测原始值 | 安全余量依据 | 最终候选值 | 实机确认 | 已写入配置 |
|---|---:|---:|---|---:|---|---|
| `ROBOT_ARM_HOME_FAST_SPEED` | 0 | 待测 | 待测 | 待定 | 否 | 否 |
| `ROBOT_ARM_HOME_SLOW_SPEED` | 0 | 待测 | 待测 | 待定 | 否 | 否 |
| `ROBOT_ARM_HOME_BACKOFF_STEPS` | 0 | 待测 | 待测 | 待定 | 否 | 否 |
| `ROBOT_ARM_X_HOME_MAX_STEPS` | 0 | 待测 | 待测 | 待定 | 否 | 否 |
| `ROBOT_ARM_Y_HOME_MAX_STEPS` | 0 | 待测 | 待测 | 待定 | 否 | 否 |
| `ROBOT_ARM_Z_HOME_MAX_STEPS` | 0 | 待测 | 待测 | 待定 | 否 | 否 |
| `ROBOT_ARM_X_HOME_TIMEOUT_MS` | 0 | 待测 | 待测 | 待定 | 否 | 否 |
| `ROBOT_ARM_Y_HOME_TIMEOUT_MS` | 0 | 待测 | 待测 | 待定 | 否 | 否 |
| `ROBOT_ARM_Z_HOME_TIMEOUT_MS` | 0 | 待测 | 待测 | 待定 | 否 | 否 |
| `ROBOT_ARM_X_HOME_OFFSET` | 0 | 待测 | 待测 | 待定 | 否 | 否 |
| `ROBOT_ARM_Y_HOME_OFFSET` | 0 | 待测 | 待测 | 待定 | 否 | 否 |
| `ROBOT_ARM_Z_HOME_OFFSET` | 0 | 待测 | 待测 | 待定 | 否 | 否 |
| `ROBOT_ARM_X_MAX_POSITION` | INT32_MAX | 待测 | 待测 | 待定 | 否 | 否 |
| `ROBOT_ARM_Y_MAX_POSITION` | INT32_MAX | 待测 | 待测 | 待定 | 否 | 否 |
| `ROBOT_ARM_Z_MAX_POSITION` | INT32_MAX | 待测 | 待测 | 待定 | 否 | 否 |
| `ROBOT_ARM_SAFE_Z_POSITION` | 0 | 待测 | 待测 | 待定 | 否 | 否 |

最终启用门禁：对应参数全部取得实机记录后，才允许依次设置 Home enabled、三轴 Soft Limit enabled、SafeMove enabled。

## 10. 两个既有 warning 冻结项

### `hc165_index`

- 当前类型：`uint8_t`。
- `stepMotorA` 位置初始化中的 `2000` 实际转换为 `208`。
- 按当前两片 HC165，合法索引只有 0～1。
- 当前搜索仅发现初始化和后续绑定写入，尚未发现绑定前读取。
- 正式实机版本冻结前必须确认原设计意图并消除此非法初始化；本阶段暂不修改。

### `reback_steps`

- `stepMotorA` 为静态全局对象，省略初始化后默认值为 0。
- 当前未发现任何读写路径。
- 本阶段暂不修改。
