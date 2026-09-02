# RobotArm V2 通信协议合同

本文档是 Android 与 STM32 RobotArm 模块之间的 V2 协议合同。Android 端不需要读取 MCU 源码即可完成编码、解码、请求跟踪和错误处理。

## 1. 串口与字节序

| 项目 | 固定值 |
|---|---|
| 物理接口 | USART1 RS485，半双工 |
| 波特率 | 9600 baud |
| 数据格式 | 8 data bits、1 stop bit、无校验、无硬件流控（8N1） |
| V2 帧长 | 固定 24 字节 |
| 多字节整数 | Little Endian |
| 有符号坐标/步数 | `int32`，二进制补码 |
| 速度 | `uint32`，单位为 steps/s；0 表示使用对应轴默认速度 |

除本合同明确允许的值外，Android 发送端必须把所有保留字节置 0。

## 2. 固定 24B 帧格式

| 字节偏移 | 长度 | 名称 | 定义 |
|---:|---:|---|---|
| 0 | 1 | HEAD | 固定 `0xAA` |
| 1 | 1 | V2_MARK | 固定 `0xFE` |
| 2 | 1 | CMD | 命令或响应类型 |
| 3 | 2 | SEQ | 16 位无符号序号，Little Endian：`SEQ_L, SEQ_H` |
| 5 | 16 | DATA | `DATA[0]`～`DATA[15]` |
| 21 | 1 | TAIL | 固定 `0x55` |
| 22 | 2 | CRC16 | Little Endian：`CRC_L, CRC_H` |

线格式：

```text
AA FE CMD SEQ_L SEQ_H D0 D1 ... D15 55 CRC_L CRC_H
```

帧没有可变长度字段，也不允许省略末尾为 0 的 DATA 字节。

## 3. CRC16-CCITT-FALSE

| 参数 | 值 |
|---|---:|
| Width | 16 |
| Poly | `0x1021` |
| Init | `0xFFFF` |
| RefIn | false |
| RefOut | false |
| XorOut | `0x0000` |
| Check (`"123456789"`) | `0x29B1` |

CRC 覆盖帧偏移 0～21，共 22 字节，即从 `0xAA` 开始并包含偏移 21 的 `0x55`。CRC 本身不参与计算。计算结果按 Little Endian 放入偏移 22、23。

参考伪代码：

```text
crc = 0xFFFF
for byte in frame[0..21]:
    crc ^= byte << 8
    repeat 8 times:
        if crc & 0x8000:
            crc = (crc << 1) ^ 0x1021
        else:
            crc = crc << 1
        crc &= 0xFFFF
frame[22] = crc & 0xFF
frame[23] = (crc >> 8) & 0xFF
```

## 4. SEQ 语义

- SEQ 是 Android 分配的 16 位请求标识，范围 `0..65535`，允许从 `65535` 自然回绕到 `0`。
- MCU 不生成新的业务 SEQ；ACK 使用原请求 SEQ，STATUS_RSP 使用 STATUS 查询 SEQ；page3 中另行携带原异步请求的 CMD/SEQ。
- Android 应以 `CMD + SEQ` 匹配 ACK；读取 page3 时必须使用 DATA 中的原 CMD/SEQ 匹配异步请求。
- MCU 当前没有“按 SEQ 去重并重放旧响应”的机制。Android 不得把超时重发视为幂等操作；重复发送运动请求可能被当作新的请求或返回 BUSY。
- 同一时刻建议只保留一个未完成的异步运动/Home 请求。ACK 后必须轮询 `0x38 page3`，读到原 CMD/SEQ 对应终态后才算完整结束。
- STATUS 可以在任务执行期间查询；STOP 可以在任务执行期间发送。

## 5. CMD 总表

### 5.1 Android → MCU

| CMD | 数值 | 名称 | 响应方式 |
|---|---:|---|---|
| ARM_HOME | `0x30` | 三轴 HomeAll | ACK，终态通过 page3 查询 |
| ARM_HOME_AXIS | `0x31` | 单轴 Home | ACK，终态通过 page3 查询 |
| ARM_MOVE_AXIS_ABS | `0x32` | 单轴绝对位置运动 | ACK，终态通过 page3 查询 |
| ARM_MOVE_AXIS_REL | `0x33` | 单轴相对步数运动 | ACK，终态通过 page3 查询 |
| ARM_MOVE_TO | `0x34` | 默认 XYZ 顺序运动；可选关节同步 | ACK，终态通过 page3 查询 |
| ARM_MOVE_TO_SAFE | `0x35` | 带 Safe Z 的 XYZ 顺序运动 | ACK，终态通过 page3 查询 |
| ARM_STOP | `0x36` | 停止当前 RobotArm 任务 | STOP 自身 ACK；原任务终态通过 page3 查询 |
| ARM_CLEAR_ERROR | `0x37` | 清除管理层 ERROR | 仅 ACK |
| ARM_STATUS | `0x38` | 查询状态页 | 仅 STATUS_RSP，不额外发送 ACK |

### 5.2 MCU → Android

| CMD | 数值 | 名称 |
|---|---:|---|
| ARM_ACK | `0x70` | 请求接受/拒绝响应 |
| ARM_EVENT | `0x71` | page3 复用的异步终态语义标识；MCU 不主动发送 |
| ARM_STATUS_RSP | `0x72` | STATUS 分页响应 |

## 6. axis 枚举

| 值 | 轴 |
|---:|---|
| `0x00` | X |
| `0x01` | Y |
| `0x02` | Z |
| `0xFF` | 无单一轴；用于 HomeAll、MoveTo、MoveToSafe 等组合任务的 EVENT axis 字段 |

请求中的 axis 只允许 `0x00..0x02`；Android 不得在请求中发送 `0xFF`。

## 7. 请求 DATA[16]

### 7.1 ARM_HOME (`0x30`)

| DATA 字节 | 定义 |
|---|---|
| D0～D15 | 保留，必须为 0 |

按 Z → Y → X 顺序执行 HomeAll。当前生产固件尚未标定时会因配置禁用而拒绝，详见 `ROBOT_ARM_ERR_CONFIG`。

### 7.2 ARM_HOME_AXIS (`0x31`)

| DATA 字节 | 定义 |
|---|---|
| D0 | axis：0=X，1=Y，2=Z |
| D1～D15 | 保留，必须为 0 |

### 7.3 ARM_MOVE_AXIS_ABS (`0x32`)

| DATA 字节 | 类型 | 定义 |
|---|---|---|
| D0 | uint8 | axis：0=X，1=Y，2=Z |
| D1～D4 | int32 LE | 绝对目标逻辑坐标 |
| D5～D8 | uint32 LE | 速度，steps/s；0 使用该轴默认速度 |
| D9～D15 | - | 保留，必须为 0 |

要求对应轴 `position_valid=1`。

### 7.4 ARM_MOVE_AXIS_REL (`0x33`)

| DATA 字节 | 类型 | 定义 |
|---|---|---|
| D0 | uint8 | axis：0=X，1=Y，2=Z |
| D1～D4 | int32 LE | 相对增量；正数为正方向，负数为负方向 |
| D5～D8 | uint32 LE | 速度，steps/s；0 使用该轴默认速度 |
| D9～D15 | - | 保留，必须为 0 |

当前实现以 current 逻辑坐标计算相对目标，MCU 入口本身没有显式拒绝 `position_valid=0`。因此 Android 合同强制要求：只有对应轴 `position_valid=1` 时才允许发送相对运动；不得利用该实现边界绕过 Home。该命令不会伪造 Homed 状态。

### 7.5 ARM_MOVE_TO (`0x34`)

| DATA 字节 | 类型 | 定义 |
|---|---|---|
| D0～D2 | int24 LE | X 绝对目标坐标 |
| D3～D5 | int24 LE | Y 绝对目标坐标 |
| D6～D8 | int24 LE | Z 绝对目标坐标 |
| D9～D10 | uint16 LE | X 最大速度，steps/s，必须非 0 |
| D11～D12 | uint16 LE | Y 最大速度，steps/s，必须非 0 |
| D13～D14 | uint16 LE | Z 最大速度，steps/s，必须非 0 |
| D15 | uint8 | motionMode：`0=SEQUENTIAL`，`1=XYZ_SYNC` |

`motionMode=0`（以及旧 Android 未写入 D15 时的默认值）严格保持 X → Y → Z 串行执行。`motionMode=1` 只会让有位移的关节轴同时启动；MCU 以距离/最大速度最长的限制轴为时间基准，并降低较短轴速度，使其尽量同时到达。距离为 0 的轴不启动。该模式不是笛卡尔直线插补、DDA 或逆运动学，X/Y 为旋转关节时末端轨迹仍可能有弧度。

### 7.6 ARM_MOVE_TO_SAFE (`0x35`)

| DATA 字节 | 类型 | 定义 |
|---|---|---|
| D0～D2 | int24 LE | X 绝对目标坐标 |
| D3～D5 | int24 LE | Y 绝对目标坐标 |
| D6～D8 | int24 LE | Z 绝对目标坐标 |
| D9～D10 | uint16 LE | X 最大速度，steps/s，必须非 0 |
| D11～D12 | uint16 LE | Y 最大速度，steps/s，必须非 0 |
| D13～D14 | uint16 LE | Z 最大速度，steps/s，必须非 0 |
| D15 | uint8 | 仅接受 `0=SEQUENTIAL` |

要求三轴坐标有效且 SafeMove 配置已启用。当前 Z 数值越大位置越低：当 X/Y 需要变化且 `currentZ > SAFE_Z` 时，顺序为 Raise Z → X → Y → Final Z；否则跳过 Raise Z。各阶段串行，不做插补。`D15=1` 被拒绝，因为同步启动会绕过已验证的 SafeMove 防撞路径。

### 7.7 ARM_STOP (`0x36`)

| DATA 字节 | 定义 |
|---|---|
| D0～D15 | 保留，必须为 0 |

STOP 总是针对 MCU 当前 RobotArm 管理任务，不携带要停止任务的 SEQ。

即使没有活动任务，STOP 仍返回 ACCEPTED ACK，并把状态快照中的 `last_move_end_reason/error_code` 更新为 STOPPED，但不会生成没有原任务可关联的 EVENT。

### 7.8 ARM_CLEAR_ERROR (`0x37`)

| DATA 字节 | 定义 |
|---|---|
| D0～D15 | 保留，必须为 0 |

只清除管理层 ERROR 和状态机错误状态，不恢复 `position_valid`，也不伪造 `homed`。忙碌时返回 BUSY。

### 7.9 ARM_STATUS (`0x38`)

| DATA 字节 | 定义 |
|---|---|
| D0 | page：0、1、2 或 3 |
| D1～D15 | 保留，必须为 0 |

合法查询直接返回一帧 `ARM_STATUS_RSP(0x72)`，不发送 ACK。

## 8. ACK 格式 (`0x70`)

ACK 帧的 SEQ 与被确认请求一致。

| DATA 字节 | 定义 |
|---|---|
| D0 | 原请求 CMD |
| D1 | ACK 状态：0=ACCEPTED，1=REJECTED |
| D2 | result：RobotArmResult 或协议错误码 |
| D3～D15 | 保留，固定为 0 |

语义：

- `D1=ACCEPTED, D2=ROBOT_ARM_OK`：请求已被 MCU 接受。对异步命令，这不代表动作完成，Android 必须继续轮询 page3。
- `D1=REJECTED`：请求未启动，不会为该请求产生终态。
- 零位移命令也会先返回 ACCEPTED ACK，随后将 COMPLETED 终态保存到 page3。

## 9. EVENT 格式 (`0x71`)

`0x71` 的 DATA 语义用于 page3 中的终态字段。同一已接受任务只保存一个最终终态，MCU 不主动发送 EVENT。

| DATA 字节 | 定义 |
|---|---|
| D0 | 原异步请求 CMD |
| D1 | Event Type |
| D2 | RobotArmResult，表示最终结果或失败原因 |
| D3 | axis：单轴任务为 0/1/2；组合任务为 `0xFF` |
| D4～D15 | 保留，固定为 0 |

### 9.1 Event Type 完整数值表

| 值 | 名称 | 含义 |
|---:|---|---|
| 0 | COMPLETED | 非 Home 任务正常完成 |
| 1 | STOPPED | 被 STOP 终止 |
| 2 | LIMIT | 非 Home 任务因 Soft Limit，或已在 Home 零点又继续请求负方向而终止；首次命中 S1/S2/S3 且目标为 0 不产生该终态 |
| 3 | TIMEOUT | 非 Home 任务运动超时 |
| 4 | INTERLOCK | 非 Home 任务因安全互锁终止 |
| 5 | SENSOR_ERROR | 非 Home 任务因传感器条件错误终止 |
| 6 | DRIVER_ERROR | 非 Home 任务因底层完成条件或驱动错误终止 |
| 7 | HOME_COMPLETED | HomeAxis 或 HomeAll 正常完成 |
| 8 | HOME_FAILED | HomeAxis 或 HomeAll 失败；具体原因读取 D2 |

Home 命令失败时统一使用 `HOME_FAILED`，例如 Home 超时为 `D1=HOME_FAILED, D2=ROBOT_ARM_ERR_HOME_TIMEOUT`，而不是 `D1=TIMEOUT`。

## 10. STATUS_RSP (`0x72`)

STATUS_RSP 的 SEQ 与 STATUS 请求一致。

### 10.1 page 0：状态与标志

| DATA 字节 | 定义 |
|---|---|
| D0 | page，固定 0 |
| D1 | arm_state |
| D2 | operation |
| D3 | error_code 低 8 位；当前公开结果码均可完整表示 |
| D4 | X axis_state |
| D5 | Y axis_state |
| D6 | Z axis_state |
| D7 | homed_flags |
| D8 | position_valid_flags |
| D9 | sensor_flags |
| D10 | last_move_end_reason |
| D11 | home_state |
| D12 | move_to_state |
| D13 | safe_move_state |
| D14～D15 | 保留，固定为 0 |

#### homed_flags（D7）

| bit | 掩码 | 定义 |
|---:|---:|---|
| 0 | `0x01` | X homed |
| 1 | `0x02` | Y homed |
| 2 | `0x04` | Z homed |
| 3～7 | - | 保留 |

#### position_valid_flags（D8）

| bit | 掩码 | 定义 |
|---:|---:|---|
| 0 | `0x01` | X current 坐标可信 |
| 1 | `0x02` | Y current 坐标可信 |
| 2 | `0x04` | Z current 坐标可信 |
| 3～7 | - | 保留 |

#### sensor_flags（D9）

上报值已经统一为 `released=0, triggered=1` 的逻辑语义；Android 不应再次取反。

| bit | 掩码 | 定义 |
|---:|---:|---|
| 0 | `0x01` | S1 = X_HOME |
| 1 | `0x02` | S2 = Y_HOME |
| 2 | `0x04` | S3 = Z_HOME / Z upper limit |
| 3～7 | - | 保留 |

### 10.2 page 1：当前坐标

| DATA 字节 | 类型 | 定义 |
|---|---|---|
| D0～D3 | int32 LE | current X |
| D4～D7 | int32 LE | current Y |
| D8～D11 | int32 LE | current Z |
| D12 | uint8 | page 标识，固定 1 |
| D13～D15 | - | 保留，固定为 0 |

只有 page 0 对应轴的 `position_valid` 为 1 时，Android 才能把 current 坐标用于绝对运动和位置显示判断。数值存在不代表数值可信。

### 10.3 page 2：目标坐标

| DATA 字节 | 类型 | 定义 |
|---|---|---|
| D0～D3 | int32 LE | target X |
| D4～D7 | int32 LE | target Y |
| D8～D11 | int32 LE | target Z |
| D12 | uint8 | page 标识，固定 2 |
| D13～D15 | - | 保留，固定为 0 |

对 MoveToSafe，page 2 返回最终 XYZ 目标，而不是当前正在执行的中间 Safe Z 目标。

### 10.4 page 3：最近一次异步终态

page3 的 `STATUS_RSP` SEQ 属于本次 `ARM_STATUS` 查询；Android 必须使用 D2～D4 的原请求 CMD/SEQ 匹配已 ACK 的异步动作。读取不会清除终态，MCU 也不会主动发送 `0x71`。

| DATA 字节 | 定义 |
|---|---|
| D0 | page 标识，固定 3 |
| D1 | terminal_valid：0=尚无终态，1=存在终态 |
| D2 | 原异步请求 CMD，例如单轴 Home 为 `0x31` |
| D3～D4 | 原异步请求 SEQ，uint16 LE |
| D5 | 固定 `0x71`，表示 D6～D7 使用 EVENT 终态语义 |
| D6 | Event Type |
| D7 | RobotArmResult，表示最终结果或失败原因 |
| D8～D15 | 保留，固定为 0 |

## 11. 状态枚举

### 11.1 arm_state

| 值 | 名称 |
|---:|---|
| 0 | IDLE |
| 1 | MOVING |
| 2 | HOMING |
| 3 | ERROR |

### 11.2 operation

| 值 | 名称 |
|---:|---|
| 0 | NONE |
| 1 | MOVE_X |
| 2 | MOVE_Y |
| 3 | MOVE_Z |
| 4 | MOVE_TO |
| 5 | MOVE_TO_SAFE |
| 6 | HOME_X |
| 7 | HOME_Y |
| 8 | HOME_Z |
| 9 | HOME_ALL |

### 11.3 axis_state

| 值 | 名称 |
|---:|---|
| 0 | IDLE |
| 1 | MOVING |
| 2 | HOMING |
| 3 | ERROR |

### 11.4 home_state

| 值 | 名称 |
|---:|---|
| 0 | IDLE |
| 1 | CHECK_SENSOR |
| 2 | BACKOFF_IF_ACTIVE |
| 3 | SEEK_FAST |
| 4 | BACKOFF_AFTER_TRIGGER |
| 5 | SEEK_SLOW |
| 6 | DONE |
| 7 | ERROR |

### 11.5 move_to_state

| 值 | 名称 |
|---:|---|
| 0 | IDLE |
| 1 | X_START |
| 2 | X_WAIT |
| 3 | Y_START |
| 4 | Y_WAIT |
| 5 | Z_START |
| 6 | Z_WAIT |
| 7 | DONE |
| 8 | ERROR |

### 11.6 safe_move_state

| 值 | 名称 |
|---:|---|
| 0 | IDLE |
| 1 | PREPARE |
| 2 | RAISE_Z_START |
| 3 | RAISE_Z_WAIT |
| 4 | X_START |
| 5 | X_WAIT |
| 6 | Y_START |
| 7 | Y_WAIT |
| 8 | FINAL_Z_START |
| 9 | FINAL_Z_WAIT |
| 10 | DONE |
| 11 | ERROR |

### 11.7 last_move_end_reason

| 值 | 名称 |
|---:|---|
| 0 | NONE |
| 1 | COMPLETED |
| 2 | STOPPED |
| 3 | SENSOR |
| 4 | LIMIT |
| 5 | INTERLOCK |
| 6 | TIMEOUT |
| 7 | DRIVER_ERROR |

## 12. RobotArmResult 完整数值表

| 值 | 名称 | Android 处理建议 |
|---:|---|---|
| 0 | OK | 请求被接受或任务成功 |
| 1 | BUSY | MCU 有未结束任务或清错时仍在运动；等待当前 EVENT/状态结束后再由用户决定是否重试 |
| 2 | POSITION_UNKNOWN | 至少一个所需坐标不可信；禁止运动并引导用户完成 Home，不得在 Android 侧伪造有效坐标 |
| 3 | NOT_HOMED | 需要 Home 条件但尚未建立原点；当前协议保留该结果值 |
| 4 | LIMIT | 目标越过启用的 Soft Limit，或轴已在对应 S1～S3 Home 零点且又请求继续负方向；拒绝时不产生 STEP。负向运行首次命中 Home 且目标为 0 时是正常归零，不返回 LIMIT |
| 5 | INTERLOCK | SafeMove 转换不满足安全高度或安全策略；不得绕过后继续下一轴 |
| 6 | DRIVER | 底层驱动拒绝启动，或 Busy 结束但完成步数证据不完整 |
| 7 | SENSOR | 传感器快照未就绪，或 Home 退让后传感器未可靠释放 |
| 8 | HOME_TIMEOUT | Home 总时间或寻边步数耗尽；Home 失败，相关轴 homed/position_valid 清零 |
| 9 | MOVE_TIMEOUT | 普通运动超过计算超时；活动轴 position_valid 清零 |
| 10 | STOPPED | 任务被 STOP 终止；正在运动的轴 position_valid 清零 |
| 11 | NOT_SUPPORTED | 功能或轴不受支持；当前公开 V2 请求通常会在协议层先返回更明确错误 |
| 12 | CONFIG | Home/Soft Limit/SafeMove 所需生产参数未启用或无效；不得由 Android 猜测参数绕过 |

协议层错误码也放在 ACK 的 D2，但不属于 RobotArmResult：

| 值 | 名称 |
|---:|---|
| `0x80` | BAD_AXIS |
| `0x81` | BAD_CMD |
| `0x82` | BAD_PAGE |

### 12.1 Android 对关键结果的处理边界

- BUSY：本次新请求没有启动。等待当前异步任务的最终 EVENT，或通过 STATUS 确认空闲后再让用户决定是否重试。
- POSITION_UNKNOWN：坐标不可作为运动依据。停止发送绝对、相对和组合运动，引导现场完成 Home；禁止在 Android 内把 valid 强制改为 1。
- CONFIG：所需 MCU 生产参数仍禁用或无效。不得通过换用旧直驱命令绕过；应由固件/机械标定流程解决。
- LIMIT：若出现在 REJECTED ACK 中，本次请求没有产生 STEP；若出现在 EVENT 中，表示运行中遇到限位，活动轴坐标已经不可信。
- TIMEOUT：Event Type 为 TIMEOUT 时，D2 进一步区分 HOME_TIMEOUT 或 MOVE_TIMEOUT。相关轴需要处理坐标失效，不能直接续跑。
- INTERLOCK：安全转换条件不成立。Android 必须终止当前动作链，不得自行跳过 Raise Z 或启动后续轴。

Android 必须联合判断 ACK 状态、Event Type 和 result，不能只凭其中一个字段推断完整结果。

## 13. 完整交互时序

### 13.1 异步命令

适用于 `0x30..0x35`：

```text
Android                         MCU
   | ---- Request, SEQ=N ------> |
   | <--- ACK ACCEPTED, SEQ=N -- |
   |       动作异步执行           |
   | <--- EVENT final, SEQ=N ---- |
```

- ACK ACCEPTED 只表示启动请求已接受。
- 最终完成、停止或失败只以 EVENT 为准。
- ACK REJECTED 后不会再有该请求的 EVENT。
- 同一任务的终态 EVENT 只生成一次。

### 13.2 STATUS 查询

```text
Android                         MCU
   | ---- STATUS, SEQ=N -------> |
   | <- STATUS_RSP, SEQ=N ------ |
```

STATUS 不发送 ACK。Android 应分别查询 page 0、1、2，并使用各自 SEQ 匹配。

### 13.3 STOP 与原任务 SEQ

例如 MoveTo 的 SEQ=105，STOP 的 SEQ=106：

```text
Android                              MCU
   | ---- MOVE_TO, SEQ=105 ---------> |
   | <--- ACK ACCEPTED, SEQ=105 ----- |
   | ---- STOP, SEQ=106 ------------> |
   | <--- ACK ACCEPTED, SEQ=106 ----- |  STOP 请求自己的确认
   | <--- EVENT STOPPED, SEQ=105 ---- |  原 MoveTo 的最终事件
```

- STOP ACK 必须匹配 STOP 请求的 SEQ。
- STOPPED EVENT 必须匹配被中止任务的原始 SEQ。
- MCU 不会用 STOP 的 SEQ 伪装原任务 EVENT。
- 如果没有活动任务，STOP 仍返回 ACCEPTED ACK，但不会凭空生成 STOPPED EVENT。
- 如果原任务终态已经产生，STOP 不会再制造第二个冲突终态。

## 14. 非法帧与非法请求行为

| 情况 | MCU 行为 |
|---|---|
| CRC 错误 | 丢弃整帧，不发送 ACK/EVENT/STATUS |
| 帧头、V2 标记、固定尾标记或长度不合法 | 不进入命令层，不发送响应；解析器尝试从后续 `0xAA` 重新同步 |
| 非法 CMD | 无活动任务时返回 REJECTED ACK，result=`0x81 BAD_CMD` |
| 非法 axis | 无活动任务时返回 REJECTED ACK，result=`0x80 BAD_AXIS`，不产生 STEP |
| 非法 STATUS page | 返回 REJECTED ACK，result=`0x82 BAD_PAGE` |
| 活动异步任务尚未完整结束时收到新的动作或未知 CMD | 返回 REJECTED ACK，result=`ROBOT_ARM_ERR_BUSY`，保护原任务 SEQ/EVENT |

因为 CRC 错误帧中的 CMD/SEQ 不可信，Android 不应期待 MCU 对 CRC 错误进行否定应答。Android 自身也必须先验证 MCU 响应 CRC，再更新任务状态。

## 15. V1/V2 共存

- V1 与 V2 共用 USART1 RS485、9600 baud 和同一字节流入口。
- 收到 `0xAA` 后，第二字节为 `0xFE` 时按 V2 固定 24B 解析；否则按既有 V1 固定 10B 解析。
- V1 格式保持为 `AA CMD D0..D5 55 SUM`，校验为偏移 0～7 的 8 位累加和。
- V1 帧继续进入既有 V1 命令处理；V2 RobotArm 帧进入独立 V2 命令层。
- V1 与 V2 各有独立的 4 帧接收队列；队列已满时丢弃最新到达的完整帧，因此该帧不会产生响应。Android 应等待响应、控制发送节奏，不能无间隔无限突发。
- Android 不得在一帧尚未发送完整时穿插另一协议的字节。
- V1 的 CMD=`0xFE` 会与 V2 标记冲突，不应使用。
- 旧 V1 步进直驱命令会绕过 RobotArm 坐标管理；执行后 MCU 会使对应轴 `position_valid` 失效。Android 新功能不得把这些旧命令与 V2 绝对运动混用。
