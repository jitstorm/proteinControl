#ifndef __ROBOT_ARM_CONFIG_H__
#define __ROBOT_ARM_CONFIG_H__

#include <stdint.h>

/* 临时调试时假定三轴已完成 Home；关闭后恢复上电必须先正式 Home 的流程。 */
#ifndef ROBOT_ARM_DEBUG_ASSUME_HOME
#if defined(ROBOT_ARM_LOGIC_TEST)
/* 逻辑回归必须从未建立坐标的上电状态开始，不能继承现场调试的假定 Home。 */
#define ROBOT_ARM_DEBUG_ASSUME_HOME 0u
#else
/* 生产上电后必须等待 S1/S2/S3 实际触发后才建立零点，不能仅因软件坐标初值为 0
 * 就允许普通 MOVE 或 Phase 误认为机构已经回零。 */
#define ROBOT_ARM_DEBUG_ASSUME_HOME 1u
#endif
#endif

/* HC165 采集层已经按位取反；以下逻辑电平仍需通过板端传感器测试确认。 */
#define ROBOT_ARM_S1_TRIGGERED_LEVEL 1u
#define ROBOT_ARM_S2_TRIGGERED_LEVEL 1u
#define ROBOT_ARM_S3_TRIGGERED_LEVEL 1u

/*
 * S4 已取消，X+/Y+/Z+ 只能依赖软件行程保护。
 * 下列值必须由实际机械标定填写，单位为步数；未知时保持 0 会安全地拒绝一切
 * 正方向目标，绝不能猜测生产坐标或回退到已取消的 S4 硬限位。
 */
#if defined(ROBOT_ARM_POST_HOME_TEST) || defined(ROBOT_ARM_HOME_SLOW_APPROACH_TEST)
/* 仅扩大补充找零回归的模拟行程，验证 20000 步下发；生产标定不变。 */
#define ROBOT_ARM_X_MAX_TRAVEL 30000
#define ROBOT_ARM_Y_MAX_TRAVEL 30000
#define ROBOT_ARM_Z_MAX_TRAVEL 30000
#elif defined(ROBOT_ARM_LOGIC_TEST) && !defined(ROBOT_ARM_PHASE_FREQUENCY_TEST)
#define ROBOT_ARM_X_MAX_TRAVEL 1000
#define ROBOT_ARM_Y_MAX_TRAVEL 1000
#define ROBOT_ARM_Z_MAX_TRAVEL 1000
#else
#define ROBOT_ARM_X_MAX_TRAVEL 1000000
#define ROBOT_ARM_Y_MAX_TRAVEL 1000000
#define ROBOT_ARM_Z_MAX_TRAVEL 1000000
#endif
#define ROBOT_ARM_X_MIN_POSITION 0
#define ROBOT_ARM_X_MAX_POSITION ROBOT_ARM_X_MAX_TRAVEL
#define ROBOT_ARM_Y_MIN_POSITION 0
#define ROBOT_ARM_Y_MAX_POSITION ROBOT_ARM_Y_MAX_TRAVEL
#define ROBOT_ARM_Z_MIN_POSITION 0
#define ROBOT_ARM_Z_MAX_POSITION ROBOT_ARM_Z_MAX_TRAVEL

/* 三轴 Home 均朝坐标负方向；Z- 对应向上运动。 */
#define ROBOT_ARM_X_HOME_DIRECTION (-1)
#define ROBOT_ARM_Y_HOME_DIRECTION (-1)
#define ROBOT_ARM_Z_HOME_DIRECTION (-1)

/* 逻辑测试使用短行程；固件默认仍保持禁用，防止未知距离的自动运动。 */
#if (defined(ROBOT_ARM_LOGIC_TEST) && !defined(ROBOT_ARM_HOME_CONFIG_REJECT_TEST)) || defined(ROBOT_ARM_HOME_SLOW_APPROACH_TEST)
#define ROBOT_ARM_X_HOME_ENABLED 1u
#define ROBOT_ARM_Y_HOME_ENABLED 1u
#define ROBOT_ARM_Z_HOME_ENABLED 1u
#define ROBOT_ARM_X_HOME_MAX_STEPS 100u
#define ROBOT_ARM_Y_HOME_MAX_STEPS 100u
#define ROBOT_ARM_Z_HOME_MAX_STEPS 100u
#define ROBOT_ARM_X_HOME_TIMEOUT_MS 1000u
#define ROBOT_ARM_Y_HOME_TIMEOUT_MS 1000u
#define ROBOT_ARM_Z_HOME_TIMEOUT_MS 1000u
/* HomeAll 依次 Z→Y→X；每一轴必须使用自身标定的快速寻零速度，单位 steps/s。 */
#if defined(ROBOT_ARM_HOME_REQUEST_SPEED_TEST)
/* 仅用于验证 0x31 必须使用请求速度，不能错误依赖 HomeAll 的默认快速速度。 */
#define ROBOT_ARM_HOME_FAST_SPEED_X 0u
#define ROBOT_ARM_HOME_FAST_SPEED_Y 0u
#define ROBOT_ARM_HOME_FAST_SPEED_Z 0u
#else
#define ROBOT_ARM_HOME_FAST_SPEED_X 100u
#define ROBOT_ARM_HOME_FAST_SPEED_Y 100u
#define ROBOT_ARM_HOME_FAST_SPEED_Z 100u
#endif
/* 旧双阶段退让/复找配置仍未启用；0x31 的末段低速由下方每轴独立宏控制。 */
#define ROBOT_ARM_HOME_SLOW_SPEED 20u
#define ROBOT_ARM_HOME_BACKOFF_STEPS 10u
#else
#define ROBOT_ARM_X_HOME_ENABLED 1u
#define ROBOT_ARM_Y_HOME_ENABLED 1u
#define ROBOT_ARM_Z_HOME_ENABLED 1u
#define ROBOT_ARM_X_HOME_MAX_STEPS 1000000u
#define ROBOT_ARM_Y_HOME_MAX_STEPS 1000000u
#define ROBOT_ARM_Z_HOME_MAX_STEPS 20000u
#define ROBOT_ARM_X_HOME_TIMEOUT_MS 60000u
#define ROBOT_ARM_Y_HOME_TIMEOUT_MS 60000u
#define ROBOT_ARM_Z_HOME_TIMEOUT_MS 60000u
#define ROBOT_ARM_HOME_FAST_SPEED_X 0u
#define ROBOT_ARM_HOME_FAST_SPEED_Y 0u
#define ROBOT_ARM_HOME_FAST_SPEED_Z 0u
#define ROBOT_ARM_HOME_SLOW_SPEED 0u
#define ROBOT_ARM_HOME_BACKOFF_STEPS 0u
#endif
/* S1/S2/S3 Active 的产品定义就是坐标 0；保留旧宏仅兼容外部配置，RobotArm
 * 不再读取它们，也不得通过偏移把已触发 Home 的轴报告为非零坐标。 */
#define ROBOT_ARM_X_HOME_OFFSET 0
#define ROBOT_ARM_Y_HOME_OFFSET 0
#define ROBOT_ARM_Z_HOME_OFFSET 0

/* Safe Z 尚未完成机械标定，生产固件必须保持禁用并拒绝安全移动。 */
#ifndef ROBOT_ARM_SAFE_MOVE_ENABLED
#define ROBOT_ARM_SAFE_MOVE_ENABLED 0u
#endif
#ifndef ROBOT_ARM_SAFE_Z_POSITION
#define ROBOT_ARM_SAFE_Z_POSITION 0
#endif

/* 适配现有 DMA 驱动的默认速度，单位均为 steps/s。 */
#define ROBOT_ARM_X_DEFAULT_SPEED 1000u
#define ROBOT_ARM_Y_DEFAULT_SPEED 1000u
#define ROBOT_ARM_Z_DEFAULT_SPEED 1000u

/*
 * Phase 协议的 xF0/xF1/yF0/yF1=0 是“从静止起步/最终停下”的边界语义，并非可写入 ARR 的
 * 真实频率。执行 XY Phase 时分别把 X/Y 的 0 映射到该安全边界频率；不能让 DMA
 * 直接收到 0，也不能再按两轴距离缩放频率。
 * 普通 MOVE 与 XYZ_SYNC 仍通过驱动层原有的 500 steps/s 起步策略，不读取此宏。
 */
#define ROBOT_ARM_PHASE_BOUNDARY_FREQUENCY 500u

/* 三轴 DMA 步进的第一版加速度，单位为 steps/s^2。
 * X 保持现有偏柔和参数；Y/Z 降低原有冲击，并由各自梯形 profile 使用。 */
#define ROBOT_ARM_X_ACCELERATION 50000u
#define ROBOT_ARM_Y_ACCELERATION 50000u
#define ROBOT_ARM_Z_ACCELERATION 2000u

/* 单轴 Home 的加速度默认沿用对应轴普通运动的安全标定值。0x31 可按次覆盖，
 * 但传入 0 必须明确回退此处默认值，不能把 0 直接交给 DMA。 */
#define ROBOT_ARM_X_HOME_DEFAULT_ACCELERATION ROBOT_ARM_X_ACCELERATION
#define ROBOT_ARM_Y_HOME_DEFAULT_ACCELERATION ROBOT_ARM_Y_ACCELERATION
#define ROBOT_ARM_Z_HOME_DEFAULT_ACCELERATION ROBOT_ARM_Z_ACCELERATION

/* 单轴 Home 已知软件坐标时的末段低速配置。距离理论零点仍有此范围时应已降至
 * 对应速度；S1/S2/S3 才是唯一的真实完成依据，不能用这些步数直接置零。 */
#define ROBOT_ARM_X_HOME_SLOW_ZONE_STEPS 1500u
#define ROBOT_ARM_Y_HOME_SLOW_ZONE_STEPS 1500u
#define ROBOT_ARM_Z_HOME_SLOW_ZONE_STEPS 1000u
#define ROBOT_ARM_X_HOME_SLOW_SPEED 7000u
#define ROBOT_ARM_Y_HOME_SLOW_SPEED 7000u
#define ROBOT_ARM_Z_HOME_SLOW_SPEED 2000u
/* 已走到理论零点仍未命中传感器时的低速额外搜索上限，单位为 STEP。 */
#define ROBOT_ARM_HOME_EXTRA_SEARCH_STEPS 20000u

/* Android 的 0x34 速度字段是 uint16；PU1/PB10、PU2/PB11、PU3/PB13 的最终上限
 * 统一限制为协议可表达的 65535 steps/s，仍由各底层 DMA 驱动执行既有加减速控制。 */
#define ROBOT_ARM_X_MAX_SPEED 65535u
#define ROBOT_ARM_Y_MAX_SPEED 65535u
#define ROBOT_ARM_Z_MAX_SPEED 65535u

/* 单轴超时按请求速度估算并保留四倍裕量，避免正常加减速被误判。 */
#define ROBOT_ARM_MOVE_TIMEOUT_MIN_MS 30000u
#define ROBOT_ARM_MOVE_TIMEOUT_MARGIN 4u

#endif
