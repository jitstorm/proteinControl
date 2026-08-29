#ifndef __ROBOT_ARM_CONFIG_H__
#define __ROBOT_ARM_CONFIG_H__

#include <stdint.h>

/* 临时调试时假定三轴已完成 Home；关闭后恢复上电必须先正式 Home 的流程。 */
#ifndef ROBOT_ARM_DEBUG_ASSUME_HOME
#define ROBOT_ARM_DEBUG_ASSUME_HOME 1u
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
#if defined(ROBOT_ARM_LOGIC_TEST)
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
#if defined(ROBOT_ARM_LOGIC_TEST) && !defined(ROBOT_ARM_HOME_CONFIG_REJECT_TEST)
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
/* 当前生产 Home 仅执行一次快速寻零；保留以下旧配置以便未来恢复双阶段流程，
 * 反向脱离和二次慢速寻零当前不得读取或使用。 */
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

/* Android 的 0x34 速度字段是 uint16；PU1/PB10、PU2/PB11、PU3/PB13 的最终上限
 * 统一限制为协议可表达的 65535 steps/s，仍由各底层 DMA 驱动执行既有加减速控制。 */
#define ROBOT_ARM_X_MAX_SPEED 65535u
#define ROBOT_ARM_Y_MAX_SPEED 65535u
#define ROBOT_ARM_Z_MAX_SPEED 65535u

/* 单轴超时按请求速度估算并保留四倍裕量，避免正常加减速被误判。 */
#define ROBOT_ARM_MOVE_TIMEOUT_MIN_MS 30000u
#define ROBOT_ARM_MOVE_TIMEOUT_MARGIN 4u

#endif
