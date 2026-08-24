#include <stdint.h>
#include "single_motor.h"
#include "shift_register.h"
#include "shift_register_input.h"

#define TEST_CHECK(condition) do { if (!(condition) && (s_failure == 0)) s_failure = __LINE__; } while (0)

static int s_failure;
uint8_t HC595Data[SHIFT_REGISTER_COUNT];
uint8_t inputData[SHIFT_REGISTER_INPUT_COUNT];

/** 模拟 74HC595 提交；逻辑测试只验证状态机是否正确请求关闭目标 MT。 */
void ShiftRegister_WriteAll(uint8_t *data)
{
    (void)data;
}

/**
 * 写入单向电机状态机使用的 74HC165 输入快照。
 *
 * @param sensor_id 项目既有的 1 起始传感器编号。
 * @param level 要模拟的原始高低电平。
 */
static void TestSetSensorLevel(uint8_t sensor_id, uint8_t level)
{
    uint8_t sensor_index = (uint8_t)((sensor_id - 1u) / 8u);
    uint8_t sensor_mask = (uint8_t)(1u << ((sensor_id - 1u) % 8u));

    sensor_index = (uint8_t)((SHIFT_REGISTER_INPUT_COUNT - 1u) - sensor_index);
    if (level)
    {
        inputData[sensor_index] |= sensor_mask;
    }
    else
    {
        inputData[sensor_index] &= (uint8_t)~sensor_mask;
    }
}

/**
 * 验证已 armed 的传感器在连续稳定 30ms 后停止目标 MT。
 *
 * @param trigger_level 本轮应触发停止的电平。
 * @param initial_level 命令启动时的传感器电平。
 */
static void TestExpectNewTriggerStops(uint8_t trigger_level, uint8_t initial_level)
{
    const uint8_t motor_id = 1u;
    const uint8_t sensor_id = 1u;

    SingleMotor_Init();
    TestSetSensorLevel(sensor_id, initial_level);
    TEST_CHECK(SingleMotor_StartSensorTimed(motor_id, sensor_id, trigger_level, 500u) == 1u);
    TEST_CHECK(g_single_motors[motor_id - 1u].running == 1u);

    if (initial_level == trigger_level)
    {
        /* 启动时的旧触发不能停机；先释放后才重新 armed。 */
        SingleMotor_Task(30u);
        TEST_CHECK(g_single_motors[motor_id - 1u].running == 1u);
        TestSetSensorLevel(sensor_id, (uint8_t)!trigger_level);
        SingleMotor_Task(1u);
        TEST_CHECK(g_single_motors[motor_id - 1u].state == MOTOR_SENSOR_ARMED);
        TEST_CHECK(g_single_motors[motor_id - 1u].running == 1u);
    }

    TestSetSensorLevel(sensor_id, trigger_level);
    SingleMotor_Task(30u);
    TEST_CHECK(g_single_motors[motor_id - 1u].running == 0u);
    TEST_CHECK(g_single_motors[motor_id - 1u].state == MOTOR_SENSOR_TRIGGERED);
}

/** 运行 0x07 单向电机新触发、超时、手动停止及 0x08 位图覆盖测试。 */
int main(void)
{
    uint8_t bitmap[3];
    const uint8_t motor_id = 1u;
    const uint8_t sensor_id = 1u;

    /* 1、2：LOW 触发，分别覆盖启动未触发和启动已触发后等待释放。 */
    TestExpectNewTriggerStops(0u, 1u);
    TestExpectNewTriggerStops(0u, 0u);

    /* 3、4：HIGH 触发，分别覆盖启动未触发和启动已触发后等待释放。 */
    TestExpectNewTriggerStops(1u, 0u);
    TestExpectNewTriggerStops(1u, 1u);

    /* 5：始终没有新的触发时，最大运行时间到达必须停止。 */
    SingleMotor_Init();
    TestSetSensorLevel(sensor_id, 0u);
    TEST_CHECK(SingleMotor_StartSensorTimed(motor_id, sensor_id, 1u, 100u) == 1u);
    SingleMotor_Task(100u);
    TEST_CHECK(g_single_motors[motor_id - 1u].running == 0u);
    TEST_CHECK(g_single_motors[motor_id - 1u].state == MOTOR_TIMEOUT);

    /* 6、7：0x05 的停止入口必须清理等待状态，随后 0x08 位图显示停止。 */
    SingleMotor_Init();
    TestSetSensorLevel(sensor_id, 0u);
    TEST_CHECK(SingleMotor_StartSensorTimed(motor_id, sensor_id, 1u, 500u) == 1u);
    TEST_CHECK(SingleMotor_Immediate(motor_id, 0u) == 1u);
    TEST_CHECK(g_single_motors[motor_id - 1u].running == 0u);
    TEST_CHECK(g_single_motors[motor_id - 1u].sensor_id == 0u);
    SingleMotor_GetRunningBitmap(bitmap);
    TEST_CHECK((bitmap[0] & 0x01u) == 0u);

    return s_failure;
}
