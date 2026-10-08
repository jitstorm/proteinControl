#include <stdint.h>

/** 记录单元测试失败次数，非零表示至少一个速度规划边界不符合预期。 */
static int s_failure;

/** 当条件不成立时记录失败，不中断后续边界场景。 */
static void TestCheck(int condition)
{
    if (!condition)
    {
        s_failure++;
    }
}

/**
 * 按 0x39 的短距离优先段尾速度规则计算恒定加速度。
 *
 * 该测试用独立整数公式覆盖正常加速、短距离加速、短距离减速和零时间匀速语义，
 * 防止后续调整定时器代码时丢失“距离不足则压缩时间”的协议约束。
 */
static uint32_t TestResolveAcceleration(uint32_t steps, uint32_t start,
                                        uint32_t terminal, uint32_t time_ms)
{
    uint32_t delta;
    uint32_t time_acceleration;
    uint64_t required_steps;
    uint64_t delta_squared;

    if ((time_ms == 0u) || (start == terminal))
    {
        return 1u;
    }
    delta = (terminal >= start) ? (terminal - start) : (start - terminal);
    required_steps = (((uint64_t)start + (uint64_t)terminal) * time_ms +
                      1999ull) / 2000ull;
    time_acceleration = ((uint64_t)delta * 1000ull + time_ms - 1u) / time_ms;
    if ((uint64_t)steps >= required_steps)
    {
        return time_acceleration;
    }
    delta_squared = ((uint64_t)terminal * terminal >= (uint64_t)start * start) ?
                    ((uint64_t)terminal * terminal - (uint64_t)start * start) :
                    ((uint64_t)start * start - (uint64_t)terminal * terminal);
    return (uint32_t)((delta_squared + 2ull * steps - 1ull) /
                      (2ull * steps));
}

/** 执行 0x39 加速时间与短距离段尾速度的基础数学回归检查。 */
int main(void)
{
    /* 4750 步正好允许 500 steps/s 在 1000ms 内变化到 9000 steps/s。 */
    TestCheck(TestResolveAcceleration(4750u, 500u, 9000u, 1000u) == 8500u);

    /* 2000 步不足时必须提高加速度，保证理论段尾到达 9000 steps/s。 */
    TestCheck(TestResolveAcceleration(2000u, 500u, 9000u, 1000u) == 20188u);

    /* 降速使用相同距离公式，不能仍按原 1000ms 的 8500 steps/s^2 运行。 */
    TestCheck(TestResolveAcceleration(2000u, 9000u, 500u, 1000u) == 20188u);

    /* 0ms 是首脉冲直接使用终止速度的匀速模式，不允许参与时间除法。 */
    TestCheck(TestResolveAcceleration(1u, 500u, 9000u, 0u) == 1u);
    return s_failure;
}
