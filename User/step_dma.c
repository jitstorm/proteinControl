#include "step_dma.h"
#include "robot_arm_driver.h"
#include "robot_arm_config.h"
#include "robot_arm_protocol.h"
#include "shift_register.h"
#include "DELAY.h"

#ifndef DMA2_Channel2_IRQn
#define DMA2_Channel2_IRQn 57
#endif

#ifndef DMA2_Channel3_IRQn
#define DMA2_Channel3_IRQn 58
#endif

/* =========================
 *  硬件绑定：实际 PU2 的 PB11 输出 STEP
 * ========================= */
#define STEP_GPIO GPIOB
#define STEP_PIN 11
#define STEP_MASK (1u << STEP_PIN)

/* =========================
 *  DMA 分段输出策略（核心）
 * =========================
 * 我们用 TIM5 的 Update 事件做“节拍器”，每次 Update 触发 DMA 写一次 GPIOB->BSRR。
 *
 * 由于一个 STEP 脉冲通常需要：高 -> 低（至少满足脉宽）
 * 这里我们把“高/低”两次写 BSRR 当成两个边沿事件 edges：
 *   1 step = 2 edges
 *
 * 为了按步数精确停止，DMA 必须用 Normal 模式：
 *   - 每次装载 N 个 edges，DMA 搬完触发 TC 中断
 *   - 在 TC 中断里继续装载下一段（chunk）
 *
 * 这样 CPU 负担很轻：每 CHUNK_STEPS 步才进一次中断。
 */
#define CHUNK_STEPS 32u
#define CHUNK_EDGES (CHUNK_STEPS * 2u) /* 每一步输出高、低两个 BSRR 边沿。 */

/* Phase 的协议 0Hz 必须先在 RobotArm 的本轴边界映射完成；此处仅防止错误直调产生 0 ARR。 */
#define PHASE_TIMER_MIN_FREQUENCY 1u

/* TIM5 计数时钟（一般 72MHz） */
static uint32_t s_tim5_clk = 72000000u;
static volatile uint32_t s_cur_f = 0; // 当前运行速度 steps/s（用于平滑切换）

/* 运行状态 */
static volatile uint8_t s_running = 0;
typedef struct
{
    uint8_t valid;
    uint32_t steps;
    uint32_t f_start;
    uint32_t f_max;
    uint32_t accel;
} PendingCmd;

static volatile PendingCmd s_pending = {0, 0, 0, 0, 0};

/* DMA chunk buffer：交替 set/reset，长度 CHUNK_EDGES */
static uint32_t s_dma_buf[CHUNK_EDGES];

/* 运行模式：常速 or 梯形 */
typedef enum
{
    MODE_IDLE = 0,
    MODE_CONST,
    MODE_TRAP
} StepMode;

static volatile StepMode s_mode = MODE_IDLE;

/* =========================
 *  常速模式的剩余边沿计数
 * ========================= */
static volatile uint32_t s_edges_left = 0; /* 还剩多少 edges 没输出 */

/* =========================
 *  梯形/三角形模式参数
 * =========================
 * 我们做“末端减到 0”的速度规划。
 *
 * 速度规划用经典匀加速公式（离散步进用 steps 当作位移）：
 *   v^2 = v0^2 + 2*a*s
 * 其中：
 *   v  : 当前速度（steps/s）
 *   v0 : 起始速度（steps/s）
 *   a  : 加速度（steps/s^2）
 *   s  : 已走步数（steps）
 *
 * 末端减到 0 时，减速段可用：
 *   v^2 = 2*a*s_remain        (因为末速度 v_end=0)
 *   v = sqrt(2*a*s_remain)
 *
 * 我们不输出 0Hz（定时器无法设置为 0），最后一步跑完直接 stop()，实现“到 0 停止”。
 */
typedef struct
{
    uint32_t steps_total; /* 总步数 */
    uint32_t steps_done;  /* 已完成步数（按 chunk 累加） */

    uint32_t f_start; /* 起步速度 steps/s（>=1） */
    uint32_t f_max;   /* 速度上限 steps/s */
    uint32_t accel;   /* 加速度 steps/s^2（>=1） */

    uint32_t v_peak;      /* 实际峰值速度（可能 < f_max，三角形时） */
    uint32_t accel_steps; /* 加速段步数：从 f_start 加速到 v_peak */
    uint32_t decel_steps; /* 减速段步数：从 v_peak 减速到 0 */
    uint32_t f_end;
    uint8_t phase_profile;
    uint8_t home_approach;
    uint32_t slow_zone_steps;
} TrapProfile;

/* 主循环通过只读接口审计完成原因，ISR 更新字段必须保持可见。 */
static volatile TrapProfile s_trap;

void stepdma_pb11_request_trap(uint32_t steps,
                               uint32_t f_start,
                               uint32_t f_max,
                               uint32_t accel)
{
    s_pending.steps = steps;
    s_pending.f_start = f_start;
    s_pending.f_max = f_max;
    s_pending.accel = accel;
    s_pending.valid = 1;

    /* 如果当前不在运行，直接启动
       如果在运行，则等 TC 中断安全切换 */
    if (!s_running)
    {
        s_pending.valid = 0;
        stepdma_pb11_move_trap(steps, f_start, f_max, accel);
    }
}

/* =========================
 *  工具：填充 DMA buffer（交替 set/reset）
 * ========================= */
static void fill_dma_buf(void)
{
    uint32_t i;
    uint32_t setw;
    uint32_t rstw;

    /* GPIOx->BSRR:
     * 低 16 位写 1：置位对应引脚
     * 高 16 位写 1：复位对应引脚
     */
    setw = (1u << STEP_PIN);
    rstw = (1u << (STEP_PIN + 16));

    for (i = 0; i < CHUNK_EDGES; i++)
    {
        /* 偶数：置位（拉高），奇数：复位（拉低） */
        s_dma_buf[i] = (i & 1u) ? rstw : setw;
    }
}

/* =========================
 *  工具：64位无符号整数开方（整数 sqrt）
 * =========================
 * 我们在梯形曲线里需要 sqrt(v^2)：
 *   v = sqrt(v0^2 + 2*a*s)
 *   v = sqrt(2*a*remain)
 *
 * 这里实现一个纯整数开方，避免用 float（F1 上 float 慢且体积大）。
 */
static uint32_t isqrt_u64(uint64_t x)
{
    uint64_t op;
    uint64_t res;
    uint64_t one;

    op = x;
    res = 0;
    one = (uint64_t)1 << 62; /* 最高位对齐 */

    while (one > op)
        one >>= 2;

    while (one != 0)
    {
        if (op >= res + one)
        {
            op -= res + one;
            res = (res >> 1) + one;
        }
        else
        {
            res >>= 1;
        }
        one >>= 2;
    }
    return (uint32_t)res;
}

/* =========================
 *  梯形曲线：给定 step_index（已走步数），计算目标速度 v（steps/s）
 * =========================
 * 分三段：
 * 1) 加速段：v^2 = v0^2 + 2*a*s
 * 2) 匀速段：v = v_peak
 * 3) 减速段（末端到 0）：v^2 = 2*a*s_remain
 */
static uint32_t trap_get_f(uint32_t step_index)
{
    uint32_t total;
    uint32_t done;
    uint32_t remain;
    uint32_t v;
    uint64_t v2;

    total = s_trap.steps_total;
    done = step_index;

    if (total == 0u)
        return 1u;

    if (s_trap.phase_profile)
    {
        uint64_t start_squared = (uint64_t)s_trap.f_start * s_trap.f_start;
        uint64_t terminal_squared = (uint64_t)s_trap.f_end * s_trap.f_end;
        uint64_t delta_squared = 2ull * (uint64_t)s_trap.accel * done;
        if (done >= total)
        {
            return s_trap.f_end;
        }
        /* 加速时间已换算为 steps/s^2；短距离已在启动时提高加速度，段尾仍应达到目标速度。 */
        if (terminal_squared >= start_squared)
        {
            v2 = start_squared + delta_squared;
            if (v2 >= terminal_squared)
                return s_trap.f_end;
        }
        else
        {
            if (delta_squared >= (start_squared - terminal_squared))
                return s_trap.f_end;
            v2 = start_squared - delta_squared;
        }
        v = isqrt_u64(v2);
        return (v < 1u) ? 1u : v;
    }

    remain = (total > done) ? (total - done) : 0u;

    /* Home 接近轮廓在末段保持低速，而不是普通 MOVE 在终点减到 0。 */
    if (s_trap.home_approach)
    {
        if ((remain <= s_trap.slow_zone_steps) &&
            (done >= s_trap.accel_steps))
        {
            return s_trap.f_end;
        }
        if ((remain > s_trap.slow_zone_steps) &&
            (remain <= (s_trap.slow_zone_steps + s_trap.decel_steps)))
        {
            v2 = (uint64_t)s_trap.f_end * (uint64_t)s_trap.f_end +
                 2ull * (uint64_t)s_trap.accel *
                     (uint64_t)(remain - s_trap.slow_zone_steps);
            v = isqrt_u64(v2);
            return (v > s_trap.v_peak) ? s_trap.v_peak : v;
        }
    }

    /* 加速段 */
    if (done < s_trap.accel_steps)
    {
        v2 = (uint64_t)s_trap.f_start * (uint64_t)s_trap.f_start + 2ull * (uint64_t)s_trap.accel * (uint64_t)done;
        v = isqrt_u64(v2);
        if (v > s_trap.v_peak)
            v = s_trap.v_peak;
        if (v < 1u)
            v = 1u;
        return v;
    }

    /* 减速段（末端到 0） */
    if (remain <= s_trap.decel_steps)
    {
        v2 = 2ull * (uint64_t)s_trap.accel * (uint64_t)remain;
        v = isqrt_u64(v2);

        /* remain>0 时，v 可能算出 0（比如 a 很小、remain 很小），
         * 但定时器不能跑 0Hz，所以夹到 1 steps/s。
         * 最后一段做完我们会 stop()，实现真正到 0。
         */
        if (remain > 0u && v < 1u)
            v = 1u;

        if (v > s_trap.v_peak)
            v = s_trap.v_peak;
        return v;
    }

    /* 匀速段 */
    return s_trap.v_peak;
}

/* =========================
 *  设置 TIM5 Update 事件频率（边沿频率 fev）
 * =========================
 * 关键关系：
 *   1 step = 2 edges
 *   fev = 2 * fstep
 *
 * TIM 的 update 频率：
 *   fev = tim_clk / ((PSC+1)*(ARR+1))
 * 我们在这里自动计算 PSC/ARR，尽量用 PSC=0 提高分辨率。
 */
static void tim5_set_edge_freq(uint32_t fev_hz)
{
    uint32_t psc;
    uint32_t arr;
    uint32_t tmp;

    if (fev_hz < 1u)
        fev_hz = 1u;

    psc = 0u;
    tmp = s_tim5_clk / fev_hz;
    if (tmp == 0u)
        tmp = 1u;
    arr = tmp - 1u;

    /* ARR 超出 16bit 时，提高 PSC 再算 */
    if (arr > 0xFFFFu)
    {
        psc = s_tim5_clk / (fev_hz * 65536u);
        if (psc > 0xFFFFu)
            psc = 0xFFFFu;

        tmp = s_tim5_clk / ((psc + 1u) * fev_hz);
        if (tmp == 0u)
            tmp = 1u;
        arr = tmp - 1u;
        if (arr > 0xFFFFu)
            arr = 0xFFFFu;
    }

    if ((TIM5->CR1 & TIM_CR1_CEN) != 0u)
    {
        /* 分段续传期间保持时基连续：PSC 在下一次自然 Update 装载，
         * 禁止 Immediate/UG 和清 CNT，以免人为延长边界 STEP 间隔。 */
        TIM5->PSC = (uint16_t)psc;
        TIM5->ARR = (uint16_t)arr;
        return;
    }

    /* 首段定时器尚未运行，允许归零并强制装载初始周期。 */
    TIM_PrescalerConfig(TIM5, (uint16_t)psc, TIM_PSCReloadMode_Immediate);
    TIM_SetAutoreload(TIM5, (uint16_t)arr);
    TIM_SetCounter(TIM5, 0);
    TIM_GenerateEvent(TIM5, TIM_EventSource_Update);
}

/* =========================
 *  启动一次 DMA 传输：输出 edges 个边沿
 * =========================
 * 首段时基尚未运行，按“停 DMA → 写寄存器 → 开 DMA → 开 TIM”启动。
 * chunk 续传时 TIM 必须连续运行：仅短暂关闭 DMA 后重装，允许禁用窗口内
 * 的 Update 请求自然丢弃，下一次有效请求仍从 SET 边沿开始。
 */
static void dma_start_edges(uint16_t edges)
{
    uint8_t timer_running;

    timer_running = ((TIM5->CR1 & TIM_CR1_CEN) != 0u) ? 1u : 0u;
    DMA_Cmd(DMA2_Channel2, DISABLE);

    /* 重置内存地址与计数器 */
    DMA2_Channel2->CMAR = (uint32_t)s_dma_buf;
    DMA2_Channel2->CNDTR = edges;

    /* 旧 TC 只能在重装后、重新使能 DMA 前清除。 */
    DMA_ClearITPendingBit(DMA2_IT_TC2);

    /* S2 已触发时绝不续装 Y(PB11) 的下一 DMA chunk；正方向脱离不受影响。 */
    if (RobotArmDriver_ShouldStopForNegativeLimit(ROBOT_AXIS_Y))
    {
        stepdma_pb11_stop();
        return;
    }
    if (!timer_running)
    {
        TIM_ClearFlag(TIM5, TIM_FLAG_Update);
    }
    DMA_Cmd(DMA2_Channel2, ENABLE);
    if (!timer_running)
    {
        /* 首段最后才启动时基；chunk 续传绝不停止 TIM5。 */
        TIM_Cmd(TIM5, ENABLE);
    }
}

/* =========================
 *  对外：是否在跑
 * ========================= */
uint8_t stepdma_pb11_is_running(void)
{
    return s_running;
}

/** 获取 PB11 梯形运动已经完成的整段步数。 */
uint32_t stepdma_pb11_get_completed_steps(void)
{
    return s_trap.steps_done;
}

/** 获取 PB11 梯形运动尚未完成的步数。 */
uint32_t stepdma_pb11_get_remaining_steps(void)
{
    return (s_trap.steps_total > s_trap.steps_done) ?
               (s_trap.steps_total - s_trap.steps_done) : 0u;
}

/* =========================
 *  初始化：实际 PU2（PB11）+ TIM5 + DMA2_Channel2 + IRQ
 * ========================= */
void stepdma_pb11_init(uint32_t tim5_clk_hz)
{
    GPIO_InitTypeDef gi;
    TIM_TimeBaseInitTypeDef tb;
    DMA_InitTypeDef di;
    NVIC_InitTypeDef ni;

    s_tim5_clk = tim5_clk_hz;

    /* GPIOB / TIM5 / DMA2 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

    /* PB11 推挽输出 */
    gi.GPIO_Pin = STEP_MASK;
    gi.GPIO_Speed = GPIO_Speed_50MHz;
    gi.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(STEP_GPIO, &gi);
    GPIO_ResetBits(STEP_GPIO, STEP_MASK);

    /* 准备 DMA 边沿序列 */
    fill_dma_buf();

    /* TIM5 基础计数（ARR 后续会在 start 时改） */
    tb.TIM_Prescaler = 0;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_Period = 1000;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM5, &tb);

    /* DMA2_Channel2：TIM5_UP 触发 → 写 GPIOB->BSRR
     * 你已经实测：只有 DMA2_Channel2 能被 TIM5_UP 触发
     */
    DMA_DeInit(DMA2_Channel2);
    di.DMA_PeripheralBaseAddr = (uint32_t)&GPIOB->BSRR;
    di.DMA_MemoryBaseAddr = (uint32_t)s_dma_buf;
    di.DMA_DIR = DMA_DIR_PeripheralDST;
    di.DMA_BufferSize = CHUNK_EDGES;
    di.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    di.DMA_MemoryInc = DMA_MemoryInc_Enable;
    di.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    di.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;

    /* Normal：跑完一段触发 TC 中断，用于精确计步/分段变速 */
    di.DMA_Mode = DMA_Mode_Normal;

    di.DMA_Priority = DMA_Priority_High;
    di.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA2_Channel2, &di);

    /* 开 DMA 传输完成中断（TC2） */
    DMA_ITConfig(DMA2_Channel2, DMA_IT_TC, ENABLE);

    /* DMA2 通道 2/3 通常共用一个 IRQ：DMA2_Channel2_3_IRQn
     * 你的库如果命名不同，可以在这里改宏
     */
#if defined(STM32F10X_HD) || defined(STM32F10X_XL)
    ni.NVIC_IRQChannel = DMA2_Channel2_IRQn;
#else
    ni.NVIC_IRQChannel = DMA2_Channel2_IRQn; /* 少见库 */
#endif
    ni.NVIC_IRQChannelPreemptionPriority = 1;
    ni.NVIC_IRQChannelSubPriority = 1;
    ni.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&ni);

    /* 关键：允许 TIM5 Update 产生 DMA 请求（UDE=1） */
    TIM_DMACmd(TIM5, TIM_DMA_Update, ENABLE);

    /* 初始停住 */
    TIM_Cmd(TIM5, DISABLE);
    DMA_Cmd(DMA2_Channel2, DISABLE);

    s_running = 0;
    s_mode = MODE_IDLE;
    s_edges_left = 0;
}

/* =========================
 *  立即停止
 * ========================= */
void stepdma_pb11_stop(void)
{
    s_running = 0;
    s_mode = MODE_IDLE;
    s_edges_left = 0;

    TIM_Cmd(TIM5, DISABLE);
    DMA_Cmd(DMA2_Channel2, DISABLE);

    GPIO_ResetBits(STEP_GPIO, STEP_MASK);
}

/**
 * 结束 PB11 当前 Phase 的脉冲计数，但保留 TIM5 时基以便同向下一 Phase 直接续装 DMA。
 *
 * 仅允许 0x39 Phase 的最后一个 DMA chunk 调用。DMA 必须先停止且 STEP 保持低电平，
 * 这样 RobotArm 仍可提交本段坐标；不关闭 TIM5 可避免下一条重新启动时产生段间空档。
 */
static void stepdma_pb11_finish_phase(void)
{
    s_running = 0u;
    s_mode = MODE_IDLE;
    s_edges_left = 0u;
    DMA_Cmd(DMA2_Channel2, DISABLE);
    GPIO_ResetBits(STEP_GPIO, STEP_MASK);
}

/* =========================
 *  常速：跑 N 步
 * ========================= */
void stepdma_pb11_move_steps(uint32_t steps, uint32_t fstep_hz)
{
    uint32_t edges_total;
    uint32_t first_edges;
    uint32_t fev;

    if (steps == 0u)
        return;
    if (fstep_hz < 1u)
        fstep_hz = 1u;

    /* 1 step = 2 edges */
    edges_total = steps * 2u;

    /* 边沿频率 fev = 2*fstep */
    fev = 2u * fstep_hz;
    tim5_set_edge_freq(fev);

    /* 首段 edges */
    first_edges = edges_total;
    if (first_edges > CHUNK_EDGES)
        first_edges = CHUNK_EDGES;

    s_edges_left = edges_total - first_edges;

    s_running = 1;
    s_mode = MODE_CONST;

    dma_start_edges((uint16_t)first_edges);
}

/* =========================
 *  梯形/三角形加减速：末端减到 0（更柔）
 * =========================
 * 关键：峰值速度 v_peak 不是一定等于 f_max
 *
 * 当步数不够跑到 f_max，会变成“三角形”，v_peak < f_max。
 *
 * 末端减到 0 的总距离：
 *   加速距离: (v^2 - f0^2)/(2a)
 *   减速距离: (v^2 - 0^2)/(2a) = v^2/(2a)
 *   总距离 steps:
 *     (v^2 - f0^2)/(2a) + v^2/(2a) = steps
 *   => (2v^2 - f0^2)/(2a) = steps
 *   => v^2 = (2*a*steps + f0^2)/2
 *
 * 计算出 v_peak 后，再算：
 *   accel_steps = (v^2 - f0^2)/(2a)
 *   decel_steps = v^2/(2a)
 *
 * 运行时每段（CHUNK_STEPS 步）更新一次速度：
 *   done < accel_steps    -> 用 v^2 = v0^2 + 2as
 *   remain <= decel_steps -> 用 v^2 = 2a*remain
 *   else 匀速 v_peak
 */
void stepdma_pb11_move_trap(uint32_t steps, uint32_t f_start, uint32_t f_max, uint32_t accel)
{
    uint64_t f02;
    uint64_t vpeak2;
    uint32_t vpeak;

    uint32_t f;
    uint32_t fev;

    uint32_t remain_steps;
    uint32_t chunk_steps;
    uint32_t chunk_edges;

    if (steps == 0u)
        return;

    /* 参数修正 */
    if (f_max < 1u)
        f_max = 1u;
    if (f_start < 1u)
        f_start = 1u;
    if (f_start > f_max)
        f_start = f_max;
    if (accel < 1u)
        accel = 1u;

    /* 保存 profile */
    s_trap.steps_total = steps;
    s_trap.steps_done = 0u;
    s_trap.f_start = f_start;
    s_trap.f_max = f_max;
    s_trap.accel = accel;
    s_trap.phase_profile = 0u;
    s_trap.home_approach = 0u;
    s_trap.slow_zone_steps = 0u;

    /* 计算实际峰值速度 v_peak（考虑末端到 0） */
    f02 = (uint64_t)f_start * (uint64_t)f_start;
    vpeak2 = (2ull * (uint64_t)accel * (uint64_t)steps + f02) / 2ull;
    vpeak = isqrt_u64(vpeak2);

    if (vpeak > f_max)
        vpeak = f_max;
    /* 极短位移无法在保持起步速度后再完整减速，峰值不得低于 f_start，
     * 否则后续无符号减法会把 accel_steps 计算成错误的大数。 */
    if (vpeak < f_start)
        vpeak = f_start;
    if (vpeak < 1u)
        vpeak = 1u;

    s_trap.v_peak = vpeak;

    /* 加速段步数：(v^2 - f0^2)/(2a) */
    s_trap.accel_steps = (uint32_t)(((uint64_t)vpeak * (uint64_t)vpeak - f02) / (2ull * (uint64_t)accel));

    /* 减速段步数：v^2/(2a) */
    s_trap.decel_steps = (uint32_t)(((uint64_t)vpeak * (uint64_t)vpeak) / (2ull * (uint64_t)accel));
    if (s_trap.decel_steps < 1u)
        s_trap.decel_steps = 1u;

    /* 首段频率 */
    f = trap_get_f(0u);
    fev = 2u * f; /* fev = 2*fstep */
    tim5_set_edge_freq(fev);

    /* 首段长度 */
    remain_steps = s_trap.steps_total - s_trap.steps_done;
    chunk_steps = (remain_steps > CHUNK_STEPS) ? CHUNK_STEPS : remain_steps;
    chunk_edges = chunk_steps * 2u;

    s_running = 1;
    s_mode = MODE_TRAP;

    dma_start_edges((uint16_t)chunk_edges);
}

/**
 * 在一条连续 PB11 DMA 脉冲序列中执行 Home 的快速、减速和末段低速接近。
 *
 * 减速距离按 v² 差和 acceleration 计算；实际开始距离取该距离加 slow_zone，
 * 因而进入最后低速区时已接近 f_slow，不能等到最后再突变 TIM5 周期。
 */
void stepdma_pb11_move_home_approach(uint32_t steps, uint32_t f_start,
                                     uint32_t f_fast, uint32_t f_slow,
                                     uint32_t slow_zone_steps, uint32_t accel)
{
    uint64_t start2, end2, peak2;
    uint32_t peak, f, remain_steps, chunk_steps, chunk_edges;
    if (steps == 0u) return;
    if (f_start < 1u) f_start = 1u;
    if (f_slow < f_start) f_slow = f_start;
    if (f_fast < f_slow) f_fast = f_slow;
    if (accel < 1u) accel = 1u;
    if (slow_zone_steps > steps) slow_zone_steps = steps;
    start2 = (uint64_t)f_start * f_start;
    end2 = (uint64_t)f_slow * f_slow;
    /* 低速区不参与加减速距离，峰值由其余已知距离决定。 */
    peak2 = (2ull * (uint64_t)accel * (uint64_t)(steps - slow_zone_steps) +
             start2 + end2) / 2ull;
    peak = isqrt_u64(peak2);
    if (peak > f_fast) peak = f_fast;
    if (peak < f_slow) peak = f_slow;
    if (peak < f_start) peak = f_start;
    s_trap.steps_total = steps;
    s_trap.steps_done = 0u;
    s_trap.f_start = f_start;
    s_trap.f_max = f_fast;
    s_trap.f_end = f_slow;
    s_trap.accel = accel;
    s_trap.v_peak = peak;
    s_trap.accel_steps = (uint32_t)(((uint64_t)peak * peak - start2) / (2ull * accel));
    s_trap.decel_steps = (uint32_t)(((uint64_t)peak * peak - end2) / (2ull * accel));
    s_trap.slow_zone_steps = slow_zone_steps;
    s_trap.phase_profile = 0u;
    s_trap.home_approach = 1u;
    f = trap_get_f(0u);
    s_cur_f = f;
    tim5_set_edge_freq(2u * f);
    remain_steps = steps;
    chunk_steps = (remain_steps > CHUNK_STEPS) ? CHUNK_STEPS : remain_steps;
    chunk_edges = chunk_steps * 2u;
    s_running = 1u;
    s_mode = MODE_TRAP;
    dma_start_edges((uint16_t)chunk_edges);
}

/**
 * 计算 0x39 单段应使用的恒定加速度。
 *
 * 上位机给出的加速时间描述正常距离下的速度变化斜率。若本段位移不足以在该时间内
 * 从起始速度达到终止速度，则按本段总脉冲数反算更大的加速度，使理论段尾速度严格
 * 落在终止速度；否则下一段按终止速度续启会形成可感知的速度跳变。
 *
 * @param steps 本段需要输出的 STEP 上升沿数量，必须非 0。
 * @param f_start 本段实际起始速度，单位 steps/s。
 * @param f_terminal 本段目标终止速度，单位 steps/s。
 * @param acceleration_time_ms 上位机要求的加速或减速时间，单位毫秒；0 表示匀速。
 * @return 可直接代入 v^2 = v0^2 + 2as 的加速度，单位 steps/s^2。
 */
static uint32_t stepdma_phase_resolve_acceleration(
    uint32_t steps, uint32_t f_start, uint32_t f_terminal,
    uint32_t acceleration_time_ms)
{
    uint32_t frequency_delta;
    uint32_t time_acceleration;
    uint64_t required_steps;
    uint64_t delta_squared;
    uint64_t endpoint_acceleration;

    if ((acceleration_time_ms == 0u) || (f_start == f_terminal))
    {
        return 1u;
    }

    frequency_delta = (f_terminal >= f_start) ?
        (f_terminal - f_start) : (f_start - f_terminal);
    /* 时间模式下 S_need=(v0+v1)*t/2；向上取整，不让一小段缺口误判为可达。 */
    required_steps = (((uint64_t)f_start + (uint64_t)f_terminal) *
                      (uint64_t)acceleration_time_ms + 1999ull) / 2000ull;
    time_acceleration = (uint32_t)(((uint64_t)frequency_delta * 1000ull +
                                    acceleration_time_ms - 1u) /
                                   acceleration_time_ms);
    if (time_acceleration == 0u)
    {
        time_acceleration = 1u;
    }
    if ((uint64_t)steps >= required_steps)
    {
        return time_acceleration;
    }

    /* 短距离必须在段尾到达 f_terminal：a=|v1^2-v0^2|/(2*S)，同样向上取整。 */
    delta_squared = ((uint64_t)f_terminal * (uint64_t)f_terminal >=
                     (uint64_t)f_start * (uint64_t)f_start) ?
                    ((uint64_t)f_terminal * (uint64_t)f_terminal -
                     (uint64_t)f_start * (uint64_t)f_start) :
                    ((uint64_t)f_start * (uint64_t)f_start -
                     (uint64_t)f_terminal * (uint64_t)f_terminal);
    endpoint_acceleration = (delta_squared + 2ull * (uint64_t)steps - 1ull) /
                            (2ull * (uint64_t)steps);
    return (endpoint_acceleration == 0ull) ? 1u : (uint32_t)endpoint_acceleration;
}

/**
 * 按一个规划 Phase 输出 PB11（Y 轴）脉冲。
 *
 * 起始速度由批次上一条终止速度继承；终止速度与加速时间均来自 0x39 当前条。
 * 距离足够时严格按加速时间换算恒定加速度；距离不足时压缩加速时间，保证段尾
 * 能达到终止速度，避免下一条按终止速度续启时产生明显跳变。
 */
void stepdma_pb11_move_phase(uint32_t steps, uint32_t f_start,
                             uint32_t f_terminal, uint32_t acceleration_time_ms)
{
    uint32_t frequency;
    uint32_t chunk_steps;
    if (steps == 0u)
    {
        return;
    }
    if (f_start == 0u) f_start = PHASE_TIMER_MIN_FREQUENCY;
    if (f_terminal == 0u) f_terminal = PHASE_TIMER_MIN_FREQUENCY;
    if (f_start > 50000u) f_start = 50000u;
    if (f_terminal > 50000u) f_terminal = 50000u;
    /* 0ms 是协议约定的匀速模式：首脉冲即使用本条终止速度，不能参与除以时间的换算。 */
    if (acceleration_time_ms == 0u)
    {
        f_start = f_terminal;
        s_trap.accel = 1u;
    }
    else
    {
        s_trap.accel = stepdma_phase_resolve_acceleration(
            steps, f_start, f_terminal, acceleration_time_ms);
    }
    s_trap.steps_total = steps;
    s_trap.steps_done = 0u;
    s_trap.f_start = f_start;
    s_trap.f_end = f_terminal;
    s_trap.phase_profile = 1u;
    frequency = trap_get_f(0u);
    s_cur_f = frequency;
    tim5_set_edge_freq(2u * frequency);
    chunk_steps = (steps > CHUNK_STEPS) ? CHUNK_STEPS : steps;
    s_running = 1u;
    s_mode = MODE_TRAP;
    dma_start_edges((uint16_t)(chunk_steps * 2u));
}

/* =========================
 *  DMA TC 中断：一段完成后继续下一段
 * =========================
 * - 常速：继续发剩余 edges
 * - 梯形：更新 steps_done，计算下一段目标速度，重设 TIM5 的 fev，再发下一段
 *
 * 注意：
 * 这里我们按 chunk 步数推进（CHUNK_STEPS 或最后不足 CHUNK_STEPS 的那段）。
 * steps_done 的推进必须和每次启动的 chunk_steps 一致。
 */
#ifdef DMA2_Channel2_3_IRQn
void DMA2_Channel2_3_IRQHandler(void)
#else
void DMA2_Channel2_IRQHandler(void)
#endif
{
    /* ===== C89：所有变量声明放最前 ===== */
    uint32_t next_edges;

    uint32_t remain_steps;
    uint32_t chunk_steps;
    uint32_t chunk_edges;

    uint32_t f;
    uint32_t fev;

    uint32_t steps_new;
    uint32_t f_start_new;
    uint32_t f_max_new;
    uint32_t accel_new;

    /* 只处理 DMA2 Channel2 的 TC 中断 */
    if (DMA_GetITStatus(DMA2_IT_TC2) == RESET)
    {
        return;
    }
    DMA_ClearITPendingBit(DMA2_IT_TC2);

    /* 如果外部已停止，安全收尾 */
    if (!s_running)
    {
        stepdma_pb11_stop();
        return;
    }

    /* =========================================================
     * ① pending 软切换（关键）
     * =========================================================
     * 目标：
     * - 不调用 stop()（避免 PB11 被拉低造成“停一下”）
     * - 不从固定 500 重新起步（避免速度瞬间掉很低再加速）
     * - 仍然只在 chunk 边界切换（不会截断脉冲）
     *
     * 策略：
     * - 用当前速度 s_cur_f 作为新任务的 f_start（夹到 f_max）
     * - 直接重建 s_trap profile
     * - 立刻启动下一段 chunk
     */
    if (s_pending.valid)
    {

        steps_new = s_pending.steps;
        f_start_new = s_pending.f_start;
        f_max_new = s_pending.f_max;
        accel_new = s_pending.accel;
        s_pending.valid = 0;

        /* 如果当前速度有效，则用它做平滑接续起步 */
        if (s_cur_f > 0u)
        {
            f_start_new = s_cur_f;
            if (f_start_new > f_max_new)
                f_start_new = f_max_new;
            if (f_start_new < 1u)
                f_start_new = 1u;
        }
        else
        {
            if (f_start_new < 1u)
                f_start_new = 1u;
            if (f_start_new > f_max_new)
                f_start_new = f_max_new;
        }

        if (steps_new == 0u)
        {
            /* 视为停机命令 */
            stepdma_pb11_stop();
            return;
        }

        /* ---------- 重建梯形 profile（末端减到0） ---------- */
        s_trap.steps_total = steps_new;
        s_trap.steps_done = 0u;
        s_trap.f_start = f_start_new;
        s_trap.f_max = (f_max_new < 1u) ? 1u : f_max_new;
        s_trap.accel = (accel_new < 1u) ? 1u : accel_new;

        /* 计算 v_peak / accel_steps / decel_steps */
        {
            uint64_t f02;
            uint64_t vpeak2;
            uint32_t vpeak;

            /* v^2 = (2*a*steps + f0^2)/2  （末端到0） */
            f02 = (uint64_t)s_trap.f_start * (uint64_t)s_trap.f_start;
            vpeak2 = (2ull * (uint64_t)s_trap.accel * (uint64_t)s_trap.steps_total + f02) / 2ull;
            vpeak = isqrt_u64(vpeak2);

            if (vpeak > s_trap.f_max)
                vpeak = s_trap.f_max;
            /* pending 续接同样不能让峰值低于实际续接起步速度。 */
            if (vpeak < s_trap.f_start)
                vpeak = s_trap.f_start;
            if (vpeak < 1u)
                vpeak = 1u;

            s_trap.v_peak = vpeak;

            /* accel_steps = (v^2 - f0^2)/(2a) */
            s_trap.accel_steps =
                (uint32_t)(((uint64_t)vpeak * (uint64_t)vpeak - f02) / (2ull * (uint64_t)s_trap.accel));

            /* decel_steps = v^2/(2a) */
            s_trap.decel_steps =
                (uint32_t)(((uint64_t)vpeak * (uint64_t)vpeak) / (2ull * (uint64_t)s_trap.accel));
            if (s_trap.decel_steps < 1u)
                s_trap.decel_steps = 1u;
        }

        /* 切到梯形模式继续跑 */
        s_mode = MODE_TRAP;
        s_running = 1;

        /* 首段速度 */
        f = trap_get_f(0u);
        if (f < 1u)
            f = 1u;
        s_cur_f = f;

        fev = 2u * f;
        tim5_set_edge_freq(fev);

        /* 首段长度 */
        remain_steps = s_trap.steps_total - s_trap.steps_done;
        chunk_steps = (remain_steps > CHUNK_STEPS) ? CHUNK_STEPS : remain_steps;
        chunk_edges = chunk_steps * 2u;

        dma_start_edges((uint16_t)chunk_edges);
        return;
    }

    /* =========================================================
     * ② 常速模式：继续输出剩余 edges
     * ========================================================= */
    if (s_mode == MODE_CONST)
    {
        if (s_edges_left == 0u)
        {
            stepdma_pb11_stop();
            return;
        }

        next_edges = s_edges_left;
        if (next_edges > CHUNK_EDGES)
            next_edges = CHUNK_EDGES;
        s_edges_left -= next_edges;

        /* 常速模式下，当前速度可认为固定（如果你有保存的话） */
        /* s_cur_f = 固定速度;  // 可选 */

        dma_start_edges((uint16_t)next_edges);
        return;
    }

    /* =========================================================
     * ③ 梯形模式：推进步数 -> 更新速度 -> 发下一段
     * ========================================================= */
    if (s_mode == MODE_TRAP)
    {

        /* 计算上一段步数长度（与启动逻辑一致） */
        remain_steps = s_trap.steps_total - s_trap.steps_done;
        chunk_steps = (remain_steps > CHUNK_STEPS) ? CHUNK_STEPS : remain_steps;

        /* 推进已完成步数 */
        s_trap.steps_done += chunk_steps;

        if (s_trap.steps_done >= s_trap.steps_total)
        {
            if (s_trap.phase_profile)
            {
                stepdma_pb11_finish_phase();
            }
            else
            {
                stepdma_pb11_stop();
            }
            RobotArmProtocol_OnPhaseAxisDmaCompleted(ROBOT_AXIS_Y);
            return;
        }

        /* 下一段速度（末端到0公式由 trap_get_f() 内部完成） */
        f = trap_get_f(s_trap.steps_done);
        if (f < 1u)
            f = 1u;
        s_cur_f = f;

        fev = 2u * f;
        tim5_set_edge_freq(fev);

        /* 下一段长度 */
        remain_steps = s_trap.steps_total - s_trap.steps_done;
        chunk_steps = (remain_steps > CHUNK_STEPS) ? CHUNK_STEPS : remain_steps;
        chunk_edges = chunk_steps * 2u;

        dma_start_edges((uint16_t)chunk_edges);
        return;
    }

    /* =========================================================
     * ④ 未知模式：安全停
     * ========================================================= */
    stepdma_pb11_stop();
}

/* ============================================================
 * 实际 PU1（PB10）步进：TIM6 + DMA2_Channel3
 * 说明：完全照 PB11 的架构写（chunk + normal + TC续段 + 梯形末端到0）
 * ============================================================ */

#define STEP10_GPIO GPIOB
#define STEP10_PIN 10
#define STEP10_MASK (1u << STEP10_PIN)
#define STEPPER2_DIR_595_INDEX 1u
#define STEPPER2_DIR_BIT 4u
#define STEPPER2_DIR_MASK (1u << STEPPER2_DIR_BIT)
#define STEPPER2_MIN_FREQUENCY 1u
#define STEPPER2_MAX_FREQUENCY 50000u
#define STEPPER2_START_FREQUENCY 500u
#define STEPPER2_ACCELERATION ROBOT_ARM_X_ACCELERATION

/* TIM6 计数时钟（一般 72MHz） */
static uint32_t s10_tim6_clk = 72000000u;
static volatile uint32_t s10_cur_f = 0;

/* 运行状态 */
static volatile uint8_t s10_running = 0;
static volatile uint32_t s10_completed_steps = 0u;
static volatile uint32_t s10_remaining_steps = 0u;
/* 已写入 Q4 的实际 DIR1 电平；连续同方向 Phase 不重复触发 595 写入。 */
static int8_t s10_direction_level = -1;

typedef struct
{
    uint8_t valid;
    uint32_t steps;
    uint32_t f_start;
    uint32_t f_max;
    uint32_t accel;
} PendingCmd10;

static volatile PendingCmd10 s10_pending = {0, 0, 0, 0, 0};

/* DMA chunk buffer：交替 set/reset，长度 CHUNK_EDGES */
static uint32_t s10_dma_buf[CHUNK_EDGES];

typedef enum
{
    MODE10_IDLE = 0,
    MODE10_CONST,
    MODE10_TRAP
} StepMode10;

static volatile StepMode10 s10_mode = MODE10_IDLE;

/* 常速模式剩余边沿 */
static volatile uint32_t s10_edges_left = 0;

/* 梯形/三角形 profile */
typedef struct
{
    uint32_t steps_total;
    uint32_t steps_done;

    uint32_t f_start;
    uint32_t f_max;
    uint32_t accel;

    uint32_t v_peak;
    uint32_t accel_steps;
    uint32_t decel_steps;
    uint32_t f_end;
    uint8_t phase_profile;
    uint8_t home_approach;
    uint32_t slow_zone_steps;
} TrapProfile10;

static TrapProfile10 s10_trap;

/* 填充 PB10 DMA buffer（交替 set/reset） */
static void fill_dma_buf10(void)
{
    uint32_t i;
    uint32_t setw;
    uint32_t rstw;

    setw = (1u << STEP10_PIN);
    rstw = (1u << (STEP10_PIN + 16));

    for (i = 0; i < CHUNK_EDGES; i++)
    {
        s10_dma_buf[i] = (i & 1u) ? rstw : setw;
    }
}

static uint32_t trap10_get_f(uint32_t step_index)
{
    uint32_t total;
    uint32_t done;
    uint32_t remain;
    uint32_t v;
    uint64_t v2;

    total = s10_trap.steps_total;
    done = step_index;

    if (total == 0u)
        return 1u;

    if (s10_trap.phase_profile)
    {
        uint64_t start_squared = (uint64_t)s10_trap.f_start * s10_trap.f_start;
        uint64_t terminal_squared = (uint64_t)s10_trap.f_end * s10_trap.f_end;
        uint64_t delta_squared = 2ull * (uint64_t)s10_trap.accel * done;
        if (done >= total)
        {
            return s10_trap.f_end;
        }
        /* 加速时间已换算为 steps/s^2；短距离已在启动时提高加速度，段尾仍应达到目标速度。 */
        if (terminal_squared >= start_squared)
        {
            v2 = start_squared + delta_squared;
            if (v2 >= terminal_squared)
                return s10_trap.f_end;
        }
        else
        {
            if (delta_squared >= (start_squared - terminal_squared))
                return s10_trap.f_end;
            v2 = start_squared - delta_squared;
        }
        v = isqrt_u64(v2);
        return (v < 1u) ? 1u : v;
    }

    remain = (total > done) ? (total - done) : 0u;

    if (s10_trap.home_approach)
    {
        if ((remain <= s10_trap.slow_zone_steps) &&
            (done >= s10_trap.accel_steps)) return s10_trap.f_end;
        if ((remain > s10_trap.slow_zone_steps) &&
            (remain <= (s10_trap.slow_zone_steps + s10_trap.decel_steps)))
        {
            v2 = (uint64_t)s10_trap.f_end * (uint64_t)s10_trap.f_end +
                 2ull * (uint64_t)s10_trap.accel *
                     (uint64_t)(remain - s10_trap.slow_zone_steps);
            v = isqrt_u64(v2);
            return (v > s10_trap.v_peak) ? s10_trap.v_peak : v;
        }
    }

    if (done < s10_trap.accel_steps)
    {
        v2 = (uint64_t)s10_trap.f_start * (uint64_t)s10_trap.f_start + 2ull * (uint64_t)s10_trap.accel * (uint64_t)done;
        v = isqrt_u64(v2);
        if (v > s10_trap.v_peak)
            v = s10_trap.v_peak;
        if (v < 1u)
            v = 1u;
        return v;
    }

    if (remain <= s10_trap.decel_steps)
    {
        v2 = 2ull * (uint64_t)s10_trap.accel * (uint64_t)remain;
        v = isqrt_u64(v2);
        if (remain > 0u && v < 1u)
            v = 1u;
        if (v > s10_trap.v_peak)
            v = s10_trap.v_peak;
        return v;
    }
    return s10_trap.v_peak;
}

static void tim6_set_edge_freq(uint32_t fev_hz)
{
    uint32_t psc;
    uint32_t arr;
    uint32_t tmp;

    if (fev_hz < 1u)
        fev_hz = 1u;

    psc = 0u;
    tmp = s10_tim6_clk / fev_hz;
    if (tmp == 0u)
        tmp = 1u;
    arr = tmp - 1u;

    if (arr > 0xFFFFu)
    {
        psc = s10_tim6_clk / (fev_hz * 65536u);
        if (psc > 0xFFFFu)
            psc = 0xFFFFu;

        tmp = s10_tim6_clk / ((psc + 1u) * fev_hz);
        if (tmp == 0u)
            tmp = 1u;
        arr = tmp - 1u;
        if (arr > 0xFFFFu)
            arr = 0xFFFFu;
    }

    if ((TIM6->CR1 & TIM_CR1_CEN) != 0u)
    {
        /* 分段续传期间不发 UG、不清 CNT，下一次自然 Update 才装载新 PSC。 */
        TIM6->PSC = (uint16_t)psc;
        TIM6->ARR = (uint16_t)arr;
        return;
    }

    /* 首段尚未运行，按既有方式装载初始周期。 */
    TIM_PrescalerConfig(TIM6, (uint16_t)psc, TIM_PSCReloadMode_Immediate);
    TIM_SetAutoreload(TIM6, (uint16_t)arr);
    TIM_SetCounter(TIM6, 0);
    TIM_GenerateEvent(TIM6, TIM_EventSource_Update);
}

static void dma10_start_edges(uint16_t edges)
{
    uint8_t timer_running;

    timer_running = ((TIM6->CR1 & TIM_CR1_CEN) != 0u) ? 1u : 0u;
    DMA_Cmd(DMA2_Channel3, DISABLE);

    DMA2_Channel3->CMAR = (uint32_t)s10_dma_buf;
    DMA2_Channel3->CNDTR = edges;

    /* 旧 TC 只能在重装后、重新使能 DMA 前清除。 */
    DMA_ClearITPendingBit(DMA2_IT_TC3);

    /* S1 已触发时绝不续装 X(PB10) 的下一 DMA chunk；只拦截逻辑负方向。 */
    if (RobotArmDriver_ShouldStopForNegativeLimit(ROBOT_AXIS_X))
    {
        stepdma_pb10_stop();
        return;
    }
    if (!timer_running)
    {
        TIM_ClearFlag(TIM6, TIM_FLAG_Update);
    }
    DMA_Cmd(DMA2_Channel3, ENABLE);
    if (!timer_running)
    {
        /* 首段最后才启动时基；chunk 续传保持 TIM6 连续计数。 */
        TIM_Cmd(TIM6, ENABLE);
    }
}

uint8_t stepdma_pb10_is_running(void)
{
    return s10_running;
}

void stepdma_pb10_init(uint32_t tim6_clk_hz)
{
    GPIO_InitTypeDef gi;
    TIM_TimeBaseInitTypeDef tb;
    DMA_InitTypeDef di;
    NVIC_InitTypeDef ni;

    s10_tim6_clk = tim6_clk_hz;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

    gi.GPIO_Pin = STEP10_MASK;
    gi.GPIO_Speed = GPIO_Speed_50MHz;
    gi.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(STEP10_GPIO, &gi);
    GPIO_ResetBits(STEP10_GPIO, STEP10_MASK);

    fill_dma_buf10();

    tb.TIM_Prescaler = 0;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_Period = 1000;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM6, &tb);

    DMA_DeInit(DMA2_Channel3);
    di.DMA_PeripheralBaseAddr = (uint32_t)&GPIOB->BSRR;
    di.DMA_MemoryBaseAddr = (uint32_t)s10_dma_buf;
    di.DMA_DIR = DMA_DIR_PeripheralDST;
    di.DMA_BufferSize = CHUNK_EDGES;
    di.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    di.DMA_MemoryInc = DMA_MemoryInc_Enable;
    di.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    di.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    di.DMA_Mode = DMA_Mode_Normal;
    di.DMA_Priority = DMA_Priority_High;
    di.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA2_Channel3, &di);

    DMA_ITConfig(DMA2_Channel3, DMA_IT_TC, ENABLE);

    /* 你的 startup 有 DMA2_Channel3_IRQHandler，所以这里用 DMA2_Channel3_IRQn */
    ni.NVIC_IRQChannel = DMA2_Channel3_IRQn;
    ni.NVIC_IRQChannelPreemptionPriority = 1;
    ni.NVIC_IRQChannelSubPriority = 2;
    ni.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&ni);

    TIM_DMACmd(TIM6, TIM_DMA_Update, ENABLE);

    TIM_Cmd(TIM6, DISABLE);
    DMA_Cmd(DMA2_Channel3, DISABLE);

    s10_running = 0;
    s10_mode = MODE10_IDLE;
    s10_edges_left = 0;
    s10_cur_f = 0;
    s10_pending.valid = 0;
    s10_completed_steps = 0u;
    s10_remaining_steps = 0u;
}

void stepdma_pb10_stop(void)
{
    s10_running = 0;
    s10_mode = MODE10_IDLE;
    s10_edges_left = 0;
    s10_cur_f = 0;
    s10_pending.valid = 0;

    TIM_Cmd(TIM6, DISABLE);
    DMA_Cmd(DMA2_Channel3, DISABLE);

    GPIO_ResetBits(STEP10_GPIO, STEP10_MASK);
}

/**
 * 结束 PB10 当前 Phase 的脉冲计数，但保留 TIM6 时基以便同向下一 Phase 直接续装 DMA。
 *
 * 仅用于 0x39 Phase 的最后一个 DMA chunk。逻辑 running 必须先清除，以便 RobotArm
 * 依据已完成步数提交当前坐标；DMA 和 STEP 低电平仍受控，TIM6 仅作为无输出的连续时基保留。
 */
static void stepdma_pb10_finish_phase(void)
{
    s10_running = 0u;
    s10_mode = MODE10_IDLE;
    s10_edges_left = 0u;
    DMA_Cmd(DMA2_Channel3, DISABLE);
    GPIO_ResetBits(STEP10_GPIO, STEP10_MASK);
}

void stepdma_pb10_move_steps(uint32_t steps, uint32_t fstep_hz)
{
    uint32_t edges_total;
    uint32_t first_edges;
    uint32_t fev;

    if (steps == 0u)
        return;
    if (fstep_hz < 1u)
        fstep_hz = 1u;

    edges_total = steps * 2u;
    fev = 2u * fstep_hz;
    tim6_set_edge_freq(fev);

    first_edges = edges_total;
    if (first_edges > CHUNK_EDGES)
        first_edges = CHUNK_EDGES;

    s10_edges_left = edges_total - first_edges;
    s10_completed_steps = 0u;
    s10_remaining_steps = steps;

    s10_running = 1;
    s10_mode = MODE10_CONST;

    dma10_start_edges((uint16_t)first_edges);
}

void stepdma_pb10_move_trap(uint32_t steps, uint32_t f_start, uint32_t f_max, uint32_t accel)
{
    uint64_t f02;
    uint64_t vpeak2;
    uint32_t vpeak;

    uint32_t f;
    uint32_t fev;

    uint32_t remain_steps;
    uint32_t chunk_steps;
    uint32_t chunk_edges;

    if (steps == 0u)
        return;

    if (f_max < 1u)
        f_max = 1u;
    if (f_start < 1u)
        f_start = 1u;
    if (f_start > f_max)
        f_start = f_max;
    if (accel < 1u)
        accel = 1u;

    s10_trap.steps_total = steps;
    s10_trap.steps_done = 0u;
    s10_completed_steps = 0u;
    s10_remaining_steps = steps;
    s10_trap.f_start = f_start;
    s10_trap.f_max = f_max;
    s10_trap.accel = accel;
    s10_trap.phase_profile = 0u;
    s10_trap.home_approach = 0u;
    s10_trap.slow_zone_steps = 0u;

    f02 = (uint64_t)f_start * (uint64_t)f_start;
    vpeak2 = (2ull * (uint64_t)accel * (uint64_t)steps + f02) / 2ull;
    vpeak = isqrt_u64(vpeak2);

    if (vpeak > f_max)
        vpeak = f_max;
    /* 32 step 以下的极短位移也必须保留合法的三角形 profile。 */
    if (vpeak < f_start)
        vpeak = f_start;
    if (vpeak < 1u)
        vpeak = 1u;

    s10_trap.v_peak = vpeak;

    s10_trap.accel_steps = (uint32_t)(((uint64_t)vpeak * (uint64_t)vpeak - f02) / (2ull * (uint64_t)accel));
    s10_trap.decel_steps = (uint32_t)(((uint64_t)vpeak * (uint64_t)vpeak) / (2ull * (uint64_t)accel));
    if (s10_trap.decel_steps < 1u)
        s10_trap.decel_steps = 1u;

    f = trap10_get_f(0u);
    if (f < 1u)
        f = 1u;
    s10_cur_f = f;

    fev = 2u * f;
    tim6_set_edge_freq(fev);

    remain_steps = s10_trap.steps_total - s10_trap.steps_done;
    chunk_steps = (remain_steps > CHUNK_STEPS) ? CHUNK_STEPS : remain_steps;
    chunk_edges = chunk_steps * 2u;

    s10_running = 1;
    s10_mode = MODE10_TRAP;

    dma10_start_edges((uint16_t)chunk_edges);
}

/** PB10/X 轴的连续 Home 接近轮廓，公式和 PB11 保持一致。 */
void stepdma_pb10_move_home_approach(uint32_t steps, uint32_t f_start,
                                     uint32_t f_fast, uint32_t f_slow,
                                     uint32_t slow_zone_steps, uint32_t accel)
{
    uint64_t start2, end2, peak2;
    uint32_t peak, f, chunk_steps, chunk_edges;
    if (steps == 0u) return;
    if (f_start < 1u) f_start = 1u;
    if (f_slow < f_start) f_slow = f_start;
    if (f_fast < f_slow) f_fast = f_slow;
    if (accel < 1u) accel = 1u;
    if (slow_zone_steps > steps) slow_zone_steps = steps;
    start2 = (uint64_t)f_start * f_start;
    end2 = (uint64_t)f_slow * f_slow;
    peak2 = (2ull * (uint64_t)accel * (uint64_t)(steps - slow_zone_steps) +
             start2 + end2) / 2ull;
    peak = isqrt_u64(peak2);
    if (peak > f_fast) peak = f_fast;
    if (peak < f_slow) peak = f_slow;
    if (peak < f_start) peak = f_start;
    s10_trap.steps_total = steps;
    s10_trap.steps_done = 0u;
    s10_completed_steps = 0u;
    s10_remaining_steps = steps;
    s10_trap.f_start = f_start;
    s10_trap.f_max = f_fast;
    s10_trap.f_end = f_slow;
    s10_trap.accel = accel;
    s10_trap.v_peak = peak;
    s10_trap.accel_steps = (uint32_t)(((uint64_t)peak * peak - start2) / (2ull * accel));
    s10_trap.decel_steps = (uint32_t)(((uint64_t)peak * peak - end2) / (2ull * accel));
    s10_trap.slow_zone_steps = slow_zone_steps;
    s10_trap.phase_profile = 0u;
    s10_trap.home_approach = 1u;
    f = trap10_get_f(0u);
    s10_cur_f = f;
    tim6_set_edge_freq(2u * f);
    chunk_steps = (steps > CHUNK_STEPS) ? CHUNK_STEPS : steps;
    chunk_edges = chunk_steps * 2u;
    s10_running = 1u;
    s10_mode = MODE10_TRAP;
    dma10_start_edges((uint16_t)chunk_edges);
}

/**
 * 按一个规划 Phase 输出 PB10（X 轴）脉冲，终止速度和加速时间由 0x39 当前条决定。
 *
 * 起始速度由批次上一条终止速度继承；60Hz 等合法频率必须原样进入 TIM6。
 * 距离不足以按给定时间达到终止速度时，压缩实际加速时间以保证段尾速度连续。
 */
void stepdma_pb10_move_phase(uint32_t steps, uint32_t f_start,
                             uint32_t f_terminal, uint32_t acceleration_time_ms)
{
    uint32_t frequency;
    uint32_t chunk_steps;
    if (steps == 0u)
    {
        return;
    }
    if (f_start == 0u) f_start = PHASE_TIMER_MIN_FREQUENCY;
    if (f_terminal == 0u) f_terminal = PHASE_TIMER_MIN_FREQUENCY;
    if (f_start > STEPPER2_MAX_FREQUENCY) f_start = STEPPER2_MAX_FREQUENCY;
    if (f_terminal > STEPPER2_MAX_FREQUENCY) f_terminal = STEPPER2_MAX_FREQUENCY;
    /* 0ms 是协议约定的匀速模式：首脉冲即使用本条终止速度，不能参与除以时间的换算。 */
    if (acceleration_time_ms == 0u)
    {
        f_start = f_terminal;
        s10_trap.accel = 1u;
    }
    else
    {
        s10_trap.accel = stepdma_phase_resolve_acceleration(
            steps, f_start, f_terminal, acceleration_time_ms);
    }
    s10_trap.steps_total = steps;
    s10_trap.steps_done = 0u;
    s10_completed_steps = 0u;
    s10_remaining_steps = steps;
    s10_trap.f_start = f_start;
    s10_trap.f_end = f_terminal;
    s10_trap.phase_profile = 1u;
    frequency = trap10_get_f(0u);
    s10_cur_f = frequency;
    tim6_set_edge_freq(2u * frequency);
    chunk_steps = (steps > CHUNK_STEPS) ? CHUNK_STEPS : steps;
    s10_running = 1u;
    s10_mode = MODE10_TRAP;
    dma10_start_edges((uint16_t)(chunk_steps * 2u));
}

void stepdma_pb10_request_trap(uint32_t steps, uint32_t f_start, uint32_t f_max, uint32_t accel)
{
    uint8_t running_now;

    if (steps == 0u)
    {
        stepdma_pb10_stop();
        return;
    }
    if (f_max < 1u)
        f_max = 1u;
    if (f_start < 1u)
        f_start = 1u;
    if (f_start > f_max)
        f_start = f_max;
    if (accel < 1u)
        accel = 1u;

    s10_pending.steps = steps;
    s10_pending.f_start = f_start;
    s10_pending.f_max = f_max;
    s10_pending.accel = accel;
    s10_pending.valid = 1u;

    running_now = s10_running;
    if (!running_now)
    {
        s10_pending.valid = 0u;
        stepdma_pb10_move_trap(steps, f_start, f_max, accel);
    }
}

/* 这个函数名必须和 startup 向量表一致：DMA2_Channel3_IRQHandler */
void DMA2_Channel3_IRQHandler(void)
{
    uint32_t next_edges;

    uint32_t remain_steps;
    uint32_t chunk_steps;
    uint32_t chunk_edges;

    uint32_t f;
    uint32_t fev;

    uint32_t steps_new;
    uint32_t f_start_new;
    uint32_t f_max_new;
    uint32_t accel_new;

    if (DMA_GetITStatus(DMA2_IT_TC3) == RESET)
    {
        return;
    }
    DMA_ClearITPendingBit(DMA2_IT_TC3);

    if (!s10_running)
    {
        stepdma_pb10_stop();
        return;
    }

    if (s10_pending.valid)
    {
        steps_new = s10_pending.steps;
        f_start_new = s10_pending.f_start;
        f_max_new = s10_pending.f_max;
        accel_new = s10_pending.accel;
        s10_pending.valid = 0;

        if (s10_cur_f > 0u)
        {
            f_start_new = s10_cur_f;
            if (f_start_new > f_max_new)
                f_start_new = f_max_new;
            if (f_start_new < 1u)
                f_start_new = 1u;
        }
        else
        {
            if (f_start_new < 1u)
                f_start_new = 1u;
            if (f_start_new > f_max_new)
                f_start_new = f_max_new;
        }

        if (steps_new == 0u)
        {
            stepdma_pb10_stop();
            return;
        }

        stepdma_pb10_move_trap(steps_new, f_start_new, f_max_new, accel_new);
        return;
    }

    if (s10_mode == MODE10_CONST)
    {
        chunk_steps = (CHUNK_EDGES / 2u);
        if (s10_remaining_steps < chunk_steps)
        {
            chunk_steps = s10_remaining_steps;
        }
        s10_completed_steps += chunk_steps;
        s10_remaining_steps -= chunk_steps;
        if (s10_edges_left == 0u)
        {
            stepdma_pb10_stop();
            return;
        }

        next_edges = s10_edges_left;
        if (next_edges > CHUNK_EDGES)
            next_edges = CHUNK_EDGES;
        s10_edges_left -= next_edges;

        dma10_start_edges((uint16_t)next_edges);
        return;
    }

    if (s10_mode == MODE10_TRAP)
    {

        remain_steps = s10_trap.steps_total - s10_trap.steps_done;
        chunk_steps = (remain_steps > CHUNK_STEPS) ? CHUNK_STEPS : remain_steps;
        s10_trap.steps_done += chunk_steps;
        s10_completed_steps = s10_trap.steps_done;
        s10_remaining_steps = s10_trap.steps_total - s10_trap.steps_done;

        if (s10_trap.steps_done >= s10_trap.steps_total)
        {
            if (s10_trap.phase_profile)
            {
                stepdma_pb10_finish_phase();
            }
            else
            {
                stepdma_pb10_stop();
            }
            RobotArmProtocol_OnPhaseAxisDmaCompleted(ROBOT_AXIS_X);
            return;
        }

        f = trap10_get_f(s10_trap.steps_done);
        if (f < 1u)
            f = 1u;
        s10_cur_f = f;

        fev = 2u * f;
        tim6_set_edge_freq(fev);

        remain_steps = s10_trap.steps_total - s10_trap.steps_done;
        chunk_steps = (remain_steps > CHUNK_STEPS) ? CHUNK_STEPS : remain_steps;
        chunk_edges = chunk_steps * 2u;

        dma10_start_edges((uint16_t)chunk_edges);
        return;
    }

    stepdma_pb10_stop();
}

/** 初始化实际 PU1：PB10 STEP、TIM6 和 DMA2 通道3。 */
void Stepper2_Init(void)
{
    stepdma_pb10_init(72000000u);
}

/** 设置第二轴方向；运行中请求会被忽略，避免脉冲期间换向。 */
void Stepper2_SetDirection(uint8_t direction)
{
    if (stepdma_pb10_is_running())
    {
        return;
    }
    if (s10_direction_level == (int8_t)(direction ? 1u : 0u))
    {
        return;
    }

    /* 实际 PU1（X 轴）的方向使用 U86 DIR1(Q4)，仅修改 HC595Data[1] 的该位。 */
    if (direction)
    {
        HC595Data[STEPPER2_DIR_595_INDEX] |= (uint8_t)STEPPER2_DIR_MASK;
    }
    else
    {
        HC595Data[STEPPER2_DIR_595_INDEX] &= (uint8_t)~STEPPER2_DIR_MASK;
    }
    ShiftRegister_WriteAll(HC595Data);
    s10_direction_level = (int8_t)(direction ? 1u : 0u);
}

/**
 * 使用 X 轴默认加速度启动实际 PU1（PB10）梯形运动。
 *
 * 普通 X 轴移动通过该入口保持既有默认加速度；需要单独指定 Home 加速度时必须
 * 改用 Stepper2_StartWithAcceleration，避免协议参数被此默认值覆盖。
 *
 * @param direction 已转换为实际 DIR1 电平的方向值。
 * @param steps 需要输出的 STEP 上升沿数量。
 * @param target_frequency 目标速度，单位 steps/s。
 * @return DMA 已开始输出返回 1；零步数或驱动忙返回 0。
 */
uint8_t Stepper2_Start(uint8_t direction, uint32_t steps, uint32_t target_frequency)
{
    return Stepper2_StartWithAcceleration(
        direction, steps, target_frequency, STEPPER2_ACCELERATION);
}

/**
 * 使用调用方给定的加速度启动实际 PU1（PB10/X 轴）梯形运动。
 *
 * 单轴 Home 通过该入口把协议中的加速度交给 TIM6/DMA2 通道3；普通移动由
 * Stepper2_Start 传入 X 轴默认值，因此两类动作互不改变对方的速度曲线。
 *
 * @param direction 已写入 DIR1 的方向电平。
 * @param steps 需要输出的 STEP 上升沿数量。
 * @param target_frequency 目标速度，单位 steps/s。
 * @param acceleration 本次加速度，单位 steps/s^2。
 * @return 成功装载 DMA 并开始运动返回 1；零步数或驱动忙返回 0。
 */
uint8_t Stepper2_StartWithAcceleration(uint8_t direction, uint32_t steps,
                                       uint32_t target_frequency,
                                       uint32_t acceleration)
{
    if (steps == 0u || stepdma_pb10_is_running())
    {
        return 0u;
    }
    if (target_frequency < STEPPER2_MIN_FREQUENCY)
    {
        target_frequency = STEPPER2_MIN_FREQUENCY;
    }
    if (target_frequency > STEPPER2_MAX_FREQUENCY)
    {
        target_frequency = STEPPER2_MAX_FREQUENCY;
    }
    if (acceleration < 1u)
    {
        acceleration = 1u;
    }

    /* 方向建立后再装载 DMA，避免首个 STEP 上升沿过早到达驱动器。 */
    stepdma_pb10_stop();
    Stepper2_SetDirection(direction);
    Delay_us(2u);
    stepdma_pb10_move_trap(steps, STEPPER2_START_FREQUENCY,
                           target_frequency, acceleration);
    return 1u;
}

/**
 * 启动实际 PU1（X 轴）的 Phase 终止速度动作。
 *
 * 方向与普通动作采用同一 74HC595 DIR1 映射；仅替换 PB10/TIM6/DMA 的速度曲线，
 * 终止速度和加速时间由 0x39 当前条提供，不建立第二套坐标或限位路径。
 */
uint8_t Stepper2_StartPhase(uint8_t direction, uint32_t steps,
                             uint32_t start_frequency,
                             uint32_t terminal_frequency,
                             uint32_t acceleration_time_ms)
{
    uint8_t direction_changed;
    if (steps == 0u || stepdma_pb10_is_running())
    {
        return 0u;
    }
    direction_changed = (s10_direction_level != (int8_t)(direction ? 1u : 0u)) ? 1u : 0u;
    /* 同向 Phase 已由 finish_phase 保留 TIM6；只有确实需要改 DIR1 时才关闭时基。 */
    if (direction_changed)
    {
        stepdma_pb10_stop();
    }
    Stepper2_SetDirection(direction);
    /* 同一 Batch 已在首条建立 DIR1，续条不重复等待方向建立时间。 */
    if (direction_changed)
    {
        Delay_us(2u);
    }
    stepdma_pb10_move_phase(steps, start_frequency, terminal_frequency, acceleration_time_ms);
    return stepdma_pb10_is_running();
}

/** 立即停止第二轴，并将 PB10 保持为低电平。 */
void Stepper2_Stop(void)
{
    stepdma_pb10_stop();
}

/** 查询第二轴是否正在运行。 */
uint8_t Stepper2_IsBusy(void)
{
    return stepdma_pb10_is_running();
}

/** 获取第二轴已完成的上升沿步数。 */
uint32_t Stepper2_GetCompletedSteps(void)
{
    return s10_completed_steps;
}

/** 获取第二轴尚未完成的步数。 */
uint32_t Stepper2_GetRemainingSteps(void)
{
    return s10_remaining_steps;
}

/* PU3 独立 DMA 步进状态；PB13 由 BSRR 写入，不占用任何定时器复用输出。 */
#define PU3_CHUNK_STEPS 32u
#define PU3_CHUNK_EDGES (PU3_CHUNK_STEPS * 2u)
#define PU3_DIR_595_INDEX 1u
#define PU3_DIR_595_MASK (1u << 6)

typedef enum
{
    PU3_ACCEL_IDLE,
    PU3_ACCEL_UP,
    PU3_ACCEL_CRUISE,
    PU3_ACCEL_DOWN
} PU3_AccelState;

/** PU3 的梯形/三角形速度 profile，字段语义与 PB10/PB11 保持一致。 */
typedef struct
{
    uint32_t steps_total;
    uint32_t f_start;
    uint32_t f_max;
    uint32_t accel;
    uint32_t v_peak;
    uint32_t accel_steps;
    uint32_t decel_steps;
    uint32_t f_end;
    uint32_t slow_zone_steps;
    uint8_t home_approach;
} PU3_TrapProfile;

typedef struct
{
    volatile uint8_t running;
    volatile uint32_t remaining_steps;
    volatile uint32_t current_chunk_steps;
    volatile uint32_t current_speed;
    volatile uint32_t target_speed;
    volatile uint8_t direction;
    volatile PU3_AccelState accel_state;
    uint32_t total_steps;
    uint32_t completed_steps;
    uint32_t start_speed;
    uint32_t acceleration;
} PU3_State;
static volatile PU3_State s_pu3;
static volatile PU3_TrapProfile s_pu3_trap;
static uint32_t s_pu3_edges[PU3_CHUNK_EDGES];
/* 已写入 Q6 的实际 DIR3 电平；连续同方向 Phase 不重复触发 595 写入。 */
static int8_t s_pu3_direction_level = -1;

/* 只修改 DIR3 的 Q6，确保第二片 595 其余输出不被覆盖。 */
static void PU3_SetDirection(uint8_t direction)
{
    if (s_pu3_direction_level == (int8_t)(direction ? 1u : 0u))
    {
        return;
    }
    if (direction)
        HC595Data[PU3_DIR_595_INDEX] |= (uint8_t)PU3_DIR_595_MASK;
    else
        HC595Data[PU3_DIR_595_INDEX] &= (uint8_t)~PU3_DIR_595_MASK;
    ShiftRegister_WriteAll(HC595Data);
    s_pu3_direction_level = (int8_t)(direction ? 1u : 0u);
}

/**
 * 按 PU3 已完成步数计算下一段实际速度。
 *
 * 使用与 X/Y 相同的整数匀加速公式；短距离由峰值速度自动形成三角形，
 * 最后一段根据剩余步数减速，不能直接跳回起步速度。
 *
 * @param steps_done 已完成的 STEP 上升沿数量。
 * @return 下一段速度，单位为 steps/s，至少为 1。
 */
static uint32_t PU3_GetTrapFrequency(uint32_t steps_done)
{
    uint32_t remaining;
    uint32_t speed;
    uint64_t speed_squared;

    if (s_pu3_trap.steps_total == 0u)
    {
        return 1u;
    }

    remaining = (s_pu3_trap.steps_total > steps_done) ?
                    (s_pu3_trap.steps_total - steps_done) : 0u;
    if (s_pu3_trap.home_approach)
    {
        if ((remaining <= s_pu3_trap.slow_zone_steps) &&
            (steps_done >= s_pu3_trap.accel_steps))
        {
            return s_pu3_trap.f_end;
        }
        if ((remaining > s_pu3_trap.slow_zone_steps) &&
            (remaining <= (s_pu3_trap.slow_zone_steps +
                           s_pu3_trap.decel_steps)))
        {
            speed_squared = (uint64_t)s_pu3_trap.f_end * s_pu3_trap.f_end +
                            2ull * (uint64_t)s_pu3_trap.accel *
                                (remaining - s_pu3_trap.slow_zone_steps);
            speed = isqrt_u64(speed_squared);
            return (speed > s_pu3_trap.v_peak) ? s_pu3_trap.v_peak : speed;
        }
    }
    if (steps_done < s_pu3_trap.accel_steps)
    {
        speed_squared = (uint64_t)s_pu3_trap.f_start *
                            (uint64_t)s_pu3_trap.f_start +
                        2ull * (uint64_t)s_pu3_trap.accel *
                            (uint64_t)steps_done;
        speed = isqrt_u64(speed_squared);
        if (speed > s_pu3_trap.v_peak)
        {
            speed = s_pu3_trap.v_peak;
        }
        return (speed < 1u) ? 1u : speed;
    }

    if (remaining <= s_pu3_trap.decel_steps)
    {
        speed_squared = 2ull * (uint64_t)s_pu3_trap.accel *
                        (uint64_t)remaining;
        speed = isqrt_u64(speed_squared);
        if ((remaining > 0u) && (speed < 1u))
        {
            speed = 1u;
        }
        return (speed > s_pu3_trap.v_peak) ? s_pu3_trap.v_peak : speed;
    }

    return s_pu3_trap.v_peak;
}

/**
 * 根据本次 PU3 总步数建立梯形或三角形速度 profile。
 *
 * 峰值速度同时满足从起步速度加速及在终点前减至零的距离约束，
 * 因而短距离不会强行跑到请求的目标速度。
 *
 * @param steps 本次运动总 STEP 上升沿数量。
 * @param start_frequency 起步速度，单位为 steps/s。
 * @param maximum_frequency 目标速度上限，单位为 steps/s。
 * @param acceleration 加速度，单位为 steps/s^2。
 */
static void PU3_BuildTrapProfile(uint32_t steps,
                                 uint32_t start_frequency,
                                 uint32_t maximum_frequency,
                                 uint32_t acceleration)
{
    uint64_t start_squared;
    uint64_t peak_squared;
    uint32_t peak_speed;

    s_pu3_trap.steps_total = steps;
    s_pu3_trap.f_start = start_frequency;
    s_pu3_trap.f_max = maximum_frequency;
    s_pu3_trap.accel = acceleration;
    s_pu3_trap.home_approach = 0u;
    s_pu3_trap.slow_zone_steps = 0u;

    start_squared = (uint64_t)start_frequency * (uint64_t)start_frequency;
    peak_squared = (2ull * (uint64_t)acceleration * (uint64_t)steps +
                    start_squared) / 2ull;
    peak_speed = isqrt_u64(peak_squared);
    if (peak_speed > maximum_frequency)
    {
        peak_speed = maximum_frequency;
    }
    /* 峰值低于起步速度会导致 accel_steps 的无符号减法下溢。 */
    if (peak_speed < start_frequency)
    {
        peak_speed = start_frequency;
    }
    if (peak_speed < 1u)
    {
        peak_speed = 1u;
    }
    s_pu3_trap.v_peak = peak_speed;
    s_pu3_trap.accel_steps = (uint32_t)(
        ((uint64_t)peak_speed * (uint64_t)peak_speed - start_squared) /
        (2ull * (uint64_t)acceleration));
    s_pu3_trap.decel_steps = (uint32_t)(
        ((uint64_t)peak_speed * (uint64_t)peak_speed) /
        (2ull * (uint64_t)acceleration));
    if (s_pu3_trap.decel_steps < 1u)
    {
        s_pu3_trap.decel_steps = 1u;
    }
}

/** 在 PB13/TIM7/DMA2 通道4上建立连续的 Home 末段低速轮廓。 */
static void PU3_BuildHomeApproachProfile(uint32_t steps, uint32_t start_frequency,
                                         uint32_t fast_frequency,
                                         uint32_t slow_frequency,
                                         uint32_t slow_zone_steps,
                                         uint32_t acceleration)
{
    uint64_t start2, end2, peak2;
    uint32_t peak;
    if (slow_zone_steps > steps) slow_zone_steps = steps;
    start2 = (uint64_t)start_frequency * start_frequency;
    end2 = (uint64_t)slow_frequency * slow_frequency;
    peak2 = (2ull * (uint64_t)acceleration * (steps - slow_zone_steps) +
             start2 + end2) / 2ull;
    peak = isqrt_u64(peak2);
    if (peak > fast_frequency) peak = fast_frequency;
    if (peak < slow_frequency) peak = slow_frequency;
    if (peak < start_frequency) peak = start_frequency;
    s_pu3_trap.steps_total = steps;
    s_pu3_trap.f_start = start_frequency;
    s_pu3_trap.f_max = fast_frequency;
    s_pu3_trap.f_end = slow_frequency;
    s_pu3_trap.accel = acceleration;
    s_pu3_trap.v_peak = peak;
    s_pu3_trap.accel_steps = (uint32_t)((((uint64_t)peak * peak) - start2) /
                                         (2ull * acceleration));
    s_pu3_trap.decel_steps = (uint32_t)((((uint64_t)peak * peak) - end2) /
                                         (2ull * acceleration));
    s_pu3_trap.slow_zone_steps = slow_zone_steps;
    s_pu3_trap.home_approach = 1u;
}

static void PU3_SetEdgeFrequency(uint32_t edge_frequency)
{
    uint32_t divisor;
    if (edge_frequency < 1u)
        edge_frequency = 1u;
    divisor = 72000000u / edge_frequency;
    if (divisor < 1u)
        divisor = 1u;
    if (divisor > 65536u)
        divisor = 65536u;
    if ((PU3_TIMER->CR1 & TIM_CR1_CEN) != 0u)
    {
        /* PU3 续段时不写 Immediate/UG 且不清 CNT，保持当前时基相位。 */
        PU3_TIMER->PSC = 0u;
        PU3_TIMER->ARR = (uint16_t)(divisor - 1u);
        return;
    }

    /* 首段尚未运行，按既有方式初始化计数起点。 */
    TIM_PrescalerConfig(PU3_TIMER, 0u, TIM_PSCReloadMode_Immediate);
    TIM_SetAutoreload(PU3_TIMER, (uint16_t)(divisor - 1u));
    TIM_SetCounter(PU3_TIMER, 0u);
}

static void PU3_LoadChunk(uint32_t steps)
{
    uint8_t timer_running;

    timer_running = ((PU3_TIMER->CR1 & TIM_CR1_CEN) != 0u) ? 1u : 0u;
    DMA_Cmd(PU3_DMA_CHANNEL, DISABLE);
    DMA_ClearITPendingBit(DMA2_IT_TC4);
    /* S3 已触发时绝不续装 PU3(PB13) 的下一 DMA chunk；允许 Z+ 脱离 S3。 */
    if (RobotArmDriver_ShouldStopForNegativeLimit(ROBOT_AXIS_Z))
    {
        PU3_Stepper_Stop();
        return;
    }
    s_pu3.current_chunk_steps = steps;
    PU3_DMA_CHANNEL->CMAR = (uint32_t)s_pu3_edges;
    PU3_DMA_CHANNEL->CNDTR = (uint16_t)(steps * 2u);
    if (!timer_running)
    {
        TIM_ClearFlag(PU3_TIMER, TIM_FLAG_Update);
    }
    /* 首段先开 DMA 再开定时器；续传不停止 TIM7。 */
    DMA_Cmd(PU3_DMA_CHANNEL, ENABLE);
    if (!timer_running)
    {
        /* 首段最后启动时基；chunk 续传保持 TIM7 连续计数。 */
        TIM_Cmd(PU3_TIMER, ENABLE);
    }
}

/** 初始化 PU3 的 PB13、TIM7 和 DMA2 通道4。 */
void PU3_Stepper_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    DMA_InitTypeDef dma;
    NVIC_InitTypeDef nvic;
    uint32_t index;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
    gpio.GPIO_Pin = PU3_STEP_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(PU3_STEP_GPIO, &gpio);
    GPIO_ResetBits(PU3_STEP_GPIO, PU3_STEP_PIN);
    for (index = 0u; index < PU3_CHUNK_EDGES; index++)
        s_pu3_edges[index] = (index & 1u) ? ((uint32_t)PU3_STEP_PIN << 16) : PU3_STEP_PIN;
    timer.TIM_Prescaler = 0u;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_Period = 999u;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_RepetitionCounter = 0u;
    TIM_TimeBaseInit(PU3_TIMER, &timer);
    TIM_DMACmd(PU3_TIMER, TIM_DMA_Update, ENABLE);
    DMA_DeInit(PU3_DMA_CHANNEL);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&GPIOB->BSRR;
    dma.DMA_MemoryBaseAddr = (uint32_t)s_pu3_edges;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = PU3_CHUNK_EDGES;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(PU3_DMA_CHANNEL, &dma);
    DMA_ITConfig(PU3_DMA_CHANNEL, DMA_IT_TC, ENABLE);
    nvic.NVIC_IRQChannel = PU3_DMA_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1u;
    nvic.NVIC_IRQChannelSubPriority = 3u;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    PU3_Stepper_Stop();
}

/** 启动 PU3 梯形加减速运动；运行中或零步数时返回 0。 */
uint8_t PU3_Stepper_Start(uint32_t steps, uint8_t direction, uint32_t start_frequency, uint32_t maximum_frequency, uint32_t acceleration)
{
    uint32_t chunk;
    if (steps == 0u || s_pu3.running)
        return 0u;
    if (start_frequency < 1u)
        start_frequency = 1u;
    if (maximum_frequency < start_frequency)
        maximum_frequency = start_frequency;
    if (acceleration < 1u)
        acceleration = 1u;
    PU3_Stepper_Stop();
    s_pu3.direction = direction ? 1u : 0u;
    PU3_SetDirection(s_pu3.direction);
    s_pu3.total_steps = steps;
    s_pu3.remaining_steps = steps;
    s_pu3.completed_steps = 0u;
    s_pu3.start_speed = start_frequency;
    s_pu3.target_speed = maximum_frequency;
    s_pu3.acceleration = acceleration;
    PU3_BuildTrapProfile(steps, start_frequency, maximum_frequency,
                         acceleration);
    s_pu3.current_speed = PU3_GetTrapFrequency(0u);
    s_pu3.accel_state = PU3_ACCEL_UP;
    GPIO_ResetBits(PU3_STEP_GPIO, PU3_STEP_PIN);
    PU3_SetEdgeFrequency(s_pu3.current_speed * 2u);
    s_pu3.running = 1u;
    chunk = (steps > PU3_CHUNK_STEPS) ? PU3_CHUNK_STEPS : steps;
    PU3_LoadChunk(chunk);
    return 1u;
}

/** 启动 Z 轴已知距离 Home，最后 slow_zone_steps 保持低速等待 S3。 */
uint8_t PU3_Stepper_StartHomeApproach(uint32_t steps, uint8_t direction,
                                      uint32_t start_frequency,
                                      uint32_t fast_frequency,
                                      uint32_t slow_frequency,
                                      uint32_t slow_zone_steps,
                                      uint32_t acceleration)
{
    uint32_t chunk;
    if ((steps == 0u) || s_pu3.running) return 0u;
    if (start_frequency < 1u) start_frequency = 1u;
    if (slow_frequency < start_frequency) slow_frequency = start_frequency;
    if (fast_frequency < slow_frequency) fast_frequency = slow_frequency;
    if (acceleration < 1u) acceleration = 1u;
    PU3_Stepper_Stop();
    s_pu3.direction = direction ? 1u : 0u;
    PU3_SetDirection(s_pu3.direction);
    s_pu3.total_steps = steps;
    s_pu3.remaining_steps = steps;
    s_pu3.completed_steps = 0u;
    s_pu3.start_speed = start_frequency;
    s_pu3.target_speed = fast_frequency;
    s_pu3.acceleration = acceleration;
    PU3_BuildHomeApproachProfile(steps, start_frequency, fast_frequency,
                                 slow_frequency, slow_zone_steps, acceleration);
    s_pu3.current_speed = PU3_GetTrapFrequency(0u);
    s_pu3.accel_state = PU3_ACCEL_UP;
    GPIO_ResetBits(PU3_STEP_GPIO, PU3_STEP_PIN);
    PU3_SetEdgeFrequency(s_pu3.current_speed * 2u);
    s_pu3.running = 1u;
    chunk = (steps > PU3_CHUNK_STEPS) ? PU3_CHUNK_STEPS : steps;
    PU3_LoadChunk(chunk);
    return 1u;
}

/** 停止 PU3，并将 PB13 保持为低电平。 */
void PU3_Stepper_Stop(void)
{
    TIM_Cmd(PU3_TIMER, DISABLE);
    DMA_Cmd(PU3_DMA_CHANNEL, DISABLE);
    DMA_ClearITPendingBit(DMA2_IT_TC4);
    TIM_ClearFlag(PU3_TIMER, TIM_FLAG_Update);
    GPIO_ResetBits(PU3_STEP_GPIO, PU3_STEP_PIN);
    s_pu3.running = 0u;
    s_pu3.accel_state = PU3_ACCEL_IDLE;
}

/** 查询 PU3 是否正在输出 DMA 步进脉冲。 */
uint8_t PU3_Stepper_IsRunning(void) { return s_pu3.running; }

/** 获取 PU3 已经完成的整段步数。 */
uint32_t PU3_Stepper_GetCompletedSteps(void) { return s_pu3.completed_steps; }

/** 获取 PU3 尚未完成的步数。 */
uint32_t PU3_Stepper_GetRemainingSteps(void) { return s_pu3.remaining_steps; }

void DMA2_Channel4_5_IRQHandler(void)
{
    uint32_t chunk;
    if (DMA_GetITStatus(DMA2_IT_TC4) == RESET)
        return;
    DMA_ClearITPendingBit(DMA2_IT_TC4);
    s_pu3.completed_steps += s_pu3.current_chunk_steps;
    s_pu3.remaining_steps -= s_pu3.current_chunk_steps;
    if (s_pu3.remaining_steps == 0u)
    {
        PU3_Stepper_Stop();
        RobotArmProtocol_OnPhaseAxisDmaCompleted(ROBOT_AXIS_Z);
        return;
    }
    /* 每完成实际装载的 32 step（末段可能更短）后，按剩余距离刷新梯形速度。 */
    s_pu3.current_speed = PU3_GetTrapFrequency(s_pu3.completed_steps);
    if (s_pu3.completed_steps < s_pu3_trap.accel_steps)
    {
        s_pu3.accel_state = PU3_ACCEL_UP;
    }
    else if (s_pu3.remaining_steps <= s_pu3_trap.decel_steps)
    {
        s_pu3.accel_state = PU3_ACCEL_DOWN;
    }
    else
    {
        s_pu3.accel_state = PU3_ACCEL_CRUISE;
    }
    PU3_SetEdgeFrequency(s_pu3.current_speed * 2u);
    chunk = (s_pu3.remaining_steps > PU3_CHUNK_STEPS) ? PU3_CHUNK_STEPS : s_pu3.remaining_steps;
    PU3_LoadChunk(chunk);
}
