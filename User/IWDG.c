#include "IWDG.h"

void IWDG_Init(void)
{
    // 启用写访问
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    // 设置预分频值（可选项如下：4, 8, 16, 32, 64, 128, 256）
    IWDG_SetPrescaler(IWDG_Prescaler_64);

    // 设置重装载值（0~0xFFF）: 重载值越大，超时时间越长
    // 例如：40kHz / 64 = 625Hz → 每1.6ms计数一次 → 625 * 1.6ms = 1秒
    IWDG_SetReload(625);

    // 重载计数器（相当于立即喂一次狗）
    IWDG_ReloadCounter();

    // 启动看门狗
    IWDG_Enable();
}
