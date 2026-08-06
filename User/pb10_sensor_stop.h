#ifndef __PB10_SENSOR_STOP_H
#define __PB10_SENSOR_STOP_H

#include <stdint.h>

/**
 * @brief 检查 PB10 步进电机绑定的停止传感器。
 * @param sensor_data 当前 2 字节 HC165 输入数据。
 */
void Protocol_CheckPb10StopSensor(const uint8_t *sensor_data);

#endif
