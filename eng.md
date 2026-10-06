ceramic capacitor 陶瓷电容器
internal 内部
Positive line of the USB data line pair.  USB数据线对的正极线路。
D+/D- based USB host/charging port detection. The detection includes data contact detection (DCD), primary
and secondary detection in BC1.2, and Adjustable high voltage adapte   基于D+/D-的USB主机/充电端口检测。检测包括数据接触检测（DCD）、BC1.2中的主检测和次检测，以及可调高压适配器。


Open drain charge status output to indicate various charger operation.
打开排水充电状态输出，以指示各种充电器操作。
Connect to the pull up rail via 10-kΩ resistor. LOW indicates charge in progress. HIGH indicates charge
complete or charge disabled. When any fault condition occurs, STAT pin blinks in 1 Hz.
The STAT pin function can be disabled when STAT_DIS bit is set  通过10 kΩ电阻连接至上拉电路，用于输出充电状态指示。低电平表示正在充电，高电平表示充电完成或充电已禁用。当发生任何故障时，STAT引脚将以1 Hz频率闪烁。若设置STAT_DIS位，则可禁用STAT引脚功能。
//has fixed batch directions 已固定批量方向