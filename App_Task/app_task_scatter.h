//
// Created by gaoxu on 2026/10/20.
//

#ifndef INTELLIGENT_RESCUE_APP_TASK_SCATTER_H // 检查头文件保护宏，防止重复包含时反复展开声明。
#define INTELLIGENT_RESCUE_APP_TASK_SCATTER_H // 定义头文件保护宏，后续重复包含时跳过本文件。

#include "app_task_move.h" // 引入运动动作及统一结果类型。

// @简介：立即按当前前框状态开始前推；只记录编码器起点，不检查图像、数量或参数。
void App_Task_Scatter_Start(void);

// @简介：非阻塞执行“平均行程前推→抬框同时旋转→累计转满一圈”。
// @说明：抬框时将调用方的cargo_pending清零；DONE时保持旋转，任务层同次切换持续搜索。
App_Task_Result App_Task_Scatter_Proc(uint8_t *cargo_pending);

#endif // 结束本头文件的重复包含保护。
