//
// Created by gaoxu on 2026/10/17.
//

#ifndef INTELLIGENT_RESCUE_APP_LINE_PID_H
#define INTELLIGENT_RESCUE_APP_LINE_PID_H

#include "main.h"

#define X_CENTER 625.0f//视野中，中心坐标对应的x
#define Y_CENTER 580.0f//视野中，物块在框中对应的y

#define ERR_MAX_X 50.0f // 对准时 X 允许的最大偏差，单位为像素
#define ERR_MAX_Y 37.5f // 靠近时 Y 允许的最大偏差，单位为像素

void App_PID_Init(void);

/* 清除视觉 PID 的采样历史，保留目标 X、系数和限幅。 */
void App_LinePID_Reset(void);

/* 初始化后传入有效的中心 X 坐标。0 表示继续调用，1 表示对准并停车。 */
uint8_t App_LinePID_Pro(uint16_t object_cx);

/* 原地对准后传入同一物块的中心 X、Y。两轴到位并停车时返回 1。 */
uint8_t App_Approach_Pro(uint16_t object_cx, uint16_t object_cy);

#endif //INTELLIGENT_RESCUE_APP_LINE_PID_H
