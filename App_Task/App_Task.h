//
// Created by gaoxu on 2026/11/21.
//

#ifndef INTELLIGENT_RESCUE_APP_TASK_H
#define INTELLIGENT_RESCUE_APP_TASK_H

#include "rescue_usart2.h"

//这个文件用来存放一些任务相关的宏定义和函数,以及逻辑代码

//任务所处的状态：
typedef enum
{
    TASK_WAIT_START,  // 等待按键启动
    TASK_TURN_45,     // 右转 45°
    TASK_FORWARD,     // 定时前进
    TASK_FIND_FIVE_FIRST, //第一次寻找绿色物块
    TASK_FIND_FIVE,    //寻找物块并框住
    TASK_CHECK_FIRST,  //第一次复检
    TASK_CHECK,        //后续复检
    TASK_FIND_SAFE_SUPPLIES, //寻找物资放置区
    TASK_FIND_SAFE_INJURED, //寻找伤员放置区
    TASK_END,  //成功找到物块并放入安全区
    TASK_MESS_UP, // 初始时打乱物块
    TASK_MESS_UP_ALL //后续打乱物块
} TaskState;

//寻找物块分的逻辑，分别为第一次寻找和后续寻找
typedef enum {
    FIND_FIRST_GREEN,  // 首趟，选择横向最接近对准位置的绿块
    FIND_NORMAL        // 后续，按当前比赛规则选择允许转运的目标
} FindMode;

//寻找物块时的步骤
typedef enum {
    FIND_ROTATE_SEARCH,  // 原地旋转寻找
    FIND_ALIGN,          // 找到后原地对准
    FIND_APPROACH,       // 边前进边纠偏
    FIND_LOWER_FRAME     // 靠近到位后落框
} FindState;


#define TASK_FORWARD_TIME 1.0f * 1e3  //向前直行时间 ，单位->ms
#define SERVO1_FALL 20.0f //舵机落下的角度
#define SERVO1_UP 130.0f  //舵机抬起的角度

#define SERVO2_FIND 48.0f  //摄像头正常抬起的角度
#define SERVO2_CHECK 20.0f  //摄像头复检的角度

#define CHECK_MIN_X 420    //复检时前框内x，y的范围
#define CHECK_MAX_X 840    //复检时前框内x，y的范围
#define CHECK_MIN_Y 220   //复检时前框内x，y的范围
#define CHECK_MAX_Y 300   //复检时前框内x，y的范围

void App_Task_Init(void);//初始化各种参数的代码
void App_Task_Pro(void);//任务进行

uint8_t App_Find(FindMode mode);  // 首次搜索返回 1 表示已停车并下发落框角度

// 从当前帧选一块绿块，返回该条目的指针。无绿块时返回 NULL。
const AppVisionObject *App_Select_First_Green(const AppVisionData *vision);


uint8_t App_Find_First_Pro(void);//第一次寻找，0：仍在运行，1：已停车并下发落框角度

uint8_t App_Find_Pro(void);//后续寻找 0：仍在运行，1：已接近到位并停车
#endif //INTELLIGENT_RESCUE_APP_TASK_H
