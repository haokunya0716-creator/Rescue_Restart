#ifndef INTELLIGENT_RESCUE_APP_ZONE_TEST_H
#define INTELLIGENT_RESCUE_APP_ZONE_TEST_H

#include "App_Task.h"
#include "rescue_usart2.h"

/* 在原硬件初始化之后，while(1)开头调用：
 *     if (App_Zone_Test(APP_TASK_TEAM_RED)) { continue; }
 * 注释这一整行并重新编译即可恢复正式按键流程；蓝方改传APP_TASK_TEAM_BLUE。
 * 测试只自动启动一次，共用App_Task_Proc，不另设控制或参数。
 * 到位/超时后仍返回1接管主循环，不接受按键、不重复启动。
 */
static inline uint8_t App_Zone_Test(uint8_t team)
{
    static uint8_t started;
    if(!started){
        started = 1;
        App_Task_StartZoneTest(team);
    }
    Rescue_Usart2_Process();
    App_Task_Proc();
    Rescue_Usart2_Debug_Proc();
    return 1;
}

#endif
