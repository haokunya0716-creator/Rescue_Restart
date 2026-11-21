// 复检多件/非绿色后的简单撞散动作：先前推，再抬框并旋转一圈。
#include "app_task_scatter.h"
#include "app_motor.h"
#include "app_usart.h"

typedef enum {
    SCATTER_PUSH,
    SCATTER_SPIN
} App_Task_ScatterState;

static App_Task_ScatterState scatter_state;
static float push_start;
static float spin_last_yaw;
static float spin_degrees;

void App_Task_Scatter_Start(void)
{
    App_Update_Data();
    push_start = (position_l + position_r) * 0.5f;
    scatter_state = SCATTER_PUSH;
    App_Task_Move_Speed(APP_TASK_SCATTER_PUSH_SPEED, APP_TASK_SCATTER_PUSH_SPEED);
    if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[SCATTER] phase=push goal=%.1f speed=%.1f\r\n",
        APP_TASK_SCATTER_PUSH_CM, APP_TASK_SCATTER_PUSH_SPEED); }
}

App_Task_Result App_Task_Scatter_Proc(uint8_t *cargo_pending)
{
    App_Update_Data();
    if(scatter_state == SCATTER_PUSH){
        float distance = (position_l + position_r) * 0.5f - push_start;
        if(distance < APP_TASK_SCATTER_PUSH_CM){
            App_Task_Move_Speed(APP_TASK_SCATTER_PUSH_SPEED, APP_TASK_SCATTER_PUSH_SPEED);
            return APP_STEP_RUNNING;
        }
        App_Task_Frame_Set(0); // 先走够距离，再抬框；同次开始旋转，不等待舵机。
        *cargo_pending = 0;
        spin_last_yaw = angle_yaw;
        spin_degrees = 0.0f;
        scatter_state = SCATTER_SPIN;
        App_Task_Move_Speed(APP_TASK_SCATTER_SPIN_SPEED, -APP_TASK_SCATTER_SPIN_SPEED);
        if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[SCATTER] phase=spin goal=%.1f speed=%.1f\r\n",
            APP_TASK_SCATTER_SPIN_DEG, APP_TASK_SCATTER_SPIN_SPEED); }
        return APP_STEP_RUNNING;
    }

    float delta = angle_yaw - spin_last_yaw;
    // IMU输出在±180度回绕；换算成两次采样之间实际连续转过的小角度。
    if(delta > 180.0f){ delta -= 360.0f; }
    else if(delta < -180.0f){ delta += 360.0f; }
    spin_degrees += delta;
    spin_last_yaw = angle_yaw;
    if(spin_degrees >= APP_TASK_SCATTER_SPIN_DEG || spin_degrees <= -APP_TASK_SCATTER_SPIN_DEG){
        if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[SCATTER] done degrees=%.1f next=search\r\n", spin_degrees); }
        return APP_STEP_DONE; // 保持旋转，任务层本次立即接持续搜索。
    }
    App_Task_Move_Speed(APP_TASK_SCATTER_SPIN_SPEED, -APP_TASK_SCATTER_SPIN_SPEED);
    return APP_STEP_RUNNING;
}
