//
// Created by gaoxu on 2026/11/21.
//

#include "main.h"
#include "App_Task.h"

#include <math.h>
#include <stddef.h>

#include "app_button.h"
#include "app_line_pid.h"
#include "app_motor.h"
#include "app_servo.h"
#include "app_speed.h"
#include "app_usart.h"
#include "drv8701.h"
#include "task.h"
#include "usart.h"

static TaskState task_state = TASK_WAIT_START;//初始默认的状态,等待按键按下

static uint32_t forward_start_ms; // 进入前进阶段时记录一次开始时间。

void App_Task_Init(void) {
    /*
  * 所有定时器和 GPIO 初始化完成后，启动 DRV8701E 的两路 EN PWM。
  * Motor_Init() 会先写入停止值，避免上电初始化过程中产生
  * 意外的电机动作。
  */
    Motor_Init();
    Motor_Cmd(1);

    /* 初始化运动控制。基础底座上电保持停车。 */
    App_Speed_Init();
    App_Motor_Init();
    App_PID_Init();

    App_Servo_Init();
    App_Button_Init();

}

void App_Task_Pro(void) {
    switch (task_state) {
        case TASK_WAIT_START:
            //检测按键是否按下
            if (key2_flag == 1) {
                key2_flag = 0;
                App_Servo_SetAngle(&servo_1, 120.0f);
                task_state = TASK_TURN_45;//接下来右转

                Set_Angle_SP(-45.0f);//设置目标角度
            }
            break;
        case TASK_TURN_45 :
            //进行角度闭环，完成转向逻辑后进行下一步直行
            if (App_Angle_Pro() == 1) {
                forward_start_ms = HAL_GetTick();//计时，方便下一步定时任务
                task_state = TASK_FORWARD;
            }
            break;
        case TASK_FORWARD:
            //直行一段时间，然后开始寻找物块
            if (HAL_GetTick() - forward_start_ms >= TASK_FORWARD_TIME) {
                App_Speed_Set(0.0f, 0.0f);
                task_state = TASK_MESS_UP;
            } else {
                App_Speed_Set(25.0f, 25.0f);
                App_Speed_Pro();
            }
            break;
        case TASK_MESS_UP:
            //一开始的打乱物块逻辑，先空白、
            task_state = TASK_FIND_FIVE_FIRST;

            break;
        case TASK_FIND_FIVE_FIRST:
            //开始寻找小物块，找不到就一直原地旋转，找到了就对准
            // 首次靠近停车并下发落框角度后，进入寻找安全区状态。
            if (App_Find(FIND_FIRST_GREEN) == 1) {
                //第一次捕捉成功之后就进入复检
                task_state = TASK_CHECK_FIRST;
            }
            break;
        case TASK_CHECK_FIRST://第一次复检

            break;

        case TASK_CHECK://后续捕捉到物块复检

            break;
        case TASK_MESS_UP_ALL:

            break;

        case TASK_FIND_SAFE_SUPPLIES: //寻找物资放置区

            break;
        case TASK_FIND_SAFE_INJURED: //寻找伤员放置区

            break;



    }

}

// 首次搜索返回 1 表示已接近停车并下发落框角度。
uint8_t App_Find(FindMode mode) {
        switch (mode) {
            case FIND_FIRST_GREEN://第一次只寻找绿色单个物块
                return App_Find_First_Pro();

            case FIND_NORMAL://后续寻找
                return App_Find_Pro();
        }

}
const AppVisionObject *App_Select_First_Green(const AppVisionData *vision)
{
    const AppVisionObject *selected = NULL;
    float best_error = 0.0f;

    for (uint8_t i = 0U; i < vision->object_count; ++i) {
        const AppVisionObject *object = &vision->objects[i];
        if (object->type != RESCUE_OBJECT_SUPPLY) {
            continue; // 首趟只选普通绿块，框内数量留到复检判断。
        }

        // 选离对准目标 X_CENTER 最近的绿块。同距时保留前一块。
        const float error = fabsf(object->cx - X_CENTER);
        if (selected == NULL || error < best_error) {
            selected = object;
            best_error = error;
        }
    }

    return selected; // 没有绿块时返回 NULL。
}

uint8_t App_Find_First_Pro(void) {
    static FindState state = FIND_ROTATE_SEARCH;
    // 每轮筛选当前帧，把选中绿块的坐标交给当前对准阶段。
    const AppVisionObject *green = App_Select_First_Green(&app_vision);

    if (green == NULL) {
        // 丢块后恢复旋转搜索，重见时重新对准 X。
        state = FIND_ROTATE_SEARCH;
        App_Speed_Set(20.0f, -20.0f);
        App_Speed_Pro();
        return 0U;
    }

    if (state == FIND_ROTATE_SEARCH) {
        state = FIND_ALIGN;
    }

    if (state == FIND_ALIGN) {
        if (App_LinePID_Pro(green->cx) == 1U) {
            state = FIND_APPROACH;
        }
        return 0U;
    }

    if (App_Approach_Pro(green->cx, green->cy) == 1U) {
        App_Servo_SetAngle(&servo_1, SERVO1_FALL);
        return 1U;
    }
    return 0U;
}
uint8_t App_Find_Pro(void) {

}
