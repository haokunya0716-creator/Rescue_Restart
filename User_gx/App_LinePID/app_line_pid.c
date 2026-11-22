//
// Created by gaoxu on 2026/10/17.
//

#include "app_line_pid.h"
#include "app_speed.h"
#include "pid.h"
#include "task.h"

#include <math.h>

static PID_TypeDef line_pid; /**< 保存 PID 配置、目标 X 和采样历史。 */

void App_PID_Init(void) {
    PID_Init(&line_pid, 0.1f, 0, 0);
    PID_ChangeSP(&line_pid,X_CENTER);
    PID_LimitConfig(&line_pid,60.0f,-60.0f);
}

void App_LinePID_Reset(void)
{
    PID_Reset(&line_pid);
}
uint8_t App_LinePID_Pro(uint16_t object_cx) {
    PERIODIC_START(LINE_PID, 15)
        if (fabsf(line_pid.SP - object_cx) <= ERR_MAX_X) {
            App_Speed_Set(0.0f, 0.0f);
            App_LinePID_Reset();
            return 1U;
        }

        float turn_speed = -PID_Compute(&line_pid, object_cx);
        App_Speed_Set(turn_speed, -turn_speed);
    PERIODIC_END

    App_Speed_Pro();
    return 0U;
}

uint8_t App_Approach_Pro(uint16_t object_cx, uint16_t object_cy) {
    PERIODIC_START(APPROACH_PID, 15)
        float y_error = Y_CENTER - object_cy;
        if (fabsf(line_pid.SP - object_cx) <= ERR_MAX_X && fabsf(y_error) <= ERR_MAX_Y) {
            App_Speed_Set(0.0f, 0.0f);
            App_LinePID_Reset();
            return 1U;
        }

        float speed = fabsf(y_error) > ERR_MAX_Y
                      ? fmaxf(-25.0f, fminf(0.1f * y_error, 25.0f))
                      : 0.0f;
        float turn_speed = 0.0f;
        if (fabsf(line_pid.SP - object_cx) <= ERR_MAX_X) {
            App_LinePID_Reset();
        } else {
            turn_speed = -PID_Compute(&line_pid, object_cx);
        }
        if (speed != 0.0f) {
            turn_speed = fmaxf(-0.3f * fabsf(speed), fminf(turn_speed, 0.3f * fabsf(speed)));
        }

        App_Speed_Set(speed + turn_speed, speed - turn_speed);
    PERIODIC_END

    App_Speed_Pro();
    return 0U;
}
