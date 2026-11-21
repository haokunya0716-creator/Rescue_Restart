#ifndef INTELLIGENT_RESCUE_APP_ZONE_XY_HEADING_TEST_H
#define INTELLIGENT_RESCUE_APP_ZONE_XY_HEADING_TEST_H

#include "main.h"

/* 仅本测试的侧移初始距离；XY参数全部引用app_task_config.h。 */
#define APP_ZONE_XY_HEADING_SHIFT_CM 5.0f

typedef enum {
    APP_ZONE_XY_PREPARE = 0,
    APP_ZONE_XY_SEARCH,
    APP_ZONE_XY_ALIGN,
    APP_ZONE_XY_ALIGN_LOST,
    APP_ZONE_XY_FOLLOW,
    APP_ZONE_XY_HOLD,
    APP_ZONE_XY_TURN_OUT,
    APP_ZONE_XY_SHIFT,
    APP_ZONE_XY_TURN_BACK,
    APP_ZONE_XY_DONE,
    APP_ZONE_XY_TIMEOUT
} App_ZoneXYHeading_Phase;

/* Ozone观察值；fixed_yaw是最终目标，motion_yaw是中间动作目标。 */
typedef struct {
    uint32_t phase, valid, frame_id, view_id, elapsed_ms;
    float x, y, x_error, y_error;
    float yaw, fixed_yaw, yaw_error, motion_yaw;
    float shift_cm, shifted_cm, base_speed, left_speed, right_speed;
} App_ZoneXYHeading_Watch;

extern volatile App_ZoneXYHeading_Watch zone_xy_heading_watch;

/* while(1)中独占调用，始终返回1：
 * if (App_ZoneXYHeading_Test(APP_TASK_TEAM_RED, 0.0f)) { continue; }
 * fixed_yaw_deg填场地标定后的绝对航向（度），不是相对本次开机角度。
 * 队伍和最终航向仅在首次调用锁存，不自动清零IMU；到位/180秒后保持停车。
 * 与App_ZoneY_Test/App_Zone_Test只能启用一个；再次测试需复位。
 */
uint8_t App_ZoneXYHeading_Test(uint8_t team, float fixed_yaw_deg);

#endif
