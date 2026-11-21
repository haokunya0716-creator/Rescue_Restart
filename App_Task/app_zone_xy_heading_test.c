#include "app_zone_xy_heading_test.h"
#include "app_task_move.h"
#include "app_line_pid.h"
#include "app_motor.h"
#include "rescue_usart2.h"
#include <math.h>

volatile App_ZoneXYHeading_Watch zone_xy_heading_watch;

static uint8_t started, zone_type, frame_seen, view_sent;
static uint16_t view_id, last_frame;
static uint32_t start_time, view_time, last_capture, missing_time;
static float fixed_yaw, base_speed, return_yaw, motion_yaw;
static float shift_cm, shift_start, previous_yaw_error;
static App_ZoneXYHeading_Phase phase;

static float ZoneXY_Wrap(float angle)
{
    angle = fmodf(angle + 180.0f, 360.0f);
    if(angle < 0.0f){ angle += 360.0f; }
    return angle - 180.0f;
}

static void ZoneXY_Command(float left, float right)
{
    zone_xy_heading_watch.left_speed = left;
    zone_xy_heading_watch.right_speed = right;
    App_Task_Move_Speed(left, right);
}

/* 仅落框等待、首次X丢失200ms、最终完成和180秒结束调用。 */
static void ZoneXY_Stop(void)
{
    zone_xy_heading_watch.left_speed = zone_xy_heading_watch.right_speed = 0.0f;
    App_Task_Move_Stop();
}

static void ZoneXY_SetPhase(App_ZoneXYHeading_Phase next)
{
    phase = next;
    zone_xy_heading_watch.phase = next;
}

static void ZoneXY_SetHeading(float target)
{
    motion_yaw = ZoneXY_Wrap(target);
    zone_xy_heading_watch.motion_yaw = motion_yaw;
    Set_Angle_Target(motion_yaw); // 复用正式航向PID，只设置目标，不清零轮速。
}

static void ZoneXY_HeadingDrive(float speed)
{
    App_Heading_Pro(speed);
    zone_xy_heading_watch.left_speed = speed - motor_turn_speed;
    zone_xy_heading_watch.right_speed = speed + motor_turn_speed;
}

/* 在调用角度控制前判到位，避免原App_Angle_Pro_Limited的到位零轮速出口。 */
static uint8_t ZoneXY_TurnDone(void)
{
    float error = ZoneXY_Wrap(motion_yaw - angle_yaw);
    if(fabsf(error) <= APP_TASK_TURN_TOL_DEG){ return 1U; }
    ZoneXY_HeadingDrive(0.0f);
    if(fabsf(motor_turn_speed) < APP_TASK_TURN_MIN_SPEED){
        float turn = error > 0.0f ? APP_TASK_TURN_MIN_SPEED : -APP_TASK_TURN_MIN_SPEED;
        ZoneXY_Command(-turn, turn);
    }
    return 0U;
}

static void ZoneXY_NewView(void)
{
    if(++view_id == 0U){ ++view_id; } // 协议视角编号0保留不用。
    view_sent = 0U;
    zone_xy_heading_watch.view_id = view_id;
}

static void ZoneXY_Search(uint8_t new_view)
{
    if(phase != APP_ZONE_XY_SEARCH){ App_LinePID_Reset(); }
    if(new_view){ ZoneXY_NewView(); }
    ZoneXY_SetPhase(APP_ZONE_XY_SEARCH);
    base_speed = zone_xy_heading_watch.base_speed = 0.0f;
    ZoneXY_Command(APP_TASK_ZONE_SPIN_SPEED, -APP_TASK_ZONE_SPIN_SPEED);
}

/* 与正式整区相同的X差速计算、死区及前进/倒车差速限幅。 */
static void ZoneXY_VisualDrive(float x, float speed)
{
    float turn = App_LinePID_ComputeTarget(x, APP_TASK_ZONE_AIM_X,
                                         APP_TASK_ZONE_X_TOL, APP_TASK_ZONE_X_TOL);
    if(speed != 0.0f){
        float limit = fabsf(speed) * LINE_PID_FORWARD_RATIO;
        if(turn > limit){ turn = limit; }
        if(turn < -limit){ turn = -limit; }
    }
    base_speed = zone_xy_heading_watch.base_speed = speed;
    ZoneXY_Command(speed + turn, speed - turn);
}

/* 返回1表示已进入航向调整，可在同次调用立即继续，不插入停车或等图。 */
static uint8_t ZoneXY_Follow(float x, float y)
{
    float error = APP_TASK_ZONE_AIM_Y - y;
    if(fabsf(error) <= APP_TASK_ZONE_Y_TOL){
        base_speed = zone_xy_heading_watch.base_speed = 0.0f;
        if(fabsf(x - APP_TASK_ZONE_AIM_X) > APP_TASK_ZONE_X_TOL){
            ZoneXY_VisualDrive(x, 0.0f);
            return 0U;
        }
        float yaw_error = ZoneXY_Wrap(fixed_yaw - angle_yaw);
        if(fabsf(yaw_error) <= APP_TASK_TURN_TOL_DEG){
            ZoneXY_SetPhase(APP_ZONE_XY_DONE);
            ZoneXY_Stop();
            return 0U;
        }
        // 实际跨过目标时减半侧移步长；±180度回绕不算越过目标。
        if(previous_yaw_error * yaw_error < 0.0f
           && fabsf(previous_yaw_error) + fabsf(yaw_error) < 180.0f){ shift_cm *= 0.5f; }
        previous_yaw_error = yaw_error;
        zone_xy_heading_watch.shift_cm = shift_cm;
        return_yaw = angle_yaw;
        ZoneXY_SetHeading(return_yaw - (yaw_error > 0.0f ? 90.0f : -90.0f));
        ZoneXY_SetPhase(APP_ZONE_XY_TURN_OUT);
        return 1U;
    }

    // 保留原Y比例控制和速度宏，取消200ms/3帧起步及额外Y滞回。
    float speed = fabsf(error) * LINE_PID_Y_KP;
    if(speed < LINE_PID_Y_MIN_SPEED){ speed = LINE_PID_Y_MIN_SPEED; }
    if(error > 0.0f && speed > LINE_PID_Y_MAX_FORWARD){ speed = LINE_PID_Y_MAX_FORWARD; }
    if(error < 0.0f && speed > LINE_PID_Y_MAX_REVERSE){ speed = LINE_PID_Y_MAX_REVERSE; }
    ZoneXY_VisualDrive(x, error > 0.0f ? speed : -speed);
    return 0U;
}

uint8_t App_ZoneXYHeading_Test(uint8_t team, float fixed_yaw_deg)
{
    uint32_t now = HAL_GetTick();
    if(!started){
        started = 1U;
        start_time = now;
        zone_type = team == APP_TASK_TEAM_BLUE ? RESCUE_OBJECT_BLUE_SAFE_ZONE : RESCUE_OBJECT_RED_SAFE_ZONE;
        fixed_yaw = ZoneXY_Wrap(fixed_yaw_deg);
        zone_xy_heading_watch.fixed_yaw = fixed_yaw;
        shift_cm = zone_xy_heading_watch.shift_cm = APP_ZONE_XY_HEADING_SHIFT_CM;
        App_Task_Frame_Set(1U);
        App_Task_Camera_Set(0U);
        App_LinePID_Reset();
        ZoneXY_NewView();
        ZoneXY_SetPhase(APP_ZONE_XY_PREPARE);
    }
    Rescue_Usart2_Process();
    Rescue_Usart2_Debug_Proc();
    now = HAL_GetTick();
    zone_xy_heading_watch.elapsed_ms = (uint32_t)(now - start_time);
    App_Update_Data();
    zone_xy_heading_watch.yaw = angle_yaw;
    zone_xy_heading_watch.yaw_error = ZoneXY_Wrap(fixed_yaw - angle_yaw);

    if((uint32_t)(now - start_time) >= APP_TASK_ROUND_TIME_MS){
        ZoneXY_SetPhase(APP_ZONE_XY_TIMEOUT);
        ZoneXY_Stop();
        return 1U;
    }
    if(phase == APP_ZONE_XY_DONE || phase == APP_ZONE_XY_TIMEOUT){ ZoneXY_Stop(); return 1U; }
    if(phase == APP_ZONE_XY_PREPARE){
        if((uint32_t)(now - start_time) < APP_TASK_SERVO_WAIT_MS){ ZoneXY_Stop(); return 1U; }
        ZoneXY_Search(0U);
    }

    if(!view_sent || (uint32_t)(now - view_time) >= APP_TASK_VIEW_RETRY_MS){
        view_sent = Rescue_Usart2_SendViewContext(view_id, 0);
        view_time = now;
    }
    RescueScene scene;
    if(!view_sent || !Rescue_Usart2_GetScene(&scene) || scene.view_id != view_id){
        zone_xy_heading_watch.valid = 0U;
        // F5/F6：通信无可用图或视角不符直接旋转；不反复换view_id阻止相机追上。
        ZoneXY_Search(0U);
        return 1U;
    }
    uint8_t fresh = !frame_seen || scene.frame_id != last_frame || scene.capture_ms != last_capture;
    int index = -1;
    float x = 0.0f, y = 0.0f;
    if(fresh){
        frame_seen = 1U;
        last_frame = scene.frame_id;
        last_capture = scene.capture_ms;
        zone_xy_heading_watch.frame_id = scene.frame_id;
        for(uint8_t i = 0U; i < scene.object_count; ++i){
            if(scene.objects[i].type == zone_type){ index = i; break; }
        }
        zone_xy_heading_watch.valid = index >= 0;
        if(index >= 0){
            x = scene.objects[index].cx * APP_TASK_IMAGE_W / scene.image_width;
            y = scene.objects[index].cy * APP_TASK_IMAGE_H / scene.image_height;
            zone_xy_heading_watch.x = x;
            zone_xy_heading_watch.y = y;
            zone_xy_heading_watch.x_error = APP_TASK_ZONE_AIM_X - x;
            zone_xy_heading_watch.y_error = APP_TASK_ZONE_AIM_Y - y;
        }
    }

    for(;;){ // 完成一个动作立即处理下一阶段；运动阶段下发命令后返回主循环。
        switch(phase){
        case APP_ZONE_XY_SEARCH:
            if(index < 0){ ZoneXY_Search(0U); return 1U; }
            App_LinePID_Reset();
            ZoneXY_SetPhase(APP_ZONE_XY_ALIGN);
            continue;
        case APP_ZONE_XY_ALIGN_LOST:
            if(index >= 0){
                App_LinePID_Reset();
                ZoneXY_SetPhase(APP_ZONE_XY_ALIGN);
                continue;
            }
            if((uint32_t)(now - missing_time) >= APP_TASK_ZONE_ALIGN_LOST_MS){ ZoneXY_Search(1U); }
            else { ZoneXY_Stop(); }
            return 1U;
        case APP_ZONE_XY_ALIGN:
            if(!fresh){ App_Task_Move_Proc(); return 1U; }
            if(index < 0){
                missing_time = now;
                ZoneXY_SetPhase(APP_ZONE_XY_ALIGN_LOST);
                ZoneXY_Stop();
                return 1U;
            }
            if(fabsf(x - APP_TASK_ZONE_AIM_X) <= APP_TASK_ZONE_X_TOL){
                App_LinePID_Reset();
                ZoneXY_SetPhase(APP_ZONE_XY_FOLLOW);
                continue;
            }
            ZoneXY_VisualDrive(x, 0.0f);
            return 1U;
        case APP_ZONE_XY_HOLD:
            if(index >= 0){
                App_LinePID_Reset();
                ZoneXY_SetPhase(APP_ZONE_XY_FOLLOW);
                continue;
            }
            if((uint32_t)(now - missing_time) >= APP_TASK_ZONE_HOLD_MS){ ZoneXY_Search(1U); }
            else { ZoneXY_HeadingDrive(base_speed); }
            return 1U;
        case APP_ZONE_XY_FOLLOW:
            if(!fresh){ App_Task_Move_Proc(); return 1U; }
            if(index < 0){
                if(base_speed == 0.0f){ ZoneXY_Search(1U); return 1U; }
                missing_time = now;
                ZoneXY_SetHeading(angle_yaw);
                ZoneXY_SetPhase(APP_ZONE_XY_HOLD);
                ZoneXY_HeadingDrive(base_speed);
                return 1U;
            }
            if(ZoneXY_Follow(x, y)){ continue; }
            return 1U;
        case APP_ZONE_XY_TURN_OUT:
            if(!ZoneXY_TurnDone()){ return 1U; }
            shift_start = position_mid;
            zone_xy_heading_watch.shifted_cm = 0.0f;
            ZoneXY_SetPhase(APP_ZONE_XY_SHIFT);
            continue;
        case APP_ZONE_XY_SHIFT:
            zone_xy_heading_watch.shifted_cm = position_mid - shift_start;
            if(position_mid - shift_start >= shift_cm){
                ZoneXY_SetHeading(return_yaw);
                ZoneXY_SetPhase(APP_ZONE_XY_TURN_BACK);
                continue;
            }
            ZoneXY_HeadingDrive(LINE_PID_Y_MIN_SPEED);
            return 1U;
        case APP_ZONE_XY_TURN_BACK:
            if(!ZoneXY_TurnDone()){ return 1U; }
            ZoneXY_Search(1U); // 新视角图到来前按用户要求旋转；不停车等图。
            return 1U;
        default:
            return 1U; // PREPARE/DONE/TIMEOUT已在上方处理，无额外失败或保护状态。
        }
    }
}
