#include "app_zone_y_test.h"
#include "app_task_move.h"
#include "rescue_usart2.h"
#include <math.h>

volatile App_ZoneY_Tune zone_y_tune = {
    .enable = 1U, .reset = 0U,
    .kp = LINE_PID_Y_KP, .ki = 0.0f, .kd = 0.0f,
    .target_y = APP_TASK_ZONE_AIM_Y, .tolerance = APP_TASK_ZONE_Y_TOL,
    .min_speed = LINE_PID_Y_MIN_SPEED,
    .max_forward = LINE_PID_Y_MAX_FORWARD, .max_reverse = 25.0f, // 独立Y调参倒车上限，单位cm/s。
    .integral_limit = 10.0f
};
volatile App_ZoneY_Watch zone_y_watch;

static uint8_t started, team_valid, zone_type, frame_seen, pid_seen, view_sent;
static uint16_t last_frame;
static uint32_t start_time, view_time, last_capture, pid_time;
static float integral, previous_error;
static App_ZoneY_Tune previous_config;

static float ZoneY_Clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static void ZoneY_ResetPID(void)
{
    pid_seen = 0;
    integral = previous_error = 0.0f;
    zone_y_watch.p = zone_y_watch.i = zone_y_watch.d = 0.0f;
    zone_y_watch.dt = zone_y_watch.raw_speed = zone_y_watch.speed = 0.0f;
}

static void ZoneY_Stop(App_ZoneY_Status status)
{
    App_Task_Move_Stop();
    ZoneY_ResetPID();
    zone_y_watch.status = status;
    zone_y_watch.valid = zone_y_watch.in_band = 0;
}

static uint8_t ZoneY_ConfigValid(const App_ZoneY_Tune *c)
{
    return isfinite(c->kp) && c->kp >= 0.0f
        && isfinite(c->ki) && c->ki >= 0.0f
        && isfinite(c->kd) && c->kd >= 0.0f
        && isfinite(c->target_y) && c->target_y >= 0.0f && c->target_y <= APP_TASK_IMAGE_H
        && isfinite(c->tolerance) && c->tolerance >= 0.0f && c->tolerance <= APP_TASK_IMAGE_H
        && isfinite(c->min_speed) && c->min_speed >= 0.0f
        && isfinite(c->max_forward) && c->max_forward >= 0.0f
        && isfinite(c->max_reverse) && c->max_reverse >= 0.0f
        && isfinite(c->integral_limit) && c->integral_limit >= 0.0f;
}

static uint8_t ZoneY_ConfigChanged(const App_ZoneY_Tune *c)
{
    return c->enable != previous_config.enable
        || c->kp != previous_config.kp || c->ki != previous_config.ki || c->kd != previous_config.kd
        || c->target_y != previous_config.target_y || c->tolerance != previous_config.tolerance
        || c->min_speed != previous_config.min_speed
        || c->max_forward != previous_config.max_forward || c->max_reverse != previous_config.max_reverse
        || c->integral_limit != previous_config.integral_limit;
}

/* 只在有效新图调用；独立PID以秒为时间单位，不修改绿色及正式整区的共用P控制。 */
static void ZoneY_Drive(const App_ZoneY_Tune *c, float y, uint32_t now)
{
    float error = c->target_y - y;
    zone_y_watch.valid = 1;
    zone_y_watch.y = y;
    zone_y_watch.error = error;
    if(fabsf(error) <= c->tolerance){
        ZoneY_Stop(APP_ZONE_Y_IN_BAND);
        zone_y_watch.valid = zone_y_watch.in_band = 1;
        return; // 停车带内清除I/D历史；偏离后仍可重新调整，不锁定完成状态。
    }

    uint32_t elapsed = (uint32_t)(now - pid_time);
    float dt = pid_seen && elapsed < RESCUE_VISION_TIMEOUT_MS ? elapsed * 0.001f : 0.0f;
    if(pid_seen && elapsed >= RESCUE_VISION_TIMEOUT_MS){ integral = 0.0f; }
    float p = c->kp * error;
    float d = dt > 0.0f ? c->kd * (error - previous_error) / dt : 0.0f;
    float candidate = c->ki == 0.0f ? 0.0f : ZoneY_Clamp(integral + c->ki * error * dt,
                                                       -c->integral_limit, c->integral_limit);
    float raw = p + candidate + d;
    // 输出饱和时禁止继续向饱和方向积累I项，但允许I项回退。
    if((raw > c->max_forward && candidate > integral)
       || (raw < -c->max_reverse && candidate < integral)){ candidate = integral; }
    integral = candidate;
    raw = p + integral + d;
    float speed = ZoneY_Clamp(raw, -c->max_reverse, c->max_forward);
    if(speed > 0.0f && speed < c->min_speed){ speed = fminf(c->min_speed, c->max_forward); }
    if(speed < 0.0f && -speed < c->min_speed){ speed = -fminf(c->min_speed, c->max_reverse); }

    if(!isfinite(raw) || !isfinite(speed)){
        ZoneY_Stop(APP_ZONE_Y_BAD_CONFIG);
        return;
    }
    previous_error = error;
    pid_time = now;
    pid_seen = 1;
    zone_y_watch.status = APP_ZONE_Y_RUNNING;
    zone_y_watch.in_band = 0;
    zone_y_watch.dt = dt;
    zone_y_watch.p = p;
    zone_y_watch.i = integral;
    zone_y_watch.d = d;
    zone_y_watch.raw_speed = raw;
    zone_y_watch.speed = speed;
    App_Task_Move_Speed(speed, speed); // 无X视觉差速、无航向外环，左右轮目标始终相同。
}

uint8_t App_ZoneY_Test(uint8_t team)
{
    uint32_t now = HAL_GetTick();
    if(!started){
        started = 1;
        start_time = now;
        team_valid = team == APP_TASK_TEAM_RED || team == APP_TASK_TEAM_BLUE;
        zone_type = team == APP_TASK_TEAM_BLUE ? RESCUE_OBJECT_BLUE_SAFE_ZONE : RESCUE_OBJECT_RED_SAFE_ZONE;
        zone_y_watch.view_id = 1U;
        App_Task_Frame_Set(1);
        App_Task_Camera_Set(0);
        ZoneY_Stop(APP_ZONE_Y_PREPARE);
    }
    Rescue_Usart2_Process();
    Rescue_Usart2_Debug_Proc();

    App_ZoneY_Tune config = zone_y_tune; // 每次读取RAM参数，Ozone改值不会被宏覆盖。
    uint8_t reset_requested = config.reset != 0U;
    if(reset_requested){ zone_y_tune.reset = 0U; }
    if(reset_requested || ZoneY_ConfigChanged(&config)){
        ZoneY_Stop(APP_ZONE_Y_WAIT_TARGET); // 改参后先停车清历史，等待尚未处理的新图再计算。
        previous_config = config;
    }
    if((uint32_t)(now - start_time) >= APP_TASK_ROUND_TIME_MS){
        ZoneY_Stop(APP_ZONE_Y_TIMEOUT);
        return 1;
    }
    if(!config.enable){ ZoneY_Stop(APP_ZONE_Y_DISABLED); return 1; }
    if(!team_valid || !ZoneY_ConfigValid(&config)){ ZoneY_Stop(APP_ZONE_Y_BAD_CONFIG); return 1; }
    if((uint32_t)(now - start_time) < APP_TASK_SERVO_WAIT_MS){
        ZoneY_Stop(APP_ZONE_Y_PREPARE);
        return 1;
    }
    // 与正式视觉协议相同：发送并重发view_id，直到相机返回对应新图。
    if(!view_sent || (uint32_t)(now - view_time) >= APP_TASK_VIEW_RETRY_MS){
        view_sent = Rescue_Usart2_SendViewContext((uint16_t)zone_y_watch.view_id, 0);
        view_time = now;
    }
    RescueScene scene;
    if(!Rescue_Usart2_GetScene(&scene) || scene.view_id != zone_y_watch.view_id
       || !(scene.flags & RESCUE_SCENE_IMAGE_VALID) || !scene.image_width || !scene.image_height
       || scene.object_count > RESCUE_MAX_OBJECTS){
        ZoneY_Stop(APP_ZONE_Y_WAIT_TARGET);
        return 1;
    }
    if(frame_seen && scene.frame_id == last_frame && scene.capture_ms == last_capture){
        App_Task_Move_Proc(); // 重复帧只维持速度内环，既不重复积分，也不重新计算D项。
        return 1;
    }
    frame_seen = 1;
    last_frame = scene.frame_id;
    last_capture = scene.capture_ms;
    zone_y_watch.frame_id = scene.frame_id;
    for(uint8_t i = 0; i < scene.object_count; ++i){
        if(scene.objects[i].type == zone_type){
            float y = scene.objects[i].cy * APP_TASK_IMAGE_H / scene.image_height;
            ZoneY_Drive(&config, y, now);
            return 1;
        }
    }
    ZoneY_Stop(APP_ZONE_Y_WAIT_TARGET); // 有效新图没有本队整区也立即停车，不旋转重搜。
    return 1;
}
