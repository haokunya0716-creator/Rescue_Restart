/* Replay the actual Y-only test with camera, tick and actuator stubs. */
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "../App_Task/app_zone_y_test.c"

static uint32_t tick;
static RescueScene input;
static uint8_t readable;
static float left_speed, right_speed;
static int checks, cases, receive_calls, debug_calls, sends, frame_down, camera_down;
#define CHECK(x) do { ++checks; if(!(x)){ fprintf(stderr,"line %d: %s\n",__LINE__,#x); abort(); } } while(0)
#define CASE() (++cases)
static int eq(float a, float b) { return fabsf(a-b) < 0.001f; }
uint32_t HAL_GetTick(void) { return tick; }
uint64_t gx_GetUs(void) { return (uint64_t)tick * 1000U; }
void Rescue_Usart2_Process(void) { ++receive_calls; }
void Rescue_Usart2_Debug_Proc(void) { ++debug_calls; }
uint8_t Rescue_Usart2_GetScene(RescueScene *out)
{
    if(!readable || !(input.flags & RESCUE_SCENE_IMAGE_VALID)
       || (uint32_t)(tick-input.received_ms) >= RESCUE_VISION_TIMEOUT_MS){ return 0; }
    *out = input;
    return 1;
}
uint8_t Rescue_Usart2_SendViewContext(uint16_t id, int16_t yaw)
{ CHECK(id == 1 && yaw == 0); ++sends; return 1; }
void App_Task_Move_Stop(void) { left_speed = right_speed = 0; }
void App_Task_Move_Speed(float l, float r) { left_speed=l; right_speed=r; }
void App_Task_Move_Proc(void) {}
void App_Task_Frame_Set(uint8_t down) { frame_down=down; }
void App_Task_Camera_Set(uint8_t down) { camera_down=down; }

static void poll(uint32_t dt) { tick+=dt; CHECK(App_ZoneY_Test(APP_TASK_TEAM_RED) == 1); }
static void image(uint32_t dt, uint8_t type, uint16_t cy)
{
    tick+=dt; ++input.frame_id;
    input.capture_ms=tick; input.received_ms=tick;
    input.object_count=type ? 1 : 0;
    input.objects[0]=(RescueObject){.type=type,.cx=0,.cy=cy}; // X严重偏心，仍必须只同速前后。
    CHECK(App_ZoneY_Test(APP_TASK_TEAM_RED) == 1);
    CHECK(eq(left_speed,right_speed));
    CHECK(eq(left_speed,zone_y_watch.speed));
}
static void stopped(void) { CHECK(eq(left_speed,0) && eq(right_speed,0)); }
static void reset_at(uint32_t start_tick, uint8_t team)
{
    started=team_valid=frame_seen=pid_seen=view_sent=0;
    last_frame=0; last_capture=pid_time=view_time=0;
    integral=previous_error=0;
    previous_config=(App_ZoneY_Tune){0};
    zone_y_tune=(App_ZoneY_Tune){.enable=1,.kp=0.2f,.target_y=100,.tolerance=5,
        .min_speed=3,.max_forward=25,.max_reverse=8,.integral_limit=10};
    zone_y_watch=(App_ZoneY_Watch){0};
    tick=start_tick; readable=1; left_speed=right_speed=0;
    receive_calls=debug_calls=sends=0; frame_down=camera_down=-1;
    input=(RescueScene){.view_id=1,.image_width=640,.image_height=480,
        .flags=RESCUE_SCENE_IMAGE_VALID,.received_ms=tick};
    CHECK(App_ZoneY_Test(team) == 1);
    stopped();
}
static void start(uint8_t team)
{ reset_at(1000,team); poll(APP_TASK_SERVO_WAIT_MS); }

int main(void)
{
    CASE(); reset_at(1000,APP_TASK_TEAM_RED);
    CHECK(frame_down==1 && camera_down==0 && zone_y_watch.status==APP_ZONE_Y_PREPARE);
    image(APP_TASK_SERVO_WAIT_MS-1,RESCUE_OBJECT_RED_SAFE_ZONE,0); stopped(); CHECK(sends==0);
    image(1,RESCUE_OBJECT_RED_SAFE_ZONE,0);
    CHECK(zone_y_watch.status==APP_ZONE_Y_RUNNING && eq(left_speed,20));
    CHECK(receive_calls==3 && debug_calls==3 && sends==1);
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,90); CHECK(eq(left_speed,3));
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,200); CHECK(eq(left_speed,-8));

    CASE(); start(APP_TASK_TEAM_RED); input.image_width=1280; input.image_height=960;
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,100);
    CHECK(eq(zone_y_watch.y,50) && eq(zone_y_watch.error,50) && eq(left_speed,10));

    CASE(); start(APP_TASK_TEAM_RED);
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,95); stopped(); CHECK(zone_y_watch.in_band==1);
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,105); stopped();
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,94); CHECK(eq(left_speed,3) && zone_y_watch.in_band==0);

    CASE(); start(APP_TASK_TEAM_RED); image(10,RESCUE_OBJECT_RED_SAFE_ZONE,80);
    CHECK(eq(left_speed,4));
    zone_y_tune.kp=0.5f; poll(1); stopped(); // 改参不对同一旧帧再次计算。
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,80); CHECK(eq(left_speed,10));
    zone_y_tune.target_y=200; image(10,RESCUE_OBJECT_RED_SAFE_ZONE,80); CHECK(eq(left_speed,25));
    zone_y_tune.min_speed=20; zone_y_tune.max_forward=5;
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,199); stopped(); // 停车带优先于最小速度。
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,190); CHECK(eq(left_speed,5));

    CASE(); start(APP_TASK_TEAM_RED);
    zone_y_tune.kp=0; zone_y_tune.ki=1; zone_y_tune.min_speed=0; zone_y_tune.integral_limit=3;
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,90); stopped(); CHECK(eq(zone_y_watch.i,0));
    image(100,RESCUE_OBJECT_RED_SAFE_ZONE,90); CHECK(eq(zone_y_watch.i,1) && eq(left_speed,1));
    image(200,RESCUE_OBJECT_RED_SAFE_ZONE,90); CHECK(eq(zone_y_watch.i,3));
    poll(100); CHECK(eq(zone_y_watch.i,3)); // 重复帧不能积分。
    image(100,RESCUE_OBJECT_RED_SAFE_ZONE,90); CHECK(eq(zone_y_watch.i,3));
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,100); stopped(); CHECK(eq(zone_y_watch.i,0));
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,90); stopped(); // 离开停车带第一帧不继承积分。

    CASE(); start(APP_TASK_TEAM_RED);
    zone_y_tune.kp=1; zone_y_tune.ki=1;
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0);
    image(100,RESCUE_OBJECT_RED_SAFE_ZONE,0);
    CHECK(eq(left_speed,25) && eq(zone_y_watch.i,0)); // 饱和不积累。
    image(100,RESCUE_OBJECT_RED_SAFE_ZONE,90); CHECK(eq(zone_y_watch.i,1) && eq(left_speed,11));

    CASE(); start(APP_TASK_TEAM_RED);
    zone_y_tune.kp=0; zone_y_tune.kd=0.1f; zone_y_tune.min_speed=0;
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,50); CHECK(eq(zone_y_watch.d,0));
    image(100,RESCUE_OBJECT_RED_SAFE_ZONE,60);
    CHECK(eq(zone_y_watch.d,-10) && eq(zone_y_watch.dt,0.1f) && eq(left_speed,-8));
    image(0,RESCUE_OBJECT_RED_SAFE_ZONE,65); CHECK(eq(zone_y_watch.d,0) && isfinite(left_speed));
    image(600,RESCUE_OBJECT_RED_SAFE_ZONE,20); CHECK(eq(zone_y_watch.d,0) && eq(zone_y_watch.dt,0));

    CASE(); start(APP_TASK_TEAM_RED);
    zone_y_tune.ki=1; zone_y_tune.kd=0.1f;
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,50);
    image(100,RESCUE_OBJECT_RED_SAFE_ZONE,50); CHECK(zone_y_watch.i>0);
    image(10,0,0); stopped(); CHECK(zone_y_watch.valid==0 && eq(zone_y_watch.i,0));
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,60); CHECK(eq(zone_y_watch.d,0) && eq(zone_y_watch.i,0));
    image(10,RESCUE_OBJECT_BLUE_SAFE_ZONE,50); stopped();
    image(10,RESCUE_OBJECT_SUPPLY,50); stopped();
    image(10,RESCUE_OBJECT_RED_SUPPLY_ZONE,50); stopped();

    CASE(); start(APP_TASK_TEAM_BLUE);
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0); stopped();
    image(10,RESCUE_OBJECT_BLUE_SAFE_ZONE,0); CHECK(eq(left_speed,20));

    CASE(); start(APP_TASK_TEAM_RED); image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0);
    poll(RESCUE_VISION_TIMEOUT_MS-1); CHECK(eq(left_speed,20));
    poll(1); stopped(); CHECK(zone_y_watch.valid==0);
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0); CHECK(eq(left_speed,20));
    input.view_id=2; poll(1); stopped();
    input.view_id=1; input.image_height=0; image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0); stopped();
    input.image_height=480; input.flags=0; image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0); stopped();
    input.flags=RESCUE_SCENE_IMAGE_VALID; image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0);
    readable=0; poll(1); stopped();

    CASE(); start(APP_TASK_TEAM_RED); image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0);
    zone_y_tune.enable=0; poll(0); stopped(); CHECK(zone_y_watch.status==APP_ZONE_Y_DISABLED);
    zone_y_tune.enable=1; poll(0); stopped();
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0); CHECK(eq(left_speed,20));
    zone_y_tune.reset=1; poll(0); stopped(); CHECK(zone_y_tune.reset==0);
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0); CHECK(eq(left_speed,20));

    CASE(); start(APP_TASK_TEAM_RED);
    zone_y_tune.kp=-1; image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0);
    stopped(); CHECK(zone_y_watch.status==APP_ZONE_Y_BAD_CONFIG);
    zone_y_tune.kp=NAN; image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0); stopped();
    zone_y_tune.kp=FLT_MAX; image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0); stopped();
    zone_y_tune.kp=0; image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0); stopped(); // 全零PID不受最小速度强推。
    zone_y_tune.kp=1; zone_y_tune.max_forward=0;
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0); stopped();
    zone_y_tune.max_forward=25; zone_y_tune.max_reverse=0;
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,200); stopped();

    CASE(); start(APP_TASK_TEAM_RED); image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0);
    tick=start_time+APP_TASK_ROUND_TIME_MS; poll(0); stopped();
    CHECK(zone_y_watch.status==APP_ZONE_Y_TIMEOUT);
    zone_y_tune.reset=1; image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0); stopped();
    CHECK(zone_y_watch.status==APP_ZONE_Y_TIMEOUT);

    CASE(); reset_at(UINT32_MAX-600U,APP_TASK_TEAM_RED); poll(APP_TASK_SERVO_WAIT_MS);
    image(10,RESCUE_OBJECT_RED_SAFE_ZONE,0); CHECK(eq(left_speed,20));
    image(200,RESCUE_OBJECT_RED_SAFE_ZONE,10); CHECK(eq(left_speed,18) && eq(zone_y_watch.dt,0.2f));
    tick=start_time+APP_TASK_ROUND_TIME_MS; poll(0); stopped();

    printf("zone_y_tuning: %d cases, %d checks passed\n",cases,checks);
    return 0;
}
