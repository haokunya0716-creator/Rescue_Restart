#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "../App_Task/app_task_vision.c"
#include "../App_Task/App_Task.c"

static uint32_t test_tick;
static RescueScene input;
static int readable, send_ok, cases, checks, scatter_calls;
static int frame_drop_calls;
static uint32_t last_drop_command_time;
static float wheel_l, wheel_r;
static App_Task_Result distance_result;
Servo_t servo_1, servo_2;
float angle_yaw;

#define CHECK(expr) do { ++checks; if(!(expr)){ fprintf(stderr, "line %d: %s\n", __LINE__, #expr); abort(); } } while(0)
#define CASE() (++cases)
uint32_t HAL_GetTick(void) { return test_tick; }
uint64_t gx_GetUs(void) { return (uint64_t)test_tick * 1000U; }
void App_USART1_Printf(const char *format, ...) { (void)format; }
uint8_t Rescue_Usart2_SendViewContext(uint16_t id, int16_t yaw)
{ (void)id; (void)yaw; return send_ok; }
uint8_t Rescue_Usart2_GetScene(RescueScene *out)
{
    if(!readable || !(input.flags & RESCUE_SCENE_IMAGE_VALID)
       || (uint32_t)(test_tick - input.received_ms) >= RESCUE_VISION_TIMEOUT_MS){ return 0; }
    *out = input;
    return 1;
}
void App_Task_Move_Stop(void) { wheel_l = wheel_r = 0; }
void App_Task_Move_Speed(float l, float r) { wheel_l = l; wheel_r = r; }
void App_Task_Move_Proc(void) {}
void App_Task_Frame_Set(uint8_t down)
{
    servo_1.Angle = down ? SERVO1_DROP : SERVO1_LIFT;
    if(down){ ++frame_drop_calls; last_drop_command_time = test_tick; }
}
void App_Task_Camera_Set(uint8_t down) { servo_2.Angle = down ? SERVO2_DROP : SERVO2_LIFT; }
void App_Task_Move_Start(float distance, float speed) { (void)distance; (void)speed; }
App_Task_Result App_Task_Move_Distance(void) { return distance_result; }
void App_Task_NextCycle_Proc(void) {}
void App_Update_Data(void) {}
void Set_Angle_Target(float heading) { (void)heading; }
void App_Heading_Pro(float speed) { App_Task_Move_Speed(speed, speed); }
void App_Task_Scatter_Start(void) { ++scatter_calls; }
App_Task_Result App_Task_Scatter_Proc(uint8_t *pending) { (void)pending; return APP_STEP_RUNNING; }

static void next_frame(void)
{
    ++input.frame_id;
    input.capture_ms = input.frame_id * 10U;
    input.received_ms = test_tick;
    input.view_id = view_id;
}

static void setup(int phase)
{
    test_tick = 1000;
    readable = send_ok = 1;
    scatter_calls = 0;
    frame_drop_calls = 0;
    frame_drop_time = 0;
    verify_attempt = 0;
    verify_result = APP_STEP_WAIT;
    verify_result_failure = APP_VERIFY_NONE;
    verify_result_reason = "not_checked";
    task_state = TASK_IDLE;
    round_started = cargo_pending = 0;
    servo_1.Angle = SERVO1_LIFT;
    servo_2.Angle = SERVO2_LIFT;
    App_LinePID_Init();
    App_Task_Vision_Init(APP_TASK_TEAM_BLUE);
    App_Task_Vision_BeginMoving(SERVO2_LIFT, 0);
    input = (RescueScene){.image_width=640, .image_height=480,
        .flags=RESCUE_SCENE_IMAGE_VALID, .object_count=1};
    input.objects[0] = (RescueObject){.type=RESCUE_OBJECT_SUPPLY,
        .cx=400, .cy=300, .width=40, .height=40, .track_id=7};
    next_frame();
    CHECK(App_Task_Vision_Select() == 1);
    follow_forward = phase != 0;
    if(phase == 2){
        target.cx = input.objects[0].cx = 360;
        target.cy = input.objects[0].cy = 422;
        arrive_frames = 2;
    }
    capture_y_direction = 1;
    capture_y_request = 1;
    capture_y_frames = 3;
    App_Task_Move_Speed(5, -5);
    next_frame();
}

static void stopped(void) { CHECK(wheel_l == 0 && wheel_r == 0); }

static App_Task_Result y_frame(uint32_t dt, float x, float y)
{
    test_tick += dt;
    input.objects[0].cx = x;
    input.objects[0].cy = y;
    next_frame();
    return App_Task_Vision_Follow();
}

static void test_restored_y_control(void)
{
    const float x = LINE_PID_AIM_X, y = LINE_PID_STOP_Y;
    CASE(); setup(0);
    CHECK(LINE_PID_Y_KP == 2.5f && LINE_PID_Y_MAX_FORWARD == 35.0f
          && LINE_PID_Y_MAX_REVERSE == 35.0f);
    CHECK(LINE_PID_Y_RESUME_WAIT_MS == 200U && LINE_PID_Y_RESUME_FRAMES == 3U
          && LINE_PID_Y_HYSTERESIS == 10.0f);
    target.cx = x; target.cy = y - 30;
    CHECK(y_frame(0, x, y - 30) == APP_STEP_RUNNING);
    CHECK(follow_forward && capture_y_direction == 0 && capture_y_frames == 0); stopped();
    CHECK(y_frame(0, x, y - 30) == APP_STEP_WAIT);
    CHECK(capture_y_request == 1 && capture_y_frames == 1); stopped();
    CHECK(App_Task_Vision_Follow() == APP_STEP_RUNNING);
    CHECK(capture_y_frames == 1); stopped(); // Repeated image cannot count as confirmation.
    CHECK(y_frame(1, x, y - 30) == APP_STEP_WAIT);
    CHECK(y_frame(1, x, y - 30) == APP_STEP_WAIT);
    CHECK(capture_y_frames == 3); stopped();
    CHECK(y_frame(197, x, y - 30) == APP_STEP_WAIT); stopped();
    CHECK(y_frame(1, x, y - 30) == APP_STEP_RUNNING);
    CHECK(capture_y_direction == 1 && wheel_l == 35.0f && wheel_r == 35.0f);

    CASE(); // A reversal first stops; elapsed time alone cannot replace three new images.
    CHECK(y_frame(1, x, y + 30) == APP_STEP_WAIT);
    CHECK(capture_y_direction == 0 && capture_y_request == -1 && capture_y_frames == 1); stopped();
    CHECK(y_frame(200, x, y + 30) == APP_STEP_WAIT);
    CHECK(capture_y_frames == 2); stopped();
    CHECK(y_frame(1, x, y + 30) == APP_STEP_RUNNING);
    CHECK(capture_y_direction == -1 && wheel_l == -35.0f && wheel_r == -35.0f);

    CASE(); // Once stopped in the Y band, errors below the outer threshold stay stopped.
    CHECK(y_frame(1, x + 20, y) == APP_STEP_RUNNING);
    CHECK(capture_y_direction == 0 && wheel_l > 0 && wheel_r < 0);
    CHECK(y_frame(1, x, y - 11) == APP_STEP_WAIT); stopped();
    CHECK(y_frame(1, x, y - 19) == APP_STEP_WAIT);
    CHECK(capture_y_request == 0 && capture_y_frames == 0); stopped();
    CHECK(y_frame(1, x, y - 20) == APP_STEP_WAIT);
    CHECK(capture_y_request == 1 && capture_y_frames == 1); stopped();
    CHECK(y_frame(100, x, y - 20) == APP_STEP_WAIT); stopped();
    CHECK(y_frame(100, x, y - 20) == APP_STEP_RUNNING);
    CHECK(wheel_l == 35.0f && wheel_r == 35.0f);
}

static void test_phases(void)
{
    for(int phase=0; phase<3; ++phase){
        CASE(); setup(phase);
        input.object_count = 0;
        CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT);
        stopped();
        CHECK(green_waiting && lost_frames == 1 && arrive_frames == 0);
        CHECK(capture_y_direction == 0 && capture_y_frames == 0);
        test_tick += 100;
        input.object_count = 1; next_frame();
        CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT);
        stopped();
        CHECK(!green_waiting && lost_frames == 0);
        CHECK(follow_forward == (phase != 0));
        test_tick += 10; next_frame();
        CHECK(App_Task_Vision_Follow() != APP_STEP_TARGET_LOST);
        if(phase == 0){ CHECK(wheel_l > 0 && wheel_r < 0); }

        CASE(); setup(phase); input.object_count=0;
        CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT);
        for(int n=0; n<3; ++n){ CHECK(App_Task_Vision_Follow() == APP_STEP_RUNNING); }
        CHECK(lost_frames == 1);
        test_tick += 30; next_frame(); CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT);
        test_tick += 30; next_frame(); CHECK(App_Task_Vision_Follow() == APP_STEP_TARGET_LOST);
        stopped(); CHECK(lost_frames == 3);

        CASE(); setup(phase); readable=0;
        CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT); stopped();
        uint32_t start = lost_time;
        test_tick += 599;
        CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT);
        CHECK(lost_time == start && lost_frames == 0);
        ++test_tick; readable=1; next_frame();
        CHECK(App_Task_Vision_Follow() == APP_STEP_TARGET_LOST);
        stopped(); CHECK(green_waiting);

        CASE(); setup(phase); readable=0;
        CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT);
        test_tick += 599; readable=1; next_frame();
        CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT);
        CHECK(!green_waiting); stopped();
    }
}

static void test_mixed_inputs(void)
{
    CASE(); setup(1); readable=0;
    CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT);
    uint32_t start = lost_time;
    test_tick += 100; readable=1;
    input.frame_id=last_frame_id; input.capture_ms=last_capture_ms;
    input.received_ms=test_tick; // A repeated good image cannot release the wait.
    CHECK(App_Task_Vision_Follow() == APP_STEP_RUNNING);
    CHECK(green_waiting && lost_frames == 0); stopped();
    input.object_count=0; next_frame();
    CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT);
    test_tick += 100; input.flags |= RESCUE_SCENE_TRUNCATED;
    CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT);
    CHECK(lost_time == start && lost_frames == 1);
    input.flags=RESCUE_SCENE_IMAGE_VALID; next_frame();
    CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT);
    test_tick += 100; readable=0;
    CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT);
    readable=1; next_frame();
    CHECK(App_Task_Vision_Follow() == APP_STEP_TARGET_LOST);
    CHECK(lost_frames == 3 && lost_time == start);

    for(int bad=0; bad<8; ++bad){
        CASE(); setup(1);
        if(bad==0){ readable=0; }
        if(bad==1){ input.flags=0; }
        if(bad==2){ input.flags |= RESCUE_SCENE_TRUNCATED; }
        if(bad==3){ ++input.view_id; }
        if(bad==4){ input.received_ms=test_tick-200; }
        if(bad==5){ input.has_source_age=1; input.source_age_ms=300; }
        if(bad==6){ servo_2.Angle=60; }
        if(bad==7){ send_ok=0; test_tick += 100; }
        CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT); stopped();
        CHECK(green_waiting && lost_frames == 0);
        readable=send_ok=1; servo_2.Angle=SERVO2_LIFT;
        input.flags=RESCUE_SCENE_IMAGE_VALID; input.has_source_age=0;
        test_tick += 10; next_frame();
        CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT);
        CHECK(!green_waiting); stopped();
    }
}

static void test_resolution_and_rejections(void)
{
    CASE(); setup(0);
    for(int n=0; n<4; ++n){
        int half = n%2 == 0;
        input.image_width=half?320:640; input.image_height=half?240:480;
        input.objects[0].cx=half?200:400; input.objects[0].cy=half?150:300;
        input.objects[0].width=input.objects[0].height=half?20:40;
        test_tick += 10; next_frame();
        CHECK(App_Task_Vision_Follow() == APP_STEP_RUNNING);
        CHECK(!green_waiting && target_image_w == input.image_width && target_image_h == input.image_height);
    }
    for(int reason=0; reason<6; ++reason){
        CASE(); setup(1);
        if(reason==0){ input.objects[0].cx += 71; }
        if(reason==1){ input.objects[0].width = 81; }
        if(reason==2){ input.objects[0].height = 19; }
        if(reason==3){ input.objects[0].track_id = 8; }
        if(reason==4){ input.objects[0].type = RESCUE_OBJECT_CORE_SUPPLY; }
        if(reason==5){
            target.track_id = input.objects[0].track_id = 0;
            input.object_count=2; input.objects[1]=input.objects[0]; ++input.objects[1].cx;
        }
        CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT);
        CHECK(lost_frames == 1 && green_waiting); stopped();
    }
    CASE(); setup(1); input.image_width=320; input.image_height=240;
    input.objects[0].cx=236; input.objects[0].cy=150;
    input.objects[0].width=input.objects[0].height=20;
    CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT); CHECK(lost_frames == 1);
}

static void test_task_recovery_and_limits(void)
{
    CASE(); setup(1); round_started=1; round_time=100;
    task_state=TASK_FOLLOW; task_time=test_tick; input.object_count=0;
    for(int n=0; n<3; ++n){ test_tick += 10; next_frame(); App_Task_Proc(); }
    CHECK(task_state == TASK_BACK_SEARCH && task_state == 19);
    CHECK(wheel_l == -15 && wheel_r == -15 && cargo_pending == 0);
    CHECK(servo_1.Angle == SERVO1_LIFT);
    uint16_t previous_view=view_id;
    uint32_t back_start=task_time;
    input.object_count=1;
    test_tick=back_start+999; next_frame(); App_Task_Proc();
    CHECK(task_state == TASK_BACK_SEARCH && wheel_l == -15 && wheel_r == -15);
    test_tick=back_start+1000; App_Task_Proc();
    CHECK(task_state == TASK_SEARCH_SPIN && wheel_l == 15 && wheel_r == -15);
    CHECK(view_id != previous_view && round_time == 100 && team_zone_type == RESCUE_OBJECT_BLUE_SAFE_ZONE);
    CHECK(!green_waiting && lost_frames == 0 && arrive_frames == 0);
    input.objects[0].track_id=99; next_frame(); App_Task_Proc();
    CHECK(task_state == TASK_FOLLOW && target.track_id == 99 && follow_forward == 0);
    stopped();

    for(int phase=0; phase<2; ++phase){
        CASE(); setup(phase);
        task_state = TASK_FOLLOW;
        task_time = round_time = test_tick;
        round_started = 1;
        uint32_t former_limit_ms = phase ? 15000U : 50000U;
        const uint32_t elapsed_ms[] = {
            former_limit_ms - 1U, former_limit_ms, former_limit_ms + 1U,
            APP_TASK_ROUND_TIME_MS - 1U
        };
        for(size_t i=0; i<sizeof(elapsed_ms) / sizeof(elapsed_ms[0]); ++i){
            test_tick = round_time + elapsed_ms[i];
            next_frame();
            App_Task_Proc();
            CHECK(task_state == TASK_FOLLOW && follow_forward == phase);
            CHECK(wheel_l > 0 && (phase ? wheel_r > 0 : wheel_r < 0));
        }
        test_tick = round_time + APP_TASK_ROUND_TIME_MS;
        next_frame();
        App_Task_Proc();
        CHECK(task_state == TASK_IDLE); stopped();
    }
    CASE(); setup(1); task_state=TASK_FOLLOW; round_started=1;
    round_time=0; test_tick=APP_TASK_ROUND_TIME_MS; readable=0;
    App_Task_Proc(); CHECK(task_state == TASK_IDLE); stopped();
    CASE(); setup(1); task_state=TASK_BACK_SEARCH; round_started=1;
    round_time=0; test_tick=APP_TASK_ROUND_TIME_MS;
    App_Task_Proc(); CHECK(task_state == TASK_IDLE); stopped();
    CASE(); setup(1); task_state=TASK_SEARCH_SPIN; round_started=1;
    round_time=0; test_tick=APP_TASK_ROUND_TIME_MS; readable=0;
    App_Task_Proc(); CHECK(task_state == TASK_SEARCH_SPIN && wheel_l == 15 && wheel_r == -15);

    CASE(); setup(1); test_tick=UINT32_MAX-200; next_frame();
    readable=0; CHECK(App_Task_Vision_Follow() == APP_STEP_WAIT);
    test_tick += 600; CHECK(App_Task_Vision_Follow() == APP_STEP_TARGET_LOST);
}

static void test_existing_paths(void)
{
    CASE(); setup(0); target_mixed=1; readable=0;
    CHECK(App_Task_Vision_Follow() == APP_STEP_FAILED); stopped();
    CASE(); setup(0); target_mixed=1; input.image_width=320;
    CHECK(App_Task_Vision_Follow() == APP_STEP_FAILED); stopped();

    CASE(); setup(1); task_state=TASK_VERIFY; task_time=test_tick;
    verify_attempt=1; cargo_pending=1;
    test_tick += APP_TASK_VERIFY_TIMEOUT_MS; App_Task_Proc();
    CHECK(task_state == TASK_VERIFY && verify_attempt == 2 && cargo_pending == 1);
    test_tick += APP_TASK_VERIFY_TIMEOUT_MS; App_Task_Proc();
    CHECK(task_state == TASK_BACK_SEARCH && cargo_pending == 0 && wheel_l == -15 && wheel_r == -15);

    CASE(); setup(1); task_state=TASK_VERIFY; task_time=test_tick;
    verify_attempt=2; cargo_pending=1;
    input.objects[0].cx=360; input.objects[0].cy=422;
    input.objects[0].type=RESCUE_OBJECT_DANGER;
    App_Task_Proc(); CHECK(task_state == TASK_SCATTER && scatter_calls == 1 && cargo_pending == 1);

    CASE(); setup(1); task_state=TASK_VERIFY; task_time=test_tick; cargo_pending=1;
    input.objects[0].cx=360; input.objects[0].cy=422;
    for(int n=0; n<3; ++n){ test_tick += 10; next_frame(); App_Task_Proc(); }
    CHECK(task_state == TASK_ZONE_SEARCH && cargo_pending == 1 && wheel_l == 15 && wheel_r == -15);
}

// Exercise the real stable-arrival -> drop -> verify transition, not a manual VERIFY state.
static uint32_t capture_and_verify(uint32_t begin)
{
    setup(2);
    test_tick=begin;
    round_started=1; round_time=begin;
    task_state=TASK_FOLLOW; task_time=begin;
    arrive_frames=0;
    verify_attempt=2; verify_result=APP_STEP_FAILED; // Previous capture must not leak into this one.
    verify_result_failure=APP_VERIFY_MULTIPLE; verify_result_reason="cargo_multiple";
    uint16_t old_view=view_id;
    for(int n=0;n<3;++n){
        if(n){ test_tick+=75; }
        next_frame(); App_Task_Proc();
        if(n<2){ CHECK(task_state==TASK_FOLLOW && frame_drop_calls==0); }
    }
    CHECK(task_state==TASK_VERIFY && verify_attempt==1 && cargo_pending==1);
    CHECK(frame_drop_calls==1 && frame_drop_time==last_drop_command_time);
    CHECK(task_time==frame_drop_time && task_time==begin+150U);
    CHECK(view_id!=old_view && verify_frames==0 && verify_hits==0);
    CHECK(verify_result==APP_STEP_WAIT && verify_result_failure==APP_VERIFY_NONE);
    CHECK(servo_1.Angle==SERVO1_DROP && servo_2.Angle==SERVO2_LIFT);
    stopped();
    return frame_drop_time;
}

static void cargo_frame(uint32_t dt, uint8_t count, uint8_t type)
{
    test_tick+=dt;
    input.object_count=count;
    input.objects[0]=(RescueObject){.type=type,.cx=360,.cy=422,.width=40,.height=40};
    input.objects[1]=input.objects[0]; input.objects[1].cx=380;
    next_frame(); App_Task_Proc();
}

static void poll_verify_at(uint32_t origin, uint32_t elapsed)
{ test_tick=origin+elapsed; App_Task_Proc(); }

static void test_parallel_verify(void)
{
    CASE(); uint32_t origin=capture_and_verify(1000);
    App_Task_Proc(); CHECK(verify_frames==0); // Capture image has the previous view_id.
    for(int n=0;n<3;++n){ cargo_frame(10,1,RESCUE_OBJECT_SUPPLY); }
    CHECK(verify_result==APP_STEP_DONE && verify_frames==3 && verify_hits==3); stopped();
    uint16_t counted_frame=last_frame_id;
    cargo_frame(10,1,RESCUE_OBJECT_DANGER); // A terminal success is frozen, not rechecked.
    CHECK(verify_result==APP_STEP_DONE && verify_frames==3 && last_frame_id==counted_frame);
    poll_verify_at(origin,499); CHECK(task_state==TASK_VERIFY && verify_attempt==1); stopped();
    readable=0; poll_verify_at(origin,500); // Cached success wins over both bad input and deadline.
    CHECK(task_state==TASK_ZONE_SEARCH && cargo_pending==1 && round_time==1000);
    CHECK(wheel_l==APP_TASK_ZONE_SPIN_SPEED && wheel_r==-APP_TASK_ZONE_SPIN_SPEED);

    CASE(); origin=capture_and_verify(1000);
    cargo_frame(10,1,RESCUE_OBJECT_DANGER);
    CHECK(verify_result==APP_STEP_FAILED && verify_result_failure==APP_VERIFY_WRONG_TYPE);
    for(int n=0;n<3;++n){ cargo_frame(10,1,RESCUE_OBJECT_SUPPLY); }
    CHECK(verify_result_failure==APP_VERIFY_WRONG_TYPE && strcmp(verify_result_reason,"cargo_wrong_type")==0);
    poll_verify_at(origin,499); CHECK(verify_attempt==1); stopped();
    uint16_t old_view=view_id;
    poll_verify_at(origin,500);
    CHECK(task_state==TASK_VERIFY && verify_attempt==2 && task_time==origin+500U);
    CHECK(frame_drop_time==origin && frame_drop_calls==1 && view_id!=old_view);
    CHECK(verify_result==APP_STEP_WAIT && verify_frames==0 && verify_hits==0); stopped();
    for(int n=0;n<3;++n){ cargo_frame(10,1,RESCUE_OBJECT_SUPPLY); }
    CHECK(test_tick==origin+530U && task_state==TASK_ZONE_SEARCH); // No second mechanical wait.

    // Each second-attempt failure reason keeps its original recovery branch, regardless of the first.
    for(int failure=0;failure<3;++failure){
        CASE(); origin=capture_and_verify(1000);
        for(int n=0;n<3;++n){ cargo_frame(10,0,0); }
        CHECK(verify_result==APP_STEP_FAILED && verify_result_failure==APP_VERIFY_NO_HIT);
        poll_verify_at(origin,500); CHECK(verify_attempt==2);
        if(failure==0){
            for(int n=0;n<3;++n){ cargo_frame(10,0,0); }
            CHECK(task_state==TASK_BACK_SEARCH && cargo_pending==0 && servo_1.Angle==SERVO1_LIFT);
            CHECK(wheel_l==-APP_TASK_SEARCH_BACK_SPEED && wheel_r==-APP_TASK_SEARCH_BACK_SPEED);
        }else {
            cargo_frame(10,failure==1?2:1,failure==1?RESCUE_OBJECT_SUPPLY:RESCUE_OBJECT_DANGER);
            CHECK(task_state==TASK_SCATTER && scatter_calls==1 && cargo_pending==1);
            CHECK(servo_1.Angle==SERVO1_DROP);
        }
        CHECK(frame_drop_calls==1 && round_time==1000);
    }

    CASE(); origin=capture_and_verify(1000); readable=0;
    poll_verify_at(origin,499); CHECK(verify_attempt==1); stopped();
    poll_verify_at(origin,500); CHECK(verify_attempt==2 && task_time==origin+500U); stopped();
    poll_verify_at(origin,999); CHECK(task_state==TASK_VERIFY && verify_attempt==2); stopped();
    poll_verify_at(origin,1000);
    CHECK(task_state==TASK_BACK_SEARCH && cargo_pending==0 && test_tick==origin+1000U);
    CHECK(wheel_l==-APP_TASK_SEARCH_BACK_SPEED && wheel_r==-APP_TASK_SEARCH_BACK_SPEED);
    CHECK(strcmp(verify_result_reason,"verify_timeout")==0);

    CASE(); origin=capture_and_verify(1000);
    cargo_frame(10,1,RESCUE_OBJECT_SUPPLY); cargo_frame(10,1,RESCUE_OBJECT_SUPPLY);
    CHECK(verify_frames==2);
    cargo_frame(480,1,RESCUE_OBJECT_SUPPLY); // Third image exactly at first deadline is late.
    CHECK(verify_attempt==2 && task_state==TASK_VERIFY && verify_frames==0);
    cargo_frame(10,1,RESCUE_OBJECT_SUPPLY); cargo_frame(10,1,RESCUE_OBJECT_SUPPLY);
    cargo_frame(480,1,RESCUE_OBJECT_SUPPLY);
    CHECK(task_state==TASK_BACK_SEARCH && test_tick==origin+1000U);

    CASE(); origin=capture_and_verify(1000);
    cargo_frame(10,1,RESCUE_OBJECT_SUPPLY); cargo_frame(10,1,RESCUE_OBJECT_SUPPLY);
    cargo_frame(479,1,RESCUE_OBJECT_SUPPLY); // Third image at 499ms is still accepted.
    CHECK(task_state==TASK_VERIFY && verify_result==APP_STEP_DONE); stopped();
    poll_verify_at(origin,500); CHECK(task_state==TASK_ZONE_SEARCH);

    CASE(); origin=capture_and_verify(1000);
    cargo_frame(10,1,RESCUE_OBJECT_SUPPLY);
    poll_verify_at(origin,20); CHECK(verify_frames==1 && verify_hits==1); // Duplicate.
    ++input.view_id; poll_verify_at(origin,30); CHECK(verify_frames==0 && verify_hits==0);
    cargo_frame(10,1,RESCUE_OBJECT_SUPPLY); CHECK(verify_frames==1);
    input.flags|=RESCUE_SCENE_TRUNCATED; poll_verify_at(origin,50); CHECK(verify_frames==0);
    input.flags=RESCUE_SCENE_IMAGE_VALID;
    cargo_frame(10,1,RESCUE_OBJECT_SUPPLY); CHECK(verify_frames==1);
    poll_verify_at(origin,260); CHECK(verify_frames==0 && verify_hits==0); // Local age reaches 200ms.
    poll_verify_at(origin,500); CHECK(verify_attempt==2 && task_time==origin+500U);

    CASE(); origin=capture_and_verify(1000);
    for(int n=0;n<3;++n){ cargo_frame(10,1,RESCUE_OBJECT_SUPPLY); }
    poll_verify_at(round_time,APP_TASK_ROUND_TIME_MS);
    CHECK(task_state==TASK_IDLE && cargo_pending==1); stopped(); // Round timeout beats cached success.

    for(int success=0;success<2;++success){
        CASE(); origin=capture_and_verify(UINT32_MAX-200U);
        if(success){
            for(int n=0;n<3;++n){ cargo_frame(10,1,RESCUE_OBJECT_SUPPLY); }
            poll_verify_at(origin,499); CHECK(task_state==TASK_VERIFY); stopped();
            poll_verify_at(origin,500); CHECK(task_state==TASK_ZONE_SEARCH);
        }else {
            readable=0; poll_verify_at(origin,500); CHECK(verify_attempt==2);
            poll_verify_at(origin,999); CHECK(task_state==TASK_VERIFY); stopped();
            poll_verify_at(origin,1000); CHECK(task_state==TASK_BACK_SEARCH);
        }
    }
}

static void test_verify_single_hit(void)
{
    for(int attempt=1;attempt<=2;++attempt){
        for(int hit=0;hit<3;++hit){
            CASE(); uint32_t origin=capture_and_verify(1000);
            if(attempt==2){
                cargo_frame(10,1,RESCUE_OBJECT_DANGER);
                poll_verify_at(origin,500);
                CHECK(verify_attempt==2 && verify_frames==0 && verify_hits==0);
            }
            for(int n=0;n<3;++n){
                cargo_frame(10,n==hit?1:0,RESCUE_OBJECT_SUPPLY);
                if(n<2){
                    CHECK(task_state==TASK_VERIFY && verify_result==APP_STEP_WAIT);
                    CHECK(verify_frames==n+1 && verify_hits==(n>=hit?1:0));
                    stopped();
                }
            }
            if(attempt==1){
                CHECK(verify_result==APP_STEP_DONE && verify_frames==3 && verify_hits==1);
                poll_verify_at(origin,499); CHECK(task_state==TASK_VERIFY); stopped();
                poll_verify_at(origin,500);
            }else {
                CHECK(test_tick==origin+530U); // Second attempt ends immediately after its third image.
            }
            CHECK(task_state==TASK_ZONE_SEARCH && cargo_pending==1 && frame_drop_calls==1);
            CHECK(wheel_l==APP_TASK_ZONE_SPIN_SPEED && wheel_r==-APP_TASK_ZONE_SPIN_SPEED);
            CHECK(round_time==1000);
        }
    }
}

static void test_verify_delayed_dispatch(void)
{
    for(int outcome=0;outcome<4;++outcome){
        CASE(); uint32_t origin=capture_and_verify(1000);
        if(outcome<2){
            for(int n=0;n<3;++n){ cargo_frame(10,outcome==0?1:0,RESCUE_OBJECT_SUPPLY); }
        }else {
            cargo_frame(10,outcome==2?2:1,outcome==2?RESCUE_OBJECT_SUPPLY:RESCUE_OBJECT_DANGER);
        }
        CHECK(verify_result==(outcome==0?APP_STEP_DONE:APP_STEP_FAILED));
        App_Task_VerifyFailure saved_failure=verify_result_failure;
        const char *saved_reason=verify_result_reason;
        uint16_t saved_view=view_id;
        readable=0;
        poll_verify_at(origin,499); CHECK(task_state==TASK_VERIFY && verify_attempt==1); stopped();
        CHECK(verify_result_failure==saved_failure && strcmp(verify_result_reason,saved_reason)==0);
        poll_verify_at(origin,600); // Main loop skipped the exact mechanical boundary.
        if(outcome==0){
            CHECK(task_state==TASK_ZONE_SEARCH && cargo_pending==1);
            CHECK(wheel_l==APP_TASK_ZONE_SPIN_SPEED && wheel_r==-APP_TASK_ZONE_SPIN_SPEED);
        }else {
            CHECK(task_state==TASK_VERIFY && verify_attempt==2 && task_time==origin+600U);
            CHECK(verify_result==APP_STEP_WAIT && verify_frames==0 && verify_hits==0); stopped();
            poll_verify_at(origin,1099); CHECK(task_state==TASK_VERIFY); stopped();
            poll_verify_at(origin,1100);
            CHECK(task_state==TASK_BACK_SEARCH && cargo_pending==0);
            CHECK(wheel_l==-APP_TASK_SEARCH_BACK_SPEED && wheel_r==-APP_TASK_SEARCH_BACK_SPEED);
        }
        CHECK(view_id!=saved_view && frame_drop_calls==1 && round_time==1000);
    }
}

static void test_verify_round_priority(void)
{
    for(int failure=0;failure<3;++failure){
        CASE(); uint32_t origin=capture_and_verify(APP_TASK_ROUND_TIME_MS-650U);
        round_time=0; // The round ends at the first mechanical/verification deadline.
        if(failure==0){
            for(int n=0;n<3;++n){ cargo_frame(10,0,0); }
        }else {
            cargo_frame(10,failure==1?2:1,failure==1?RESCUE_OBJECT_SUPPLY:RESCUE_OBJECT_DANGER);
        }
        CHECK(verify_result==APP_STEP_FAILED);
        App_Task_VerifyFailure saved_failure=verify_result_failure;
        const char *saved_reason=verify_result_reason;
        uint16_t saved_view=view_id;
        poll_verify_at(origin,499); CHECK(task_state==TASK_VERIFY && verify_attempt==1); stopped();
        poll_verify_at(origin,500);
        CHECK(task_state==TASK_IDLE && cargo_pending==1 && verify_attempt==1); stopped();
        CHECK(verify_result_failure==saved_failure && strcmp(verify_result_reason,saved_reason)==0);
        CHECK(view_id==saved_view && scatter_calls==0 && servo_1.Angle==SERVO1_DROP);
        CHECK(round_time==0 && frame_drop_calls==1);
    }

    for(int outcome=0;outcome<4;++outcome){
        CASE(); uint32_t origin=capture_and_verify(APP_TASK_ROUND_TIME_MS-(outcome==0?1150U:680U));
        round_time=0;
        readable=0; poll_verify_at(origin,500);
        CHECK(task_state==TASK_VERIFY && verify_attempt==2); stopped();
        if(outcome!=0){
            readable=1;
            for(int n=0;n<2;++n){ cargo_frame(10,outcome==2?0:1,RESCUE_OBJECT_SUPPLY); }
            CHECK(verify_result==APP_STEP_WAIT && verify_frames==2);
        }
        uint16_t saved_view=view_id, saved_frame=last_frame_id;
        uint8_t saved_frames=verify_frames;
        test_tick=APP_TASK_ROUND_TIME_MS;
        input.object_count=outcome==2?0:1;
        input.objects[0].type=outcome==3?RESCUE_OBJECT_DANGER:RESCUE_OBJECT_SUPPLY;
        next_frame(); App_Task_Proc(); // Round timeout wins over timeout, success, no-hit and conflict.
        CHECK(task_state==TASK_IDLE && cargo_pending==1 && verify_attempt==2); stopped();
        CHECK(verify_result==APP_STEP_WAIT && verify_frames==saved_frames && last_frame_id==saved_frame);
        CHECK(view_id==saved_view && scatter_calls==0 && servo_1.Angle==SERVO1_DROP);
        CHECK(round_time==0 && frame_drop_calls==1);
    }
}

int main(void)
{
    test_restored_y_control();
    test_phases();
    test_mixed_inputs();
    test_resolution_and_rejections();
    test_task_recovery_and_limits();
    test_existing_paths();
    test_parallel_verify();
    test_verify_single_hit();
    test_verify_delayed_dispatch();
    test_verify_round_priority();
    printf("PASS: %d scenarios, %d checks\n", cases, checks);
    return 0;
}
