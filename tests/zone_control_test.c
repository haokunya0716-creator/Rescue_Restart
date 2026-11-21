/* Host replay of the real task/vision state machines. Hardware is stubbed only. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "../App_Task/app_task_vision.c"
#include "../App_Task/App_Task.c"
#include "../App_Task/app_zone_test.h"

static uint32_t tick;
static RescueScene input;
static int readable, send_ok, checks, cases, heading_calls, rx_calls, debug_calls;
static float wheel_l, wheel_r, heading_target, held_speed;
Servo_t servo_1, servo_2;
float angle_yaw;
#define CHECK(x) do { ++checks; if(!(x)){ fprintf(stderr,"line %d: %s\n",__LINE__,#x); abort(); } } while(0)
#define CASE() (++cases)
static int eq(float a, float b) { return fabsf(a-b) < 0.001f; }
uint32_t HAL_GetTick(void) { return tick; }
uint64_t gx_GetUs(void) { return (uint64_t)tick * 1000U; }
void App_USART1_Printf(const char *format, ...) { (void)format; }
uint8_t Rescue_Usart2_SendViewContext(uint16_t id, int16_t yaw) { (void)id; (void)yaw; return send_ok; }
uint8_t Rescue_Usart2_GetScene(RescueScene *out)
{
    if(!readable || !(input.flags & RESCUE_SCENE_IMAGE_VALID)
       || (uint32_t)(tick-input.received_ms) >= RESCUE_VISION_TIMEOUT_MS){ return 0; }
    *out=input; return 1;
}
void Rescue_Usart2_Process(void) { ++rx_calls; }
void Rescue_Usart2_Debug_Proc(void) { ++debug_calls; }
void App_Task_Move_Stop(void) { wheel_l=wheel_r=0; }
void App_Task_Move_Speed(float l,float r) { wheel_l=l; wheel_r=r; }
void App_Task_Move_Proc(void) {}
void App_Task_Frame_Set(uint8_t down) { servo_1.Angle=down?SERVO1_DROP:SERVO1_LIFT; }
void App_Task_Camera_Set(uint8_t down) { servo_2.Angle=down?SERVO2_DROP:SERVO2_LIFT; }
void App_Update_Data(void) {}
void Set_Angle_Target(float h) { heading_target=h; }
void App_Heading_Pro(float speed) { ++heading_calls; held_speed=speed; App_Task_Move_Speed(speed,speed); }
void App_Task_Scatter_Start(void) {}
App_Task_Result App_Task_Scatter_Proc(uint8_t *pending) { (void)pending; return APP_STEP_RUNNING; }

static void stopped(void) { CHECK(eq(wheel_l,0) && eq(wheel_r,0)); }
static void spinning(void) { CHECK(eq(wheel_l,APP_TASK_ZONE_SPIN_SPEED) && eq(wheel_r,-APP_TASK_ZONE_SPIN_SPEED)); }
static void reset(void)
{
    tick=1000; readable=send_ok=1; heading_calls=rx_calls=debug_calls=0;
    wheel_l=wheel_r=held_speed=heading_target=0; angle_yaw=27;
    task_state=TASK_IDLE; round_started=cargo_pending=0; view_id=0;
    frame_drop_time=0; verify_attempt=0; verify_result=APP_STEP_WAIT;
    verify_result_failure=APP_VERIFY_NONE; verify_result_reason="not_checked";
    zone_missing=zone_confirming=zone_stopped=0;
    servo_1.Angle=SERVO1_LIFT; servo_2.Angle=SERVO2_LIFT;
    App_LinePID_Init();
    input=(RescueScene){.image_width=640,.image_height=480,.flags=RESCUE_SCENE_IMAGE_VALID};
}
static void image_at(uint8_t type,float x,float y)
{
    ++input.frame_id; input.capture_ms=tick; input.received_ms=tick; input.view_id=view_id;
    input.object_count=type?1:0;
    input.objects[0]=(RescueObject){.type=type,.cx=(uint16_t)x,.cy=(uint16_t)y,.width=40,.height=40};
}
static void frame(uint32_t dt,uint8_t type,float x,float y)
{ tick+=dt; image_at(type,x,y); App_Task_Proc(); }
static void poll(uint32_t dt) { tick+=dt; App_Task_Proc(); }
static void start(uint8_t team)
{
    reset(); App_Task_StartZoneTest(team);
    CHECK(task_state==TASK_ZONE_TEST_PREPARE && cargo_pending==1 && round_time==1000);
    stopped(); CHECK(servo_1.Angle==SERVO1_DROP && servo_2.Angle==SERVO2_LIFT);
    poll(APP_TASK_SERVO_WAIT_MS-1); CHECK(task_state==TASK_ZONE_TEST_PREPARE); stopped();
    poll(1); CHECK(task_state==TASK_ZONE_SEARCH); spinning();
}
static void lock_zone(float x,float y)
{ frame(10,team_zone_type,x,y); CHECK(task_state==TASK_ZONE_FOLLOW && !follow_forward); stopped(); }
static void align_zone(float y)
{
    lock_zone(APP_TASK_ZONE_AIM_X,y);
    frame(10,team_zone_type,APP_TASK_ZONE_AIM_X,y);
    CHECK(follow_forward && !zone_confirming); stopped();
}
static void drive_zone(float y)
{
    align_zone(y);
    frame(10,team_zone_type,APP_TASK_ZONE_AIM_X,y);
    frame(100,team_zone_type,APP_TASK_ZONE_AIM_X,y); stopped();
    frame(100,team_zone_type,APP_TASK_ZONE_AIM_X,y);
    CHECK(!eq(wheel_l,0) && eq(wheel_l,wheel_r));
}
static void confirm_zone(void)
{
    align_zone(APP_TASK_ZONE_AIM_Y);
    frame(10,team_zone_type,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y);
    CHECK(zone_confirming && zone_confirm_frames==0); stopped();
}

static void test_search_and_start(void)
{
    for(int team=0;team<2;++team){
        CASE(); start((uint8_t)team);
        uint8_t own=team?RESCUE_OBJECT_BLUE_SAFE_ZONE:RESCUE_OBJECT_RED_SAFE_ZONE;
        CHECK(team_zone_type==own);
        for(int type=0;type<=RESCUE_OBJECT_SAFE_ZONE_DIVIDER;++type){
            if(type==own){ continue; }
            frame(10,(uint8_t)type,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y);
            CHECK(task_state==TASK_ZONE_SEARCH); spinning();
        }
        send_ok=0; readable=0; poll(5000); CHECK(task_state==TASK_ZONE_SEARCH); spinning();
        tick=round_time+APP_TASK_ROUND_TIME_MS; App_Task_Proc(); CHECK(task_state==TASK_IDLE); stopped();
        App_Task_StartZoneTest((uint8_t)team); CHECK(task_state==TASK_IDLE && cargo_pending==1); stopped();
    }
    CASE(); reset(); App_Task_StartZoneTest(9); CHECK(task_state==TASK_IDLE && !round_started); stopped();
}

static void test_align_loss(void)
{
    CHECK(APP_TASK_ZONE_ALIGN_LOST_MS==200U);
    for(int recovery=0;recovery<3;++recovery){
        CASE(); start(APP_TASK_TEAM_RED); lock_zone(APP_TASK_ZONE_AIM_X+30,APP_TASK_ZONE_AIM_Y-70);
        frame(10,team_zone_type,APP_TASK_ZONE_AIM_X+30,APP_TASK_ZONE_AIM_Y-70);
        CHECK(wheel_l>0 && wheel_r<0 && !follow_forward);
        frame(10,0,0,0); CHECK(zone_missing); stopped();
        uint32_t lost=zone_missing_time; uint16_t oldview=view_id;
        poll(50); stopped(); readable=0; poll(50); stopped();
        CHECK(zone_missing_time==lost && heading_calls==0);
        if(recovery){
            // Keep the existing priority: a usable target on the deadline poll also recovers.
            readable=1; frame(recovery==1?99:100,team_zone_type,APP_TASK_ZONE_AIM_X-30,APP_TASK_ZONE_AIM_Y-70);
            CHECK(!zone_missing && !follow_forward && wheel_l<0 && wheel_r>0 && view_id==oldview);
        }else{
            poll(99); stopped(); CHECK(task_state==TASK_ZONE_FOLLOW);
            poll(1); CHECK(task_state==TASK_ZONE_SEARCH && view_id!=oldview); spinning();
            readable=1; lock_zone(APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y-70);
            CHECK(!zone_missing && !follow_forward && cargo_pending==1);
        }
    }
    CASE(); start(APP_TASK_TEAM_BLUE); lock_zone(APP_TASK_ZONE_AIM_X+30,300);
    poll(499); CHECK(!zone_missing); poll(1); CHECK(zone_missing); // stale duplicate triggers loss once.
    poll(199); CHECK(task_state==TASK_ZONE_FOLLOW); stopped();
    poll(1); CHECK(task_state==TASK_ZONE_SEARCH); spinning();
    CASE(); start(APP_TASK_TEAM_RED); lock_zone(APP_TASK_ZONE_AIM_X+30,APP_TASK_ZONE_AIM_Y-70);
    tick=UINT32_MAX-100U; round_time=tick-1000U; image_at(0,0,0); App_Task_Proc();
    uint32_t lost=zone_missing_time; stopped();
    poll(199); CHECK(task_state==TASK_ZONE_FOLLOW && zone_missing_time==lost); stopped();
    poll(1); CHECK(task_state==TASK_ZONE_SEARCH); spinning();
}

static void test_xy_loss(void)
{
    const float ys[]={APP_TASK_ZONE_AIM_Y-80,APP_TASK_ZONE_AIM_Y+40};
    for(int dir=0;dir<2;++dir){
        for(int recover=0;recover<3;++recover){
            CASE(); start(APP_TASK_TEAM_RED); drive_zone(ys[dir]);
            float speed=wheel_l; CHECK(dir?speed<0:speed>0);
            frame(10,0,0,0); uint32_t lost=zone_missing_time;
            CHECK(eq(heading_target,angle_yaw) && eq(held_speed,speed));
            poll(299); CHECK(eq(wheel_l,speed));
            if(recover==1){
                frame(0,team_zone_type,APP_TASK_ZONE_AIM_X,ys[dir]);
                CHECK(!zone_missing && follow_forward && eq(wheel_l,speed)); continue;
            }
            poll(1); stopped(); CHECK(zone_stopped);
            if(recover==2){
                frame(299,team_zone_type,APP_TASK_ZONE_AIM_X,ys[dir]);
                CHECK(!zone_missing && follow_forward && eq(wheel_l,speed)); continue;
            }
            poll(299); stopped(); CHECK(zone_missing_time==lost);
            poll(1); CHECK(task_state==TASK_ZONE_SEARCH); spinning();
        }
    }
    CASE(); start(APP_TASK_TEAM_RED); align_zone(APP_TASK_ZONE_AIM_Y-70);
    frame(10,0,0,0); CHECK(heading_calls==1 && held_speed==0); stopped();
    poll(300); stopped(); poll(300); spinning();
    CASE(); start(APP_TASK_TEAM_RED); drive_zone(APP_TASK_ZONE_AIM_Y-70);
    tick=UINT32_MAX-150; image_at(0,0,0); round_time=tick-1000; App_Task_Proc();
    poll(300); stopped(); poll(300); spinning(); CHECK(task_state==TASK_ZONE_SEARCH);
}

static void test_confirmation(void)
{
    for(unsigned mask=0;mask<8;++mask){
        CASE(); start(APP_TASK_TEAM_RED); confirm_zone();
        for(unsigned n=0;n<3;++n){
            unsigned good=mask&(1U<<n);
            frame(10,good?team_zone_type:0,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y);
            if(n<2){ CHECK(task_state==TASK_ZONE_FOLLOW && zone_confirm_frames==n+1); stopped(); }
        }
        if(mask){
            CHECK(task_state==TASK_ZONE_DONE && cargo_pending==1); stopped();
            frame(100,RESCUE_OBJECT_SAFE_ZONE_DIVIDER,10,10); stopped(); CHECK(task_state==TASK_ZONE_DONE);
            CHECK(servo_1.Angle==SERVO1_DROP);
        }else { CHECK(task_state==TASK_ZONE_SEARCH); spinning(); }
    }
    CASE(); start(APP_TASK_TEAM_RED); confirm_zone();
    for(int i=0;i<3;++i){ frame(10,team_zone_type,APP_TASK_ZONE_AIM_X+APP_TASK_ZONE_X_TOL+1,APP_TASK_ZONE_AIM_Y); }
    CHECK(task_state==TASK_ZONE_SEARCH); spinning();
    for(int good=0;good<2;++good){
        CASE(); start(APP_TASK_TEAM_BLUE); confirm_zone(); uint32_t begin=zone_confirm_time;
        frame(10,good?team_zone_type:0,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y);
        for(int i=0;i<30;++i){ poll(1); }
        CHECK(zone_confirm_frames==1);
        input.view_id++; poll(20); CHECK(zone_confirm_frames==1);
        input.flags=0; poll(20); CHECK(zone_confirm_frames==1);
        tick=begin+APP_TASK_ZONE_CONFIRM_MS-1; App_Task_Proc(); CHECK(task_state==TASK_ZONE_FOLLOW); stopped();
        input.flags=RESCUE_SCENE_IMAGE_VALID;
        frame(1,team_zone_type,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y);
        CHECK(task_state==TASK_ZONE_SEARCH); spinning();
    }
}

static void test_ranges_and_shared_controller(void)
{
    for(int dx=-1;dx<=1;dx+=2){
        for(int dy=-1;dy<=1;dy+=2){
            CASE(); start(APP_TASK_TEAM_RED);
            float x=APP_TASK_ZONE_AIM_X+dx*APP_TASK_ZONE_X_TOL;
            float y=APP_TASK_ZONE_AIM_Y+dy*APP_TASK_ZONE_Y_TOL;
            lock_zone(x,y); frame(10,team_zone_type,x,y); CHECK(follow_forward);
            frame(10,team_zone_type,x,y); CHECK(zone_confirming); stopped();
            for(int n=0;n<3;++n){ frame(10,team_zone_type,x,y); }
            CHECK(task_state==TASK_ZONE_DONE);
        }
    }
    CASE(); start(APP_TASK_TEAM_RED); align_zone(APP_TASK_ZONE_AIM_Y);
    frame(10,team_zone_type,APP_TASK_ZONE_AIM_X+APP_TASK_ZONE_X_TOL+1,APP_TASK_ZONE_AIM_Y);
    CHECK(!zone_confirming && wheel_l>0 && wheel_r<0);
    frame(10,team_zone_type,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y+APP_TASK_ZONE_Y_TOL+1);
    CHECK(!zone_confirming); stopped(); // unchanged Y hysteresis.
    CASE(); start(APP_TASK_TEAM_RED); lock_zone(APP_TASK_ZONE_AIM_X+40,APP_TASK_ZONE_AIM_Y-80);
    tick+=10; image_at(team_zone_type,APP_TASK_ZONE_AIM_X+40,APP_TASK_ZONE_AIM_Y-80);
    input.objects[1]=input.objects[0]; input.objects[0].type=RESCUE_OBJECT_SAFE_ZONE_DIVIDER;
    input.objects[0].cx=APP_TASK_ZONE_AIM_X; input.objects[0].cy=APP_TASK_ZONE_AIM_Y;
    input.object_count=2; App_Task_Proc(); CHECK(!follow_forward && wheel_l>0 && wheel_r<0);
    // A changed camera ID/size/resolution is not a zone association failure.
    tick+=10; image_at(team_zone_type,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y-80);
    input.image_width=1280; input.image_height=960; input.objects[0].cx*=2; input.objects[0].cy*=2;
    input.objects[0].width=180; input.objects[0].track_id=99;
    input.flags|=RESCUE_SCENE_TRUNCATED; App_Task_Proc(); CHECK(follow_forward && !zone_missing);
    CASE(); reset(); App_LinePID_Reset(); tick+=10;
    float zone_turn=App_LinePID_ComputeTarget(APP_TASK_ZONE_AIM_X+30,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_X_TOL,APP_TASK_ZONE_X_TOL);
    App_LinePID_Reset(); tick+=10; float green_turn=App_LinePID_Compute(LINE_PID_AIM_X+30);
    CHECK(eq(zone_turn,green_turn) && eq(green_turn,30*LINE_PID_KP));
    App_LinePID_Reset(); tick+=10; CHECK(eq(App_LinePID_Compute(LINE_PID_AIM_X),0));
}

typedef struct { int state; float left,right,servo; uint32_t elapsed; } Sample;
static unsigned replay(int test,Sample *out)
{
    reset();
    if(test){ App_Task_StartZoneTest(APP_TASK_TEAM_RED); poll(APP_TASK_SERVO_WAIT_MS); }
    else {
        App_Task_Start(APP_TASK_TEAM_RED);
        // Model completed capture; the real VERIFY branch supplies the common entry.
        App_Task_Move_Stop(); frame_drop_time=tick; App_Task_Frame_Set(1); cargo_pending=1; verify_attempt=1;
        App_Task_BeginVerify();
        for(int n=0;n<3;++n){ frame(10,RESCUE_OBJECT_SUPPLY,360,422); }
        CHECK(task_state==TASK_VERIFY && verify_result==APP_STEP_DONE); stopped();
        poll(APP_TASK_SERVO_WAIT_MS-30U);
    }
    CHECK(task_state==TASK_ZONE_SEARCH && tick==1500 && round_time==1000);
    unsigned count=0;
    for(unsigned i=0;i<16;++i){
        uint8_t type=(i==4 || i==5)?0:team_zone_type;
        float x=i<2?APP_TASK_ZONE_AIM_X+30:APP_TASK_ZONE_AIM_X;
        float y=i<10?APP_TASK_ZONE_AIM_Y-80:APP_TASK_ZONE_AIM_Y;
        frame(i==5?310:100,type,x,y);
        out[count++]=(Sample){task_state,wheel_l,wheel_r,servo_1.Angle,tick-round_time};
    }
    CHECK(task_state==TASK_ZONE_DONE); return count;
}
static void test_entry_parity_and_deadline(void)
{
    CASE(); Sample formal[16],test[16]; unsigned count=replay(0,formal); CHECK(replay(1,test)==count);
    for(unsigned i=0;i<count;++i){
        CHECK(formal[i].state==test[i].state && eq(formal[i].left,test[i].left)
              && eq(formal[i].right,test[i].right) && eq(formal[i].servo,test[i].servo)
              && formal[i].elapsed==test[i].elapsed);
    }
    for(int phase=0;phase<4;++phase){
        CASE(); start(APP_TASK_TEAM_RED);
        if(phase==1){ lock_zone(APP_TASK_ZONE_AIM_X+30,300); }
        if(phase>=2){ confirm_zone(); }
        if(phase==3){ for(int n=0;n<3;++n){ frame(10,team_zone_type,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y); } }
        tick=round_time+APP_TASK_ROUND_TIME_MS; image_at(team_zone_type,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y);
        App_Task_Proc(); CHECK(task_state==TASK_IDLE && cargo_pending==1); stopped();
        poll(1000); stopped(); CHECK(task_state==TASK_IDLE);
    }
    // Wrapper owns the loop after both completion and timeout and starts exactly once.
    CASE(); reset(); CHECK(App_Zone_Test(APP_TASK_TEAM_RED)==1); CHECK(task_state==TASK_ZONE_TEST_PREPARE);
    tick+=500; CHECK(App_Zone_Test(APP_TASK_TEAM_BLUE)==1); CHECK(team_zone_type==RESCUE_OBJECT_RED_SAFE_ZONE);
    tick=round_time+APP_TASK_ROUND_TIME_MS; CHECK(App_Zone_Test(APP_TASK_TEAM_RED)==1); stopped();
    CHECK(App_Zone_Test(APP_TASK_TEAM_RED)==1); CHECK(task_state==TASK_IDLE && rx_calls==4 && debug_calls==4);
}

int main(void)
{
    test_search_and_start(); test_align_loss(); test_xy_loss(); test_confirmation();
    test_ranges_and_shared_controller(); test_entry_parity_and_deadline();
    printf("PASS zone: %d scenarios, %d checks\n",cases,checks); return 0;
}
