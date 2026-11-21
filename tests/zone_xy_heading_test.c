/* Real independent test controller and visual PID, with hardware I/O stubbed. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "../App_Task/app_zone_xy_heading_test.c"

static uint32_t tick;
static RescueScene input;
static uint8_t team_arg;
static float target_arg, wheel_l, wheel_r, frame_angle, camera_angle;
static int readable, send_ok, stops, zero_commands, cases, checks;
float angle_yaw, angle_yaw_ref, motor_turn_speed, position_mid;
#define CHECK(x) do { ++checks; if(!(x)){ fprintf(stderr,"line %d: %s\n",__LINE__,#x); abort(); } } while(0)
#define CASE() (++cases)
static int eq(float a,float b) { return fabsf(a-b)<0.001f; }
uint32_t HAL_GetTick(void) { return tick; }
uint64_t gx_GetUs(void) { return (uint64_t)tick*1000U; }
void App_Task_Move_Stop(void) { ++stops; wheel_l=wheel_r=0; }
void App_Task_Move_Speed(float l,float r) { wheel_l=l; wheel_r=r; if(l==0 && r==0){ ++zero_commands; } }
void App_Task_Move_Proc(void) {}
void App_Task_Frame_Set(uint8_t down) { frame_angle=down?SERVO1_DROP:SERVO1_LIFT; }
void App_Task_Camera_Set(uint8_t down) { camera_angle=down?SERVO2_DROP:SERVO2_LIFT; }
void App_Update_Data(void) {}
void Set_Angle_Target(float target) { angle_yaw_ref=target; }
void App_Heading_Pro(float speed)
{
    motor_turn_speed=0.9f*ZoneXY_Wrap(angle_yaw_ref-angle_yaw);
    App_Task_Move_Speed(speed-motor_turn_speed,speed+motor_turn_speed);
}
void Rescue_Usart2_Process(void) {}
void Rescue_Usart2_Debug_Proc(void) {}
uint8_t Rescue_Usart2_SendViewContext(uint16_t id,int16_t yaw) { (void)id; (void)yaw; return send_ok; }
uint8_t Rescue_Usart2_GetScene(RescueScene *out)
{
    if(!readable || !(input.flags&RESCUE_SCENE_IMAGE_VALID)
       || (uint32_t)(tick-input.received_ms)>=RESCUE_VISION_TIMEOUT_MS){ return 0U; }
    *out=input; return 1U;
}
static void poll(uint32_t dt)
{ tick+=dt; CHECK(App_ZoneXYHeading_Test(team_arg,target_arg)==1U); }
static void image_at(uint8_t type,float x,float y)
{
    ++input.frame_id; input.capture_ms=input.received_ms=tick; input.view_id=view_id;
    input.object_count=type?1:0;
    input.objects[0]=(RescueObject){.type=type,.cx=(uint16_t)x,.cy=(uint16_t)y,.width=40,.height=40};
}
static void frame(uint32_t dt,uint8_t type,float x,float y)
{ tick+=dt; image_at(type,x,y); poll(0); }
static void spin(void)
{ CHECK(eq(wheel_l,APP_TASK_ZONE_SPIN_SPEED) && eq(wheel_r,-APP_TASK_ZONE_SPIN_SPEED)); }
static void stopped(void) { CHECK(eq(wheel_l,0) && eq(wheel_r,0)); }
static void start(uint8_t team,float target)
{
    started=zone_type=frame_seen=view_sent=0;
    view_id=last_frame=0;
    start_time=view_time=last_capture=missing_time=0;
    fixed_yaw=base_speed=return_yaw=motion_yaw=shift_cm=shift_start=previous_yaw_error=0;
    zone_xy_heading_watch=(App_ZoneXYHeading_Watch){0};
    phase=APP_ZONE_XY_PREPARE; tick=1000; readable=send_ok=1;
    wheel_l=wheel_r=angle_yaw=angle_yaw_ref=motor_turn_speed=position_mid=0;
    stops=zero_commands=0; team_arg=team; target_arg=target;
    input=(RescueScene){.image_width=640,.image_height=480,.flags=RESCUE_SCENE_IMAGE_VALID};
    App_LinePID_Init(); poll(0);
    CHECK(phase==APP_ZONE_XY_PREPARE && eq(frame_angle,SERVO1_DROP) && eq(camera_angle,SERVO2_LIFT));
    poll(APP_TASK_SERVO_WAIT_MS-1); stopped(); CHECK(phase==APP_ZONE_XY_PREPARE);
    poll(1); CHECK(phase==APP_ZONE_XY_SEARCH); spin();
}

static void test_search_and_xy(void)
{
    for(int team=0;team<2;++team){
        CASE(); start((uint8_t)team,0);
        int before=stops;
        frame(1,team?RESCUE_OBJECT_RED_SAFE_ZONE:RESCUE_OBJECT_BLUE_SAFE_ZONE,360,140); spin();
        frame(1,zone_type,APP_TASK_ZONE_AIM_X+30,APP_TASK_ZONE_AIM_Y-50);
        CHECK(phase==APP_ZONE_XY_ALIGN && wheel_l>0 && wheel_r<0);
        frame(1,zone_type,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y-50);
        CHECK(phase==APP_ZONE_XY_FOLLOW && eq(wheel_l,LINE_PID_Y_MAX_FORWARD) && eq(wheel_l,wheel_r));
        CHECK(stops==before && zero_commands==0); // Same frame starts Y; no 200ms/3-frame pause.
        frame(1,zone_type,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y+APP_TASK_ZONE_Y_TOL+1);
        CHECK(wheel_l<0 && eq(wheel_l,wheel_r)); // Direct reversal, no extra hysteresis.
        CHECK(stops==before && zero_commands==0);
        frame(1,zone_type,APP_TASK_ZONE_AIM_X+30,APP_TASK_ZONE_AIM_Y);
        CHECK(wheel_l>0 && wheel_r<0 && stops==before);
        frame(1,0,0,0); CHECK(phase==APP_ZONE_XY_SEARCH); spin(); CHECK(stops==before);
    }
    CASE(); start(APP_TASK_TEAM_RED,0);
    tick++; image_at(zone_type,APP_TASK_ZONE_AIM_X+40,APP_TASK_ZONE_AIM_Y-80);
    input.objects[1]=input.objects[0]; input.objects[1].cx=APP_TASK_ZONE_AIM_X-40; input.object_count=2;
    poll(0); CHECK(wheel_l>0 && wheel_r<0 && eq(zone_xy_heading_watch.x,APP_TASK_ZONE_AIM_X+40));
    tick++; image_at(zone_type,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y-80);
    input.image_width=1280; input.image_height=960; input.objects[0].cx*=2; input.objects[0].cy*=2;
    poll(0); CHECK(phase==APP_ZONE_XY_FOLLOW && eq(wheel_l,LINE_PID_Y_MAX_FORWARD));
}

static void test_align_loss(void)
{
    for(int recover=0;recover<3;++recover){
        CASE(); start(APP_TASK_TEAM_RED,0);
        frame(1,zone_type,APP_TASK_ZONE_AIM_X+30,APP_TASK_ZONE_AIM_Y-50);
        frame(1,0,0,0); uint32_t lost=missing_time; uint16_t oldview=view_id;
        CHECK(phase==APP_ZONE_XY_ALIGN_LOST); stopped();
        poll(100); CHECK(missing_time==lost); stopped();
        if(recover){
            frame(recover==1?99:100,zone_type,APP_TASK_ZONE_AIM_X-30,APP_TASK_ZONE_AIM_Y-50);
            CHECK(phase==APP_ZONE_XY_ALIGN && view_id==oldview && wheel_l<0 && wheel_r>0);
        }else{
            poll(99); CHECK(phase==APP_ZONE_XY_ALIGN_LOST); stopped();
            poll(1); CHECK(phase==APP_ZONE_XY_SEARCH && view_id!=oldview); spin();
        }
    }
    CASE(); start(APP_TASK_TEAM_RED,0);
    frame(1,zone_type,APP_TASK_ZONE_AIM_X+30,APP_TASK_ZONE_AIM_Y-50);
    tick=UINT32_MAX-100U; start_time=tick-1000U;
    frame(0,0,0,0); poll(199); CHECK(phase==APP_ZONE_XY_ALIGN_LOST); stopped();
    poll(1); CHECK(phase==APP_ZONE_XY_SEARCH); spin();
}

static void test_xy_hold_and_communications(void)
{
    for(int reverse=0;reverse<2;++reverse){
        for(int recover=0;recover<2;++recover){
            CASE(); start(APP_TASK_TEAM_RED,0);
            float y=APP_TASK_ZONE_AIM_Y+(reverse?40:-50);
            frame(1,zone_type,APP_TASK_ZONE_AIM_X,y); float speed=wheel_l; int before=stops;
            angle_yaw=17; frame(1,0,0,0);
            CHECK(phase==APP_ZONE_XY_HOLD && eq(angle_yaw_ref,17) && eq(wheel_l,speed));
            poll(299); CHECK(phase==APP_ZONE_XY_HOLD && eq(wheel_l,speed));
            if(recover){ frame(0,zone_type,APP_TASK_ZONE_AIM_X,y); CHECK(phase==APP_ZONE_XY_FOLLOW); }
            else { poll(1); CHECK(phase==APP_ZONE_XY_SEARCH); spin(); }
            CHECK(stops==before && zero_commands==0);
        }
    }
    for(int fault=0;fault<4;++fault){
        CASE(); start(APP_TASK_TEAM_RED,0);
        frame(1,zone_type,APP_TASK_ZONE_AIM_X+30,APP_TASK_ZONE_AIM_Y-50); int before=stops;
        if(fault==0){ readable=0; poll(1); }
        if(fault==1){ input.view_id++; poll(1); }
        if(fault==2){ send_ok=0; poll(APP_TASK_VIEW_RETRY_MS); }
        if(fault==3){ poll(499); CHECK(phase==APP_ZONE_XY_ALIGN); poll(1); }
        CHECK(phase==APP_ZONE_XY_SEARCH && stops==before); spin();
        readable=send_ok=1; frame(1,zone_type,APP_TASK_ZONE_AIM_X-30,APP_TASK_ZONE_AIM_Y-50);
        CHECK(phase==APP_ZONE_XY_ALIGN && wheel_l<0 && wheel_r>0 && stops==before);
    }
}

static void test_side_shift_and_completion(void)
{
    for(int direction=-1;direction<=1;direction+=2){
        CASE(); start(APP_TASK_TEAM_RED,0); angle_yaw=direction*20;
        int before=stops;
        frame(1,zone_type,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y);
        CHECK(phase==APP_ZONE_XY_TURN_OUT && eq(motion_yaw,direction*110));
        CHECK(eq(fixed_yaw,0) && stops==before);
        angle_yaw=motion_yaw; frame(1,0,0,0); // Deliberate loss during maneuver does not interrupt.
        CHECK(phase==APP_ZONE_XY_SHIFT && eq(wheel_l,LINE_PID_Y_MIN_SPEED));
        position_mid=APP_ZONE_XY_HEADING_SHIFT_CM-0.1f; poll(1); CHECK(phase==APP_ZONE_XY_SHIFT);
        position_mid=APP_ZONE_XY_HEADING_SHIFT_CM; poll(1); CHECK(phase==APP_ZONE_XY_TURN_BACK);
        CHECK(eq(motion_yaw,direction*20) && stops==before && zero_commands==0);
        angle_yaw=motion_yaw; poll(1); CHECK(phase==APP_ZONE_XY_SEARCH); spin();
        CHECK(stops==before && zero_commands==0);
        angle_yaw=-direction*5;
        frame(1,zone_type,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y);
        CHECK(phase==APP_ZONE_XY_TURN_OUT && eq(shift_cm,APP_ZONE_XY_HEADING_SHIFT_CM*0.5f));
        readable=0; poll(1); CHECK(phase==APP_ZONE_XY_SEARCH); spin(); // F5 overrides a maneuver.
        readable=1; angle_yaw=0;
        frame(1,zone_type,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y);
        CHECK(phase==APP_ZONE_XY_DONE); stopped();
        frame(1,zone_type,0,0); CHECK(phase==APP_ZONE_XY_DONE); stopped();
    }
    CASE(); start(APP_TASK_TEAM_RED,179); angle_yaw=-179;
    frame(1,zone_type,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y);
    CHECK(phase==APP_ZONE_XY_DONE && eq(zone_xy_heading_watch.yaw_error,-2)); stopped();
    CASE(); start(APP_TASK_TEAM_RED,0); angle_yaw=-179;
    frame(1,zone_type,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y);
    readable=0; poll(1); readable=1; angle_yaw=179;
    frame(1,zone_type,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y);
    CHECK(eq(shift_cm,APP_ZONE_XY_HEADING_SHIFT_CM)); // Wrap crossing must not halve step.
}

static void test_no_extra_failure_and_deadline(void)
{
    CASE(); start(APP_TASK_TEAM_RED,0); angle_yaw=20;
    frame(1,zone_type,APP_TASK_ZONE_AIM_X,APP_TASK_ZONE_AIM_Y); int before=stops;
    for(int i=0;i<10;++i){ frame(1000,0,0,0); CHECK(phase==APP_ZONE_XY_TURN_OUT && stops==before); }
    angle_yaw=motion_yaw; frame(1,0,0,0); CHECK(phase==APP_ZONE_XY_SHIFT);
    for(int i=0;i<10;++i){ frame(1000,0,0,0); CHECK(phase==APP_ZONE_XY_SHIFT && stops==before); }
    position_mid=-10; frame(1,0,0,0); CHECK(phase==APP_ZONE_XY_SHIFT); // No encoder-failure branch.
    tick=start_time+APP_TASK_ROUND_TIME_MS; poll(0); CHECK(phase==APP_ZONE_XY_TIMEOUT); stopped();
    poll(1); CHECK(phase==APP_ZONE_XY_TIMEOUT); stopped();
    CASE(); start(APP_TASK_TEAM_BLUE,45);
    team_arg=APP_TASK_TEAM_RED; target_arg=-80; poll(1);
    CHECK(zone_type==RESCUE_OBJECT_BLUE_SAFE_ZONE && eq(fixed_yaw,45));
    tick=start_time+APP_TASK_ROUND_TIME_MS; poll(0); CHECK(phase==APP_ZONE_XY_TIMEOUT); stopped();
}

int main(void)
{
    CHECK(APP_TASK_ZONE_ALIGN_LOST_MS==200U);
    test_search_and_xy(); test_align_loss(); test_xy_hold_and_communications();
    test_side_shift_and_completion(); test_no_extra_failure_and_deadline();
    printf("PASS zone_xy_heading: %d scenarios, %d checks\n",cases,checks);
    return 0;
}
