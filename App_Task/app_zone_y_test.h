#ifndef INTELLIGENT_RESCUE_APP_ZONE_Y_TEST_H
#define INTELLIGENT_RESCUE_APP_ZONE_Y_TEST_H

#include "main.h"

/* Ozone中展开zone_y_tune修改；全部为RAM变量，复位后恢复编译初值。 */
typedef struct {
    uint32_t enable;        // 1运行，0立即停车；仍处理视觉接收。
    uint32_t reset;         // 写1清除PID历史，程序自动清0；不重置180秒计时。
    float kp;              // cm/s/px。
    float ki;              // cm/s/(px*s)。
    float kd;              // cm/s/(px/s)。
    float target_y;        // 整区框中心Y目标，归一化到480高。
    float tolerance;       // |Y误差|不超过此值时停车。
    float min_speed;       // 非零PID输出的最小速度幅值，cm/s。
    float max_forward;     // 前进速度上限，cm/s；0禁止前进。
    float max_reverse;     // 倒车速度幅值上限，cm/s；0禁止倒车。
    float integral_limit;  // I项输出幅值上限，cm/s。
} App_ZoneY_Tune;

typedef enum {
    APP_ZONE_Y_DISABLED = 0,
    APP_ZONE_Y_PREPARE = 1,
    APP_ZONE_Y_WAIT_TARGET = 2,
    APP_ZONE_Y_RUNNING = 3,
    APP_ZONE_Y_IN_BAND = 4,
    APP_ZONE_Y_BAD_CONFIG = 5,
    APP_ZONE_Y_TIMEOUT = 6
} App_ZoneY_Status;

/* 观察值；speed为下发给两轮的目标速度，实速见speed_l/r_measure。 */
typedef struct {
    uint32_t status;        // 对应App_ZoneY_Status。
    uint32_t valid;         // 1表示当前控制有有效本队整区；0时y/error仅为旧记录。
    uint32_t in_band;       // 1表示当前Y在停车范围内。
    uint32_t frame_id;
    uint32_t view_id;       // 相机必须返回此视角编号。
    float y;
    float error;            // target_y-y；正值通常对应前进。
    float dt;               // 两次有效新图控制运算的间隔，单位s。
    float p;
    float i;
    float d;
    float raw_speed;        // P+I+D，限幅前。
    float speed;            // 限幅后的左右轮共同目标，cm/s。
} App_ZoneY_Watch;

extern volatile App_ZoneY_Tune zone_y_tune;
extern volatile App_ZoneY_Watch zone_y_watch;

/* while(1)开头单独调用：只前后控制Y，不搜索旋转、不控制X、不调用正式任务。
 * 第一次调用落框/设置摄像头，等待现有舵机时间后自动开始；始终返回1接管循环。
 * team传APP_TASK_TEAM_RED/BLUE，启动后固定；180秒后停车，重新测试需复位。
 */
uint8_t App_ZoneY_Test(uint8_t team);

#endif
