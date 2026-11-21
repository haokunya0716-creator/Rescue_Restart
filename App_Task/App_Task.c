//
// Created by gaoxu on 2026/11/21.
//

#include "main.h"
#include "App_Task.h"

#include "app_button.h"
#include "app_motor.h"
#include "app_servo.h"
#include "app_speed.h"
#include "drv8701.h"
#include "usart.h"

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

    App_Servo_Init();
    App_Button_Init();

}

void App_Task_Pro(void) {

}
