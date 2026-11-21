#ifndef INTELLIGENT_RESCUE_APP_VISION_TEST_H
#define INTELLIGENT_RESCUE_APP_VISION_TEST_H

#include "main.h"
#include "app_task_config.h"
#include "app_usart.h"
#include "rescue_usart2.h"

/* main.c包含本头文件，在Rescue_Usart2_Process()之后调用App_Vision_Test()。
 * 不测试时只需注释这一行调用；本函数只读场景，不控制电机、舵机或视角。
 * 串口1每50 ms打印最新有效场景的全部条目，不按类型、置信度或任务目标筛选。
 * 中心x及框宽按640宽归一化，中心y及框高按480高归一化，保留1位小数。
 * 每组首行分别标明相机原始分辨率image和归一化分辨率norm。
 * 使用毫秒差值计时，不加HAL_Delay；阻塞串口发送较多条目时，实际周期会延长。
 */
static inline void App_Vision_Test(void)
{
    static RescueScene scene;
    static uint32_t last_print_ms;
    uint32_t now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - last_print_ms) < 50U) {
        return;
    }
    last_print_ms = now_ms;

    if (Rescue_Usart2_GetScene(&scene) == 0U ||
        scene.image_width == 0U || scene.image_height == 0U) {
        App_USART1_Printf("[VISION] no valid scene\r\n");
        return;
    }

    float scale_x = APP_TASK_IMAGE_W / scene.image_width;
    float scale_y = APP_TASK_IMAGE_H / scene.image_height;

    App_USART1_Printf("[VISION] frame=%u view=%u image=%ux%u norm=%.0fx%.0f count=%u flags=0x%02X\r\n",
                      (unsigned int)scene.frame_id, (unsigned int)scene.view_id,
                      (unsigned int)scene.image_width, (unsigned int)scene.image_height,
                      APP_TASK_IMAGE_W, APP_TASK_IMAGE_H,
                      (unsigned int)scene.object_count, (unsigned int)scene.flags);
    for (uint8_t i = 0U; i < scene.object_count; i++) {
        const RescueObject *object = &scene.objects[i];
        App_USART1_Printf("[OBJ] i=%u type=%u conf=%u x=%.1f y=%.1f w=%.1f h=%.1f id=%u\r\n",
                          (unsigned int)i, (unsigned int)object->type,
                          (unsigned int)object->confidence,
                          object->cx * scale_x, object->cy * scale_y,
                          object->width * scale_x, object->height * scale_y,
                          (unsigned int)object->track_id);
    }
}

#endif
