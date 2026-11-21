//
// Created by gaoxu on 2026/10/20.
//

#include "app_task_vision.h" // 引入视觉选择、跟随和复检接口。
#include "app_line_pid.h" // 引入视觉纠偏PID及死区接口。
#include "app_servo.h" // 引入舵机结构体及角度设置接口。
#include "app_usart.h" // 引入串口1日志接口，仅在跟随失败时打印原因。
#include "rescue_usart2.h" // 引入相机场景读取及视角发送接口。
#include "app_motor.h" // 整区XY接近暂失期间保持当前航向。
#include <string.h> // 只用于诊断原因去重及日志分组。

static RescueScene task_scene;       // 相机原始像素快照，不是厘米坐标
static RescueObject target;          // 保存目标类型及检测框，不保存列表下标
static uint16_t target_image_w; // 最近成功匹配目标所属图像宽度，用于跨分辨率关联。
static uint16_t target_image_h; // 最近成功匹配目标所属图像高度。
static uint16_t view_id; // 当前视角编号，换阶段时递增以排除旧视角图像。
static uint16_t last_frame_id; // 最近已处理图像的帧编号。
static uint32_t last_capture_ms; // 最近已处理图像的相机采集时间，与帧号一起用于去重。
static uint32_t view_begin_time; // 当前视角开始时刻，供画面稳定等待使用。
static uint32_t view_settle_ms; // 本次新视角的画面稳定等待；复检和后退后搜索为0，其它搜索保留配置值。
static uint32_t view_time; // 上次视角上下文发送尝试时刻，限制重发频率。
static uint8_t view_sent; // 当前视角上下文是否已经成功发送。
static uint8_t frame_seen; // 本阶段是否接受过图像，静态初值为0。
static uint8_t verify_frames; // 当前复检窗口已检查的有效新图数量，空区域也占一张，重复帧不累计。
static uint8_t verify_hits; // 当前三张复检新图中，区域内恰好一个绿色的合格张数。
static App_Task_VerifyFailure verify_failure; // 当前复检失败分类，新视角开始时清零。
static uint8_t target_mixed; // 当前锁定是否用于撞散，锁定后不因列表变化而改变用途。
static uint8_t target_zone; // 已锁定本队整块安全区；不跟随分区或分界线。
static uint8_t zone_missing, zone_stopped; // 本次丢失等待，以及接近丢失是否已进入停车等待。
static uint32_t zone_missing_time; // 首次判定丢失的时间；坏图和重复图不重置。
static float zone_drive_speed; // 最近一次实际纵向速度，允许为0或倒车。
static uint8_t zone_confirming, zone_confirm_frames, zone_confirm_hits;
static uint32_t zone_confirm_time; // 停车后三图确认的总期限起点。
static uint8_t follow_forward;        // 首次对准结束后置1，接近时不再置0
static int8_t capture_y_direction; // 单独绿色纵向实际控制方向：前进1、倒车-1、停车0。
static int8_t capture_y_request; // 等待稳定确认的单独绿色纵向方向。
static uint8_t capture_y_frames; // 同一请求方向连续出现的新图计数。
static uint32_t capture_y_wait_time; // 进入停车/换向等待的起始时刻。
static uint8_t team_zone_type; // 本队整块安全区的视觉协议类型，红5/蓝6。
static float camera_angle = SERVO2_LIFT; // 本阶段期望的摄像头俯角，默认正常观察角，并非实测反馈。
static uint8_t search_empty_frames;   // 本侧连续无候选的新帧数量
static uint8_t lost_frames;           // 只统计未匹配原目标的有效新帧
static uint32_t lost_time;            // 第一次短暂丢失时间，不延长总接近超时
static uint8_t green_waiting;         // 坏图和有效新图失配共用等待起点，独立于丢失帧数
static uint8_t arrive_frames;         // 停车后连续到位确认，不重复计算同帧
static uint32_t arrive_time; // 当前一轮到位停车确认的计时起点。
static float arrive_x; // 当前一轮到位确认的目标中心x位置基准。
static float arrive_y; // 当前一轮到位确认的目标中心y位置基准。

// 以下字段仅保存诊断现场，不参与动作通过/失败的判断。
static const char *scene_reason = "not_checked";
static const char *match_reason = "not_checked";
static const char *cargo_reason = "not_checked";
static const char *scatter_reason = "not_checked";
static const char *verify_wait = "new_view";
static const char *select_wait = "new_view";
static const char *verify_reported;
static const char *select_reported;
static uint8_t scene_snapshot; // 1表示本次读图取得了快照；0表示task_scene仅是上次保存值。
static uint8_t view_send_failed;
static uint32_t checked_age; // 保存时效检查实际使用的合计年龄。
static uint8_t match_reject[5]; // 依次为类别、ID、宽、高、位移拒绝数量。
static float match_best_score, match_second_score;
static RescueObject diagnostic_target;
static int cargo_bad_index = -1;
static int scatter_bad_index = -1;
static uint8_t cargo_count;
static uint16_t verify_new_total, verify_duplicates, verify_invalid;

// 同一种持续等待只打印首次或原因变化；正常重复帧不周期打印。
static void App_Task_LogWait(const char *stage, const char *reason, const char **reported)
{
    if(APP_TASK_DEBUG_ENABLE && (*reported == 0 || strcmp(*reported, reason) != 0)){
        App_USART1_Printf("[%s] wait=%s\r\n", stage, reason);
    }
    *reported = reason;
}

// 按原始场景下标读取物块，不生成本地分区框。
static const RescueObject *App_Task_ObjectAt(uint8_t index)
{
    return &task_scene.objects[index];
}

// @简介：返回浮点绝对值，用于像素门限和舵机角度比较。
static float App_Task_Abs(float value) // 函数入口：返回浮点绝对值，用于像素门限和舵机角度比较。
{ // 开始App_Task_Abs函数体。
    return value < 0.0f ? -value : value; // 负值取反、非负值保持，返回浮点绝对值。
} // 结束App_Task_Abs函数体。

// @简介：把目标中心 x 换算到 640 宽度；原始检测坐标不变。
static float App_Task_X(const RescueObject *object) // 函数入口：把目标中心 x 换算到 640 宽度；原始检测坐标不变。
{ // 开始App_Task_X函数体。
    return object->cx * APP_TASK_IMAGE_W / task_scene.image_width; // 将原始中心x按实际图像宽度缩放到参考图像坐标。
} // 结束App_Task_X函数体。

// @简介：把目标中心 y 换算到 480 高度；用于停车，不使用检测框底边。
static float App_Task_Y(const RescueObject *object) // 函数入口：把目标中心 y 换算到 480 高度；用于停车，不使用检测框底边。
{ // 开始App_Task_Y函数体。
    return object->cy * APP_TASK_IMAGE_H / task_scene.image_height; // 将原始中心y按实际图像高度缩放到参考图像坐标。
} // 结束App_Task_Y函数体。

// @简介：把框底边换算到 480 高度，只用于候选排序和通道阻挡判断。
static float App_Task_Bottom(const RescueObject *object) // 函数入口：把框底边换算到 480 高度，只用于选择、通道检查和边缘裁切判断。
{ // 开始App_Task_Bottom函数体。
    return (object->cy - object->height / 2U + object->height) * APP_TASK_IMAGE_H // 以中心减半高再加整高计算框底边，并乘参考图像高度。
           / task_scene.image_height; // 除以实际图像高度，完成框底边y归一化。
} // 结束App_Task_Bottom函数体。

// @简介：画面稳定后发送/重发当前编号；相机必须在采集图像时锁存编号。
static void App_Task_UpdateView(void) // 函数入口：画面稳定后发送/重发当前编号；相机必须在采集图像时锁存编号。
{ // 开始App_Task_UpdateView函数体。
    uint32_t now_ms = HAL_GetTick(); // 读取当前毫秒计时，作为本次处理的时间基准。
    if((uint32_t)(now_ms - view_begin_time) < view_settle_ms){ return; } // 按本次视角的等待时间判断；传0的阶段立即发送上下文。
    if(view_sent == 0 || (uint32_t)(now_ms - view_time) >= APP_TASK_VIEW_RETRY_MS){ // 首次未发送成功或已经到达重发间隔时，再尝试发送。
        view_sent = Rescue_Usart2_SendViewContext(view_id, 0);
        view_send_failed = view_sent == 0; // 保存发送失败，交给调用方在停车后或等待事件中打印。 // 发送当前视角上下文，并记录串口发送接口是否接受成功。
        view_time = now_ms; // 记录本次发送尝试的时刻，供下一次重发间隔判断。
    } // 结束条件分支。
} // 结束App_Task_UpdateView函数体。

// @简介：当前图像是否为本阶段未处理的新帧；同一帧不能重复累计复检次数。
static uint8_t App_Task_NewFrame(void) // 函数入口：当前图像是否为本阶段未处理的新帧；同一帧不能重复累计复检次数。
{ // 开始App_Task_NewFrame函数体。
    if(frame_seen != 0 && task_scene.frame_id == last_frame_id // 本阶段已经见过图像且帧编号相同时，继续比较采集时间。
       && task_scene.capture_ms == last_capture_ms){ return 0; } // 帧编号及采集时间都相同则为重复图像，返回0不重复累计。
    frame_seen = 1; // 记录本阶段已经接受过图像。
    last_frame_id = task_scene.frame_id; // 保存最近处理帧的编号，用于识别重复图像。
    last_capture_ms = task_scene.capture_ms; // 保存最近处理帧的相机采集时间，与帧号共同去重。
    return 1; // 当前检查通过，返回1表示本次条件成立。
} // 结束App_Task_NewFrame函数体。

// @简介：检查本地接收年龄；扩展帧再检查采集至发送、串口及本地等待的合计估计年龄。
// @说明：扩展帧额外检查相机耗时+估算串口耗时；旧帧无相机耗时，不能声称图像真实年龄合格。
static uint8_t App_Task_ImageFresh(void)
{
    uint32_t age = (uint32_t)(HAL_GetTick() - task_scene.received_ms);
    uint32_t wire_size = RESCUE_FRAME_OVERHEAD + RESCUE_EXT_HEADER_SIZE
                         + task_scene.object_count * RESCUE_EXT_OBJECT_SIZE;
    checked_age = age;
    if(age >= APP_TASK_TRACK_MAX_AGE_MS){ scene_reason = "receive_age"; return 0; }
    if(task_scene.has_source_age == 0){ return 1; }
    age += task_scene.source_age_ms
           + (wire_size * 10000U + APP_TASK_SERIAL_BAUD - 1U) / APP_TASK_SERIAL_BAUD;
    checked_age = age;
    if(age >= APP_TASK_IMAGE_MAX_AGE_MS){ scene_reason = "source_age"; }
    return age < APP_TASK_IMAGE_MAX_AGE_MS;
}

// @简介：读取完整、当前视角、新鲜场景；无效、过期或摄像头设定角改变返回0。
static uint8_t App_Task_ReadScene(void)
{
    App_Task_UpdateView(); // 保留原发送/重发时序和短路检查顺序。
    scene_snapshot = 0;
    if(view_sent == 0){ scene_reason = view_send_failed ? "view_send_failed" : "view_not_sent"; return 0; }
    if(Rescue_Usart2_GetScene(&task_scene) == 0){ scene_reason = "scene_unavailable"; return 0; }
    scene_snapshot = 1;
    if(task_scene.view_id != view_id){ scene_reason = "view_mismatch"; return 0; }
    if((task_scene.flags & RESCUE_SCENE_IMAGE_VALID) == 0){ scene_reason = "image_invalid"; return 0; }
    if((task_scene.flags & RESCUE_SCENE_TRUNCATED) != 0){ scene_reason = "list_truncated"; return 0; }
    if(App_Task_ImageFresh() == 0){ return 0; }
    if(!(App_Task_Abs(servo_2.Angle - camera_angle) < APP_TASK_SERVO_TOL)){
        scene_reason = "camera_angle"; return 0; // 保留原浮点比较语义，包含非数值不通过的情况。
    }
    scene_reason = "ok";
    return 1;
}

// @简介：撞散的可见场景检查；最多3件普通/核心物资，其它实体/区域均拒绝。
// @说明：旋转期间也检查整个可见画面；看不到的物块和场地边界仍需后续感知补充。
static uint8_t App_Task_ScatterSceneClear(void)
{
    uint8_t count = 0;
    scatter_bad_index = -1;
    scatter_reason = "ok";
    for(uint8_t i = 0; i < task_scene.object_count; ++i){
        const RescueObject *object = &task_scene.objects[i];
        if(object->type == RESCUE_OBJECT_START_ZONE){ continue; }
        if(object->type != RESCUE_OBJECT_SUPPLY && object->type != RESCUE_OBJECT_CORE_SUPPLY){
            scatter_reason = "forbidden_type"; scatter_bad_index = i; return 0;
        }
        if(++count > APP_TASK_SCATTER_MAX_COUNT){
            scatter_reason = "too_many_supplies"; scatter_bad_index = i; return 0;
        }
    }
    return 1;
}

// @简介：锁存目标并复位视觉PID，从下一幅新图开始首次对准；zone=1表示本队整块安全区。
static void App_Task_Lock(uint8_t index, uint8_t zone, uint8_t mixed) // 函数入口：锁存目标并复位视觉PID，从下一幅新图开始首次对准；zone=1表示本队整块安全区。
{ // 开始App_Task_Lock函数体。
    target = *App_Task_ObjectAt(index); // 保存当前目标，不依赖列表下标保持身份。
    target_image_w = task_scene.image_width; // 保存该目标原始坐标所属的图像尺寸。
    target_image_h = task_scene.image_height;
    last_frame_id = task_scene.frame_id; // 保存最近处理帧的编号，用于识别重复图像。
    last_capture_ms = task_scene.capture_ms; // 保存最近处理帧的相机采集时间，与帧号共同去重。
    frame_seen = 1; // 记录本阶段已经接受过图像。
    target_zone = zone; // 安全区只接近整区，使用独立丢失恢复和三图确认。
    target_mixed = mixed; // 记录本次锁定是单件救援还是物资堆撞散用途。
    follow_forward = 0; // 新锁定目标先进入首次对准，尚未开始持续前进。
    capture_y_direction = 0; // 新目标从停车状态启动纵向控制。
    capture_y_request = 0; // 清除上一个目标遗留的待恢复方向。
    capture_y_frames = 0; // 清空待恢复方向的连续新图计数。
    lost_frames = 0; // 清零原目标连续未匹配的新帧计数。
    green_waiting = 0;
    arrive_frames = 0; // 清零连续到位确认帧数，禁止继承旧确认结果。
    App_LinePID_Reset(); // 清空视觉PID历史误差及死区状态，避免旧输出干扰新阶段。
} // 结束App_Task_Lock函数体。

// @简介：在新图像中关联已锁定目标（普通/核心物资），失败/歧义返回-1。
// @说明：比较本帧同类候选与上一次成功匹配目标的(x,y)，选择平方距离最小者；
//        不按列表下标、离屏幕中心远近或单独比较x重新选目标，y方向同样参与判断。
//        默认位移最多70px、框宽高变化不超过两倍；有非零相机ID时还必须同ID。
//        最近/次近候选得分难以区分时返回失败，交给连续丢失确认，不能强行换锁。
static int App_Task_MatchTarget(void) // 函数入口：在新图像中关联已锁定目标（普通/核心物资），失败/歧义返回-1。
{ // 开始App_Task_MatchTarget函数体。
    memset(match_reject, 0, sizeof(match_reject)); // 每幅新图独立记录各类候选拒绝。
    match_reason = "no_candidate";
    float best_score = APP_TASK_TRACK_GATE * APP_TASK_TRACK_GATE; // 最佳关联距离初值设为允许跨帧位移的平方。
    float second_score = 1.0e9f; // 次近候选距离初值设为很大，后续用于关联歧义检查。
    int best = -1; // 最佳候选下标初始化为未找到。
    uint8_t i; // 声明遍历场景目标数组的下标。

    uint8_t capture = target_zone == 0 && target_mixed == 0;
    if(!capture && (task_scene.image_width != target_image_w || task_scene.image_height != target_image_h)){
        match_reason = "image_size_changed";
        return -1; // 返回当前目标不可使用或未能可靠匹配，由调用者按本步骤处理。
    } // 结束条件分支。
    for(i = 0; i < task_scene.object_count; i++){
        const RescueObject *object = App_Task_ObjectAt(i); // 取得相机原始检测框。
        float dx = ((float)object->cx - target.cx) * APP_TASK_IMAGE_W / task_scene.image_width;
        float dy = ((float)object->cy - target.cy) * APP_TASK_IMAGE_H / task_scene.image_height;
        float width = object->width, height = object->height;
        float previous_width = target.width, previous_height = target.height;
        if(capture){
            dx = App_Task_X(object) - target.cx * APP_TASK_IMAGE_W / target_image_w;
            dy = App_Task_Y(object) - target.cy * APP_TASK_IMAGE_H / target_image_h;
            width = object->width * APP_TASK_IMAGE_W / task_scene.image_width;
            height = object->height * APP_TASK_IMAGE_H / task_scene.image_height;
            previous_width = target.width * APP_TASK_IMAGE_W / target_image_w;
            previous_height = target.height * APP_TASK_IMAGE_H / target_image_h;
        }
        float score = dx * dx + dy * dy; // 与上次目标的二维平方距离；不必开平方即可比较远近

        if(object->type != target.type){ ++match_reject[0]; continue; }
        if(target.track_id != 0 && object->track_id != target.track_id){ ++match_reject[1]; continue; }
        if(width * APP_TASK_TRACK_SIZE_RATIO < previous_width
           || width > previous_width * APP_TASK_TRACK_SIZE_RATIO){ ++match_reject[2]; continue; }
        if(height * APP_TASK_TRACK_SIZE_RATIO < previous_height
           || height > previous_height * APP_TASK_TRACK_SIZE_RATIO){ ++match_reject[3]; continue; }
        if(score > APP_TASK_TRACK_GATE * APP_TASK_TRACK_GATE){ ++match_reject[4]; continue; }
        if(best < 0 || score < best_score){ // 第一个合格候选，或比当前最佳更接近原位置的候选，更新最佳排名。
            second_score = best < 0 ? second_score : best_score; // 已经存在最佳候选时，将其距离保存为新的次近距离。
            best_score = score; // 保存新最佳候选的二维平方距离。
            best = i; // 保存当前最佳候选的本帧下标，不能将下标当作跨帧身份。
        }else if(score < second_score){ // 当前候选不是最近，但优于已有次近候选时更新次近距离。
            second_score = score; // 保存次近距离，用于最终判断最佳匹配是否足够明确。
        } // 结束条件分支。
    } // 结束目标遍历循环。
    match_best_score = best_score; match_second_score = second_score; // 只保存排序得分供诊断。
    if(second_score - best_score < APP_TASK_TRACK_AMBIGUITY * APP_TASK_TRACK_AMBIGUITY){ // 最近与次近候选距离差太小时放弃关联，避免强行换锁。
        match_reason = "ambiguous";
        return -1; // 返回当前目标不可使用或未能可靠匹配，由调用者按本步骤处理。
    } // 结束条件分支。
    if(best >= 0){ match_reason = "ok"; }
    else if(match_reject[1] || match_reject[2] || match_reject[3] || match_reject[4]){ match_reason = "candidates_rejected"; }
    else { match_reason = "no_same_type"; }
    return best; // 返回最近且无歧义的候选下标，未找到时保持-1。
} // 结束App_Task_MatchTarget函数体。

// @简介：视觉PID输出左右轮速；speed=0用于原地对准，正负值分别表示前进和倒车。
// 平移时按速度绝对值限幅，保证两轮保持同一行驶方向；目标偏右时左轮加速、右轮减速。
static void App_Task_VisualDriveAt(float x, float speed, uint8_t near, float aim_x, float x_tol)
{ // 开始App_Task_VisualDrive函数体。
    // x_tol<0保留绿色远/近死区；整区显式传入独立X允许误差。
    float enter = x_tol >= 0.0f ? x_tol : (near ? LINE_PID_NEAR_ENTER : LINE_PID_DEAD_ENTER);
    float exit = x_tol >= 0.0f ? x_tol : (near ? LINE_PID_NEAR_EXIT : LINE_PID_DEAD_EXIT);
    float turn = App_LinePID_ComputeTarget(x, aim_x, enter, exit);
    float limit = App_Task_Abs(speed) * LINE_PID_FORWARD_RATIO; // 按基础速度绝对值计算差速限幅，避免纠偏使任一侧反向。
    if(speed != 0.0f){ // 前进和倒车均限制差速；speed为0时保留原地转向输出。
        if(turn > limit){ turn = limit; } // 限制正差速，避免转向修正抵消基础平移方向。
        if(turn < -limit){ turn = -limit; } // 限制负差速，避免转向修正抵消基础平移方向。
    } // 结束条件分支。
    App_Task_Move_Speed(speed + turn, speed - turn); // 仅运行速度内环
} // 结束App_Task_VisualDrive函数体。

static void App_Task_VisualDrive(float x, float speed, uint8_t near)
{
    App_Task_VisualDriveAt(x, speed, near, LINE_PID_AIM_X, -1.0f);
}

// @简介：按归一化中心坐标统计前框内实体；1=单绿色通过，0=未看到，-1=有冲突。
// @说明：与App_Cargo_Test打印的x/y相同，230≤x≤490且400≤y≤450才计数；不检查框边缘。
// 中心在区域外的实体不计数；区域内未知物、危险物、伤员等也计入，不能只数绿色。
static int App_Task_CheckCargo(void) // 位置合格后继续检查单件和绿色类别。
{ // 开始App_Task_CheckCargo函数体。
    uint8_t i; // 声明遍历场景目标数组的下标。
    cargo_bad_index = -1; cargo_count = 0; cargo_reason = "cargo_empty";
    uint8_t count = 0; // 从零开始累计本次检查涉及的实体数量。
    for(i = 0; i < task_scene.object_count; i++){ // 逐项检查相机上报场景中的全部目标。
        const RescueObject *object = &task_scene.objects[i]; // 取得当前目标的只读指针，避免复制检测信息。
        float x = App_Task_X(object); // 与测试打印一致：cx×640/image_width，物块中心横坐标。
        float y = App_Task_Y(object); // 与测试打印一致：cy×480/image_height，物块中心纵坐标。

        // 区域标签不是待运实体，避免安全区背景框被统计成物块。
        if(object->type >= RESCUE_OBJECT_RED_SAFE_ZONE && object->type <= RESCUE_OBJECT_START_ZONE){ continue; } // 整体安全区及出发区标签不作为前框内实体计数。
        if(object->type >= RESCUE_OBJECT_RED_SUPPLY_ZONE){ continue; } // 物资区、伤员区等分区标签同样不计作载物实体。
        if(x < APP_TASK_CARGO_X_MIN || x > APP_TASK_CARGO_X_MAX // 中心x在230～490之外时忽略；端点包含在内。
           || y < APP_TASK_CARGO_Y_MIN || y > APP_TASK_CARGO_Y_MAX){ continue; } // 中心y在400～450之外时同样忽略。
        count++;
        cargo_count = count; // 中心位于前框区域内，计入本帧实体数量。
        if(count > 1 || object->type != RESCUE_OBJECT_SUPPLY){ // 区域内多个实体或非绿色时拒绝复检。
            cargo_bad_index = i;
            cargo_reason = count > 1 ? "cargo_multiple" : "cargo_wrong_type";
            return -1; // 返回当前目标不可使用或未能可靠匹配，由调用者按本步骤处理。
        } // 结束中心入区后的数量和类型检查。
    } // 结束目标遍历循环。
    if(count == 1){ cargo_reason = "ok"; }
    return count == 1 ? 1 : 0; // 恰好一个合格绿色返回1，无实体中心在区域内则返回0。
} // 结束App_Task_CheckCargo函数体。


// 远处只查本队整体安全区5/6，不要求分界线或分区标签。
static int App_Task_FindWholeZone(void)
{
    for(uint8_t i = 0; i < task_scene.object_count; ++i){
        if(task_scene.objects[i].type == team_zone_type){ return i; }
    }
    return -1;
}




// @简介：失败/超时时打印最后检查现场；snapshot=0表示快照是旧值，不能当作当前图。
void App_Task_Vision_LogDetails(const char *stage)
{
    if(APP_TASK_DEBUG_ENABLE == 0){ return; }
    App_USART1_Printf("[SCENE] reason=%s snapshot=%u frame=%u age=%lu\r\n", scene_reason,
                     (unsigned int)scene_snapshot, (unsigned int)task_scene.frame_id,
                     (unsigned long)(HAL_GetTick() - task_scene.received_ms));
    App_USART1_Printf("[SCENE] view=%u/%u flags=%u n=%u img=(%u,%u)\r\n",
                     (unsigned int)task_scene.view_id, (unsigned int)view_id,
                     (unsigned int)task_scene.flags, (unsigned int)task_scene.object_count,
                     (unsigned int)task_scene.image_width, (unsigned int)task_scene.image_height);
    if(strcmp(scene_reason, "receive_age") == 0 || strcmp(scene_reason, "source_age") == 0){
        App_USART1_Printf("[SCENE] checked_age=%lu source=%u local_max=%u total_max=%u\r\n",
                         (unsigned long)checked_age, (unsigned int)task_scene.source_age_ms,
                         (unsigned int)APP_TASK_TRACK_MAX_AGE_MS, (unsigned int)APP_TASK_IMAGE_MAX_AGE_MS);
    }
    if(strcmp(scene_reason, "camera_angle") == 0){
        App_USART1_Printf("[SCENE] camera=%.1f expected=%.1f tol=%.1f\r\n",
                         servo_2.Angle, camera_angle, APP_TASK_SERVO_TOL);
    }
    if(strcmp(stage, "VERIFY") == 0){
        App_USART1_Printf("[VERIFY] frames=%u/%u hits=%u/%u wait=%s\r\n",
                         (unsigned int)verify_frames, (unsigned int)APP_TASK_VERIFY_FRAMES,
                         (unsigned int)verify_hits, (unsigned int)APP_TASK_VERIFY_MIN_HITS, verify_wait);
        App_USART1_Printf("[VERIFY] new=%u duplicate=%u invalid=%u cargo=%s count=%u idx=%d\r\n",
                         (unsigned int)verify_new_total, (unsigned int)verify_duplicates,
                         (unsigned int)verify_invalid, cargo_reason, (unsigned int)cargo_count, cargo_bad_index);
        App_USART1_Printf("[VERIFY] region=(%.0f..%.0f,%.0f..%.0f) ref=(%.0f,%.0f)\r\n",
                         APP_TASK_CARGO_X_MIN, APP_TASK_CARGO_X_MAX, APP_TASK_CARGO_Y_MIN,
                         APP_TASK_CARGO_Y_MAX, APP_TASK_IMAGE_W, APP_TASK_IMAGE_H);
        if(task_scene.image_width != 0 && task_scene.image_height != 0){
            for(uint8_t i = 0; i < task_scene.object_count && i < RESCUE_MAX_OBJECTS; ++i){
                const RescueObject *o = &task_scene.objects[i];
                float x = App_Task_X(o), y = App_Task_Y(o);
                unsigned int inside = x >= APP_TASK_CARGO_X_MIN && x <= APP_TASK_CARGO_X_MAX
                                      && y >= APP_TASK_CARGO_Y_MIN && y <= APP_TASK_CARGO_Y_MAX;
                unsigned int entity = !((o->type >= RESCUE_OBJECT_RED_SAFE_ZONE && o->type <= RESCUE_OBJECT_START_ZONE)
                                        || o->type >= RESCUE_OBJECT_RED_SUPPLY_ZONE);
                App_USART1_Printf("[VERIFY] obj=%u type=%u conf=%u xy=(%.1f,%.1f) in=%u entity=%u\r\n",
                                 (unsigned int)i, (unsigned int)o->type, (unsigned int)o->confidence,
                                 x, y, inside, entity);
            }
        }
    }else if(strcmp(stage, "SELECT") == 0){
        App_USART1_Printf("[SELECT] detail=%s empty=%u/%u\r\n", select_wait,
                         (unsigned int)search_empty_frames, (unsigned int)APP_TASK_SEARCH_EMPTY_FRAMES);
    }else if(strcmp(stage, "SCATTER") == 0){
        App_USART1_Printf("[SCATTER] detail=%s idx=%d camera=%.1f frame=%.1f\r\n",
                         scatter_reason, scatter_bad_index, camera_angle, servo_1.Angle);
        if(scatter_bad_index >= 0 && scatter_bad_index < task_scene.object_count){
            const RescueObject *o = &task_scene.objects[scatter_bad_index];
            App_USART1_Printf("[SCATTER] type=%u conf=%u xy=(%u,%u)\r\n", (unsigned int)o->type,
                             (unsigned int)o->confidence, (unsigned int)o->cx, (unsigned int)o->cy);
        }
    }else {
        App_USART1_Printf("[MATCH] reason=%s reject(type,id,w,h,dist)=(%u,%u,%u,%u,%u)\r\n",
                         match_reason, (unsigned int)match_reject[0], (unsigned int)match_reject[1],
                         (unsigned int)match_reject[2], (unsigned int)match_reject[3],
                          (unsigned int)match_reject[4]);
        App_USART1_Printf("[MATCH] best2=%.1f second2=%.1f gate=%.1f ambiguity2=%.1f\r\n",
                         match_best_score, match_second_score, APP_TASK_TRACK_GATE,
                         APP_TASK_TRACK_AMBIGUITY * APP_TASK_TRACK_AMBIGUITY);
        App_USART1_Printf("[TRACK] type=%u conf=%u id=%u xy=(%u,%u) wh=(%u,%u)\r\n",
                         (unsigned int)diagnostic_target.type, (unsigned int)diagnostic_target.confidence,
                         (unsigned int)diagnostic_target.track_id, (unsigned int)diagnostic_target.cx,
                         (unsigned int)diagnostic_target.cy, (unsigned int)diagnostic_target.width,
                         (unsigned int)diagnostic_target.height);

    }
}

// @简介：视觉步骤统一失败出口，立即撤销运动，不直接改变上层任务状态。
static App_Task_Result App_Task_Vision_Failed(void) // 函数入口：视觉步骤统一失败出口，立即撤销运动，不直接改变上层任务状态。
{ // 开始App_Task_Vision_Failed函数体。
    App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
    return APP_STEP_FAILED; // 报告本步骤失败，不再批准后续动作。
} // 结束App_Task_Vision_Failed函数体。


static App_Task_Result App_Task_CaptureFollow(float x, float y, float aim_x, float x_tol,
                                               float stop_y, float y_tol, float *drive_speed);

// 沿用安全区读图：当前view_id、有效图及新帧；GetScene另有500ms本地年龄限制。
// 不新增绿色的ID/尺寸/位移/置信度/列表截断筛选。
static int App_Task_ReadZoneScene(void)
{
    App_Task_UpdateView();
    if(Rescue_Usart2_GetScene(&task_scene) == 0
       || task_scene.view_id != view_id
       || (task_scene.flags & RESCUE_SCENE_IMAGE_VALID) == 0){ return -1; }
    return App_Task_NewFrame() ? 1 : 0;
}

static uint8_t App_Task_ZoneInRange(const RescueObject *object)
{
    return App_Task_Abs(App_Task_X(object) - APP_TASK_ZONE_AIM_X) <= APP_TASK_ZONE_X_TOL
           && App_Task_Abs(App_Task_Y(object) - APP_TASK_ZONE_AIM_Y) <= APP_TASK_ZONE_Y_TOL;
}

static App_Task_Result App_Task_ZoneResearch(const char *reason)
{
    App_Task_Move_Stop();
    zone_confirming = zone_missing = zone_stopped = 0;
    zone_drive_speed = 0.0f;
    App_LinePID_Reset();
    if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[ZONE] research=%s\r\n", reason); }
    return APP_STEP_RESEARCH; // 任务层发新view_id并立即开始旋转；不抬框、不清载物锁。
}

static App_Task_Result App_Task_ZoneFollow(void)
{
    uint32_t now = HAL_GetTick();
    // 确认总期限不因重复/无效图延长；到期不接受迟到的第三张。
    if(zone_confirming && (uint32_t)(now - zone_confirm_time) >= APP_TASK_ZONE_CONFIRM_MS){
        return App_Task_ZoneResearch("confirm_timeout");
    }
    int frame = App_Task_ReadZoneScene();
    int index = frame > 0 ? App_Task_FindWholeZone() : -1;
    if(zone_confirming){
        App_Task_Move_Stop();
        if(frame > 0){
            ++zone_confirm_frames; // 有效空图/无本队整区/XY超差均占一张失败图。
            if(index >= 0 && App_Task_ZoneInRange(&task_scene.objects[index])){ ++zone_confirm_hits; }
            if(APP_TASK_DEBUG_ENABLE){
                App_USART1_Printf("[ZONE] confirm=%u/%u hits=%u\r\n",
                                 (unsigned int)zone_confirm_frames, (unsigned int)APP_TASK_ZONE_CONFIRM_FRAMES,
                                 (unsigned int)zone_confirm_hits);
            }
            if(zone_confirm_frames >= APP_TASK_ZONE_CONFIRM_FRAMES){
                if(zone_confirm_hits >= APP_TASK_ZONE_CONFIRM_HITS){
                    zone_confirming = 0;
                    if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[ZONE] done cargo=held\r\n"); }
                    return APP_STEP_DONE;
                }
                return App_Task_ZoneResearch("confirm_no_hit");
            }
        }
        return APP_STEP_WAIT;
    }

    if(index >= 0){
        target = task_scene.objects[index]; // 只取第一个本队整区，不关联旧框ID/尺寸/位移。
        if(zone_missing){
            App_LinePID_Reset();
            if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[ZONE] found phase=%s\r\n", follow_forward ? "xy" : "align"); }
        }
        zone_missing = zone_stopped = 0;
        App_Task_Result result = App_Task_CaptureFollow(App_Task_X(&target), App_Task_Y(&target),
            APP_TASK_ZONE_AIM_X, APP_TASK_ZONE_X_TOL, APP_TASK_ZONE_AIM_Y, APP_TASK_ZONE_Y_TOL,
            &zone_drive_speed);
        if(result == APP_STEP_DONE){
            zone_confirming = 1;
            zone_confirm_frames = zone_confirm_hits = 0;
            zone_confirm_time = HAL_GetTick(); // 触发停车的本张已被读图去重，不计入后续三图。
            if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[ZONE] phase=confirm\r\n"); }
        }
        return APP_STEP_RUNNING;
    }

    if(frame != 0 && !zone_missing){ // 正常两张新图之间只维持速度环，不误判丢失。
        zone_missing = 1;
        zone_stopped = 0;
        zone_missing_time = now;
        if(follow_forward){
            App_Update_Data();
            Set_Angle_Target(angle_yaw);
        }else { App_Task_Move_Stop(); }
        if(APP_TASK_DEBUG_ENABLE){
            App_USART1_Printf("[ZONE] missing phase=%s next=%s\r\n",
                             follow_forward ? "xy" : "align", follow_forward ? "hold" : "wait");
        }
    }
    if(!zone_missing){ App_Task_Move_Proc(); return APP_STEP_RUNNING; }

    uint32_t elapsed = (uint32_t)(now - zone_missing_time);
    if(!follow_forward){
        if(elapsed >= APP_TASK_ZONE_ALIGN_LOST_MS){ return App_Task_ZoneResearch("align_lost"); }
        App_Task_Move_Stop();
    }else if(elapsed < APP_TASK_ZONE_HOLD_MS){
        App_Heading_Pro(zone_drive_speed); // 保留实际前进/倒车/零速度，不使用旧视觉差速。
    }else if(elapsed < APP_TASK_ZONE_HOLD_MS + APP_TASK_ZONE_STOP_WAIT_MS){
        App_Task_Move_Stop();
        if(!zone_stopped && APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[ZONE] missing phase=xy next=wait\r\n"); }
        zone_stopped = 1;
    }else { return App_Task_ZoneResearch("xy_lost"); }
    return APP_STEP_WAIT;
}


// @简介：跟随失败时先停车，再打印一次原因和最后保存的图像/目标信息。
// @说明：phase为0表示首次对准、1表示前进；读取场景失败时，字段仍是最后保存值。
static App_Task_Result App_Task_FollowFailed(const char *reason) // reason为本次退出原因，不周期打印。
{ // 开始失败诊断函数。
    App_Task_Result result = App_Task_Vision_Failed(); // 先撤销运动，避免串口发送延后停车。
    if(APP_TASK_DEBUG_ENABLE){ // 与任务、转向日志使用同一个调试开关。
        App_USART1_Printf("[FOLLOW] stop=%s phase=%u\r\n", reason, (unsigned int)follow_forward); // 输出退出原因和当前阶段。
        App_USART1_Printf("[VISION] frame=%u age=%lu view=%u/%u flags=%u n=%u\r\n", (unsigned int)task_scene.frame_id, (unsigned long)(HAL_GetTick() - task_scene.received_ms), (unsigned int)task_scene.view_id, (unsigned int)view_id, (unsigned int)task_scene.flags, (unsigned int)task_scene.object_count); // 输出最后保存图像的本地年龄、收到/期望视角和标志。
        App_USART1_Printf("[TARGET] xy=(%u,%u) wh=(%u,%u) img=(%u,%u) lost=%u mixed=%u\r\n", (unsigned int)target.cx, (unsigned int)target.cy, (unsigned int)target.width, (unsigned int)target.height, (unsigned int)target_image_w, (unsigned int)target_image_h, (unsigned int)lost_frames, (unsigned int)target_mixed); // 输出锁定目标原始像素框、锁定时分辨率、丢失帧数及撞散用途。
    } // 结束调试开关判断。
    App_Task_Vision_LogDetails(strcmp(reason, "scatter_unsafe") == 0 ? "SCATTER" : "FOLLOW"); // 打印实际失败子检查。
    return result; // 保持原来的失败返回值，上层随后回到停车状态。
} // 结束失败诊断函数。

// 普通绿色的坏图和失配共用停等期限，只有有效新图失配累计帧数。
static App_Task_Result App_Task_GreenMissing(uint8_t missing_frame, const char *reason)
{
    App_Task_Move_Stop();
    capture_y_direction = 0;
    capture_y_request = 0;
    capture_y_frames = 0;
    arrive_frames = 0;
    if(green_waiting == 0){
        green_waiting = 1;
        lost_time = HAL_GetTick();
        App_LinePID_Reset();
        if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[FOLLOW] wait=%s phase=%u\r\n", reason, (unsigned int)follow_forward); }
        App_Task_Vision_LogDetails("FOLLOW");
    }
    if(missing_frame){ ++lost_frames; }
    if(lost_frames >= APP_TASK_LOST_FRAMES
       || (uint32_t)(HAL_GetTick() - lost_time) >= APP_TASK_LOST_TIMEOUT_MS){
        if(APP_TASK_DEBUG_ENABLE){
            App_USART1_Printf("[FOLLOW] lost=%s next=back_search\r\n",
                             lost_frames >= APP_TASK_LOST_FRAMES ? "target_lost" : "lost_timeout");
        }
        return APP_STEP_TARGET_LOST;
    }
    return APP_STEP_WAIT;
}

// @简介：未匹配原目标时先停车；普通绿色确认丢失后交任务层后退，旧用途保持原处理。
// @说明：同帧/坏图不在此累计；恢复匹配仍需原ID（如有）、位置/尺寸门限，不能改选其它绿色。
static App_Task_Result App_Task_TargetMissing(void) // 函数入口：未匹配原目标时先停车；连续有效新帧均找不到才结束，旧锁定框保持不变。
{ // 开始App_Task_TargetMissing函数体。
    if(target_mixed == 0){ return App_Task_GreenMissing(1, match_reason); }
    App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
    capture_y_direction = 0; // 丢失时清除纵向实际运动方向，恢复后必须重新稳定确认。
    capture_y_request = 0; // 清除丢失前等待恢复的方向请求。
    capture_y_frames = 0; // 清除丢失期间未处理新图对应的连续帧确认。
    arrive_frames = 0; // 清零连续到位确认帧数，禁止继承旧确认结果。
    if(lost_frames == 0){ // 第一次未匹配原目标时，记录本次短暂丢失的起点。
        lost_time = HAL_GetTick(); // 先记录原计时起点，再打印；日志不延后丢失超时起点。
        if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[MATCH] wait=%s lost=1/%u\r\n", match_reason, (unsigned int)APP_TASK_LOST_FRAMES); }
        App_Task_Vision_LogDetails("FOLLOW"); // 已停车，只在首次丢失时打印关联失败现场。
        App_LinePID_Reset(); // 清空视觉PID历史误差及死区状态，避免旧输出干扰新阶段。
    } // 结束条件分支。
    if(++lost_frames >= APP_TASK_LOST_FRAMES){ return App_Task_FollowFailed("target_lost"); } // 连续有效新帧均找不到原目标达到门限后确认丢失并失败。
    return APP_STEP_WAIT; // 保持当前安全状态，等待下一张新图或机械稳定。
} // 结束App_Task_TargetMissing函数体。

// @简介：到位先停车，连续新图位置稳定且经过制动等待才批准下一步机械动作。
// @说明：中心x必须始终满足前框容差；y仅允许少量回落。位置漂移则重新计数/等待。
static App_Task_Result App_Task_Arrive(float x, float y) // 函数入口：到位先停车，连续新图位置稳定且经过制动等待才批准下一步机械动作。
{ // 开始App_Task_Arrive函数体。
    App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
    if(App_Task_Abs(x - APP_TASK_AIM_X) > APP_TASK_ARRIVE_X){ // 原横向条件保持不变，补充具体到位值。
        if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[ARRIVE] x=%.1f aim=%.1f tol=%.1f y=%.1f\r\n", x, APP_TASK_AIM_X, APP_TASK_ARRIVE_X, y); }
        return App_Task_FollowFailed("arrive_off_center");
    }
    if(arrive_frames == 0 || App_Task_Abs(x - arrive_x) > APP_TASK_ARRIVE_STABLE_PX // 首次复核或目标x相对本轮基准漂移过大时重新建立稳定窗口。
       || App_Task_Abs(y - arrive_y) > APP_TASK_ARRIVE_STABLE_PX){ // 目标y漂移过大同样重新计时与计帧。
        arrive_x = x; // 保存本轮到位确认的目标x位置基准。
        arrive_y = y; // 保存本轮到位确认的目标y位置基准。
        arrive_time = HAL_GetTick(); // 重新记录停车稳定等待起点。
        arrive_frames = 0; // 清零连续到位确认帧数，禁止继承旧确认结果。
    } // 结束到位确认基准及计时窗口的重建。
    if(arrive_frames < APP_TASK_ARRIVE_FRAMES){ arrive_frames++; } // 每张新图增加一次到位计数，达到门限后停止累加。
    return arrive_frames >= APP_TASK_ARRIVE_FRAMES // 到位批准必须满足连续稳定新帧数量要求。
           && (uint32_t)(HAL_GetTick() - arrive_time) >= APP_TASK_ARRIVE_WAIT_MS // 同时必须已经停车等待足够的制动及稳定时间。
           ? APP_STEP_DONE : APP_STEP_WAIT; // 帧数和时间都满足才返回完成，否则保持停车等待。
} // 结束App_Task_Arrive函数体。

// @简介：绿色和整区共用首次X对准与纵向P控制；目标及允许误差由调用方传入。
// @说明：Y误差为目标Y减当前Y，正值前进、负值倒车；停车带内只做横向补正和到位复核。
static App_Task_Result App_Task_CaptureFollow(float x, float y, float aim_x, float x_tol,
                                               float stop_y, float y_tol, float *drive_speed) // 输出实际纵向速度供暂失时保持。
{ // 绿色与整区共用同一份视觉PID和Y轴比例调速，不设专用接近速度。
    float deadband = target_zone ? x_tol : -1.0f; // 绿色保留原远/近横向死区。
    *drive_speed = 0.0f; // 对准、停车及等待分支均没有纵向运动。
    if(follow_forward == 0){ // 首次原地对准只用X误差转向，Y轴运动控制从对准完成后开始。
        if(target_zone ? App_Task_Abs(x - aim_x) <= x_tol : App_LinePID_InDeadband(x)){ // 目标x进入横向死区时结束首次对准。
            App_Task_Move_Stop(); // 对准阶段结束时停车，避免把转向输出带入平移。
            App_LinePID_Reset(); // 清除横向PID历史误差及死区状态。
            follow_forward = 1; // 标记首次对准已完成，进入XY纵向跟随阶段。
            capture_y_direction = 0; // 从停车状态开始判定纵向控制方向。
            capture_y_request = 0; // 清除之前可能遗留的待恢复方向。
            capture_y_frames = 0; // 重置纵向方向稳定确认帧数。
            capture_y_wait_time = HAL_GetTick(); // 记录首次平移前的停车等待起点。
            return APP_STEP_RUNNING; // 下一张新图再开始Y轴方向确认和运动。
        } // 结束首次对准完成判断。
        App_Task_VisualDriveAt(x, 0.0f, 0, aim_x, deadband); // 目标尚未对准时仅原地执行X轴视觉PID。
        return APP_STEP_RUNNING; // 对准阶段不执行前后运动。
    } // 结束首次原地对准处理。
    float error = stop_y - y; // 由本用途的独立Y目标计算，其余P参数不变。
    float abs_error = App_Task_Abs(error); // 取得纵向像素误差绝对值，用于停车带和速度计算。
    int8_t request = error >= 0.0f ? 1 : -1; // 将误差方向转换为前进1或倒车-1。
    float speed; // 保存Y轴比例控制算出的轮速幅值，单位cm/s。
    uint32_t now = HAL_GetTick(); // 读取当前时间，供方向稳定等待使用。

    if(abs_error <= y_tol){ // 绿色和整区均使用各自的Y停车带。
        capture_y_direction = 0; // 停止纵向控制，不让微小误差引起前后摆动。
        capture_y_request = 0; // 清除待恢复方向，滞回停车本身不代表到位完成。
        capture_y_frames = 0; // 清空方向确认帧数。
        App_Task_Move_Stop(); // 停车后再判断是否满足落框所需的位置条件。
        if(App_Task_Abs(x - aim_x) <= x_tol){ // 纵向和横向都在允许范围才进入落框稳定复核。
            return target_zone ? APP_STEP_DONE : App_Task_Arrive(x, y); // 整区交给三图确认；绿色保留原稳定复核。
        } // 结束到位位置判断。
        App_Task_VisualDriveAt(x, 0.0f, 0, aim_x, deadband); // Y已到位但X偏差较大时，只原地用横向PID转正。
        return APP_STEP_RUNNING; // 等待下一张新图重新判断XY位置。
    } // 结束停车带处理。

    arrive_frames = 0; // 离开停车带后撤销落框到位计数。
    if(capture_y_direction != 0){ // 当前正在纵向移动时先判断是否需要改变方向。
        if(request != capture_y_direction){ // 误差方向反转时不直接反向驱动，先停车进入稳定等待。
            App_Task_Move_Stop(); // 立即停止，避免越过目标后前后猛切。
            capture_y_direction = 0; // 标记纵向控制已经停车。
            capture_y_request = request; // 保存新的反向请求，后续必须重新确认。
            capture_y_frames = 1; // 当前新图作为反向确认的第一帧。
            capture_y_wait_time = now; // 从停车时刻开始累计方向切换等待。
            return APP_STEP_WAIT; // 本帧只停车，不下发反向速度。
        } // 结束方向反转判断。
    }else { // 当前已停车，检查是否达到滞回外门限并满足稳定重启条件。
        if(abs_error < y_tol + LINE_PID_Y_HYSTERESIS){ // 以本用途的停车带加公共滞回作为外门限。
            capture_y_request = 0; // 清除尚未越过外门限的方向请求。
            capture_y_frames = 0; // 清空不满足恢复条件时的连续新帧计数。
            App_Task_Move_Stop(); // 保持停车，不把滞回区误当作到位区。
            if(App_Task_Abs(x - aim_x) > x_tol){ // X仍偏离前框范围时允许原地修正朝向。
                App_Task_VisualDriveAt(x, 0.0f, 0, aim_x, deadband); // 只调整横向，不执行纵向平移。
            } // 结束横向修正判断。
            return APP_STEP_WAIT; // 等待误差明确越过外门限，或下一帧进入真实停车带。
        } // 结束外门限判断。
        if(capture_y_request != request){ // 首次请求或方向变更时，重建方向稳定确认窗口。
            capture_y_request = request; // 保存当前请求方向。
            capture_y_frames = 1; // 当前新图计为该方向的第一帧。
            capture_y_wait_time = now; // 记录停车稳定等待起点。
        }else if(capture_y_frames < LINE_PID_Y_RESUME_FRAMES){ // 请求方向未改变时，累计同方向的新图。
            capture_y_frames++; // 增加连续同方向帧数，重复帧已在调用处过滤。
        } // 结束请求方向帧数更新。
        if(capture_y_frames < LINE_PID_Y_RESUME_FRAMES // 连续同方向新图不足时继续保持停车。
           || (uint32_t)(now - capture_y_wait_time) < LINE_PID_Y_RESUME_WAIT_MS){ // 或停车等待时间尚未达到200ms时保持停车。
            App_Task_Move_Stop(); // 在确认窗口内保持小车停车。
            return APP_STEP_WAIT; // 等待后续新帧或时间满足恢复条件。
        } // 结束运动恢复条件判断。
        capture_y_direction = request; // 同方向新帧和停车时间都满足后，允许按该方向运动。
    } // 结束停车状态下的滞回与恢复判断。

    speed = abs_error * LINE_PID_Y_KP; // 纵向P控制：离目标越远，前后调整速度越大。
    if(speed < LINE_PID_Y_MIN_SPEED){ speed = LINE_PID_Y_MIN_SPEED; } // 误差仍在运动区时施加最小轮速，克服电机静摩擦。
    if(request > 0 && speed > LINE_PID_Y_MAX_FORWARD){ speed = LINE_PID_Y_MAX_FORWARD; } // 限制向前接近的最大速度。
    if(request < 0 && speed > LINE_PID_Y_MAX_REVERSE){ speed = LINE_PID_Y_MAX_REVERSE; } // 倒车方向速度上限更低，减少过近修正冲击。
    *drive_speed = request > 0 ? speed : -speed; // 保存这次真正使用的纵向速度，不另设固定速度。
    App_Task_VisualDriveAt(x, *drive_speed, 1, aim_x, deadband); // 按Y轴P速度前进/倒车，同时用横向视觉PID差速对准。
    return APP_STEP_RUNNING; // 纵向调整仍在进行，主循环继续处理新图和速度闭环。
} // 结束单独绿色纵向跟随函数体。

// @简介：初始化队伍及跟踪用途；view_id不清零，重试不能复用之前的视角编号。
void App_Task_Vision_Init(uint8_t team) // 函数入口：初始化队伍及跟踪用途；view_id不清零，重试不能复用之前的视角编号。
{ // 开始App_Task_Vision_Init函数体。
    team_zone_type = team == APP_TASK_TEAM_RED ? RESCUE_OBJECT_RED_SAFE_ZONE : RESCUE_OBJECT_BLUE_SAFE_ZONE; // 按启动队伍锁存红方或蓝方整块安全区类型。
    camera_angle = SERVO2_LIFT; // 本阶段期望俯角初始化为正常观察角。
    target_mixed = 0; // 清除撞散用途标志，后续由目标选择重新确定。
    target_zone = 0; // 清除整区用途标志，初始化为场地物块用途。
    App_LinePID_Reset(); // 清空视觉PID历史误差及死区状态，避免旧输出干扰新阶段。
    green_waiting = 0;
} // 结束App_Task_Vision_Init函数体。

// @简介：开始新视角，清除上一阶段的新帧和复检计数；等待由调用方指定，0表示立即发送新编号。
void App_Task_Vision_Begin(float angle, uint32_t settle_ms) // 函数入口：开始新视角并指定稳定等待，0表示立即发送新编号。
{ // 开始App_Task_Vision_Begin函数体。
    App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
    App_Task_Vision_BeginMoving(angle, settle_ms); // 复用原新视角初始化；其它阶段仍先停车。
} // 结束App_Task_Vision_Begin函数体。

// @简介：保持当前轮速开始新视角；启动切换、复检及恢复重搜共用，不产生切换短停。
void App_Task_Vision_BeginMoving(float angle, uint32_t settle_ms)
{
    camera_angle = angle; // 保存本阶段期望俯角，后续对照舵机结构体设定角检查。
    view_begin_time = HAL_GetTick(); // 记录新视角开始时刻，从此计算画面稳定等待。
    view_settle_ms = settle_ms; // 保存本次等待时长，不改变右转、换向或撞散重选的等待参数。
    view_id++; // 递增视角编号，使上一阶段迟到图像不再有效。
    if(view_id == 0){ view_id = 1; } // 编号回绕到0时跳过零值，继续使用非零上下文编号。
    view_sent = 0; // 标记新视角上下文尚未成功发送。
    view_send_failed = 0; scene_snapshot = 0;
    scene_reason = match_reason = cargo_reason = scatter_reason = "not_checked";
    verify_wait = select_wait = "new_view";
    verify_reported = select_reported = 0;
    verify_new_total = verify_duplicates = verify_invalid = 0;
    cargo_bad_index = scatter_bad_index = -1;
    cargo_count = 0; checked_age = 0;
    diagnostic_target = target;

    frame_seen = 0; // 清除已见帧标志，重新建立本阶段帧去重基准。
    verify_frames = 0; // 清零本轮复检已检查的新图数量。
    verify_hits = 0; // 清零本轮复检合格新图数量，不能继承上一阶段的命中。
    verify_failure = APP_VERIFY_NONE;
    search_empty_frames = 0; // 清零连续无候选计数，重新判断本视角是否无目标。
    lost_frames = 0; // 清零原目标连续未匹配的新帧计数。
    green_waiting = 0;
    arrive_frames = 0; // 清零连续到位确认帧数，禁止继承旧确认结果。
    App_Task_UpdateView(); // 无稳定等待时本次调用立即发新编号；有等待的阶段仍到时后再发。
} // 结束App_Task_Vision_BeginMoving函数体。

// @简介：初步调试只按类型选择绿色，不判断其它检测框是否与其相邻。
// @说明：沿用框底边更低优先、同底边时更靠近瞄准中心x的排序；锁定后不重新排名。
//        暂不根据相邻关系选择物资堆或自动进入撞散分支。
int App_Task_Vision_Select(void) // 函数入口：从完整有效的新图中选择绿色物资，不检查邻居。
{ // 开始App_Task_Vision_Select函数体。
    int best = -1; // 最佳绿色候选下标初始化为未找到。
    uint8_t i; // 声明遍历场景目标数组的下标。
    if(App_Task_ReadScene() == 0){ search_empty_frames = 0; select_wait = scene_reason; App_Task_LogWait("SELECT", select_wait, &select_reported); return -1; } // 坏图不作为空图累计，清除判空计数后继续等待。
    if(App_Task_NewFrame() == 0){ select_wait = "duplicate_frame"; return -1; } // 不刷屏，超时时仍能区分重复帧等待。
    for(i = 0; i < task_scene.object_count; i++){ // 逐项检查相机上报的目标，只按本框信息筛选绿色。
        const RescueObject *object = &task_scene.objects[i]; // 取得当前候选的只读指针。
        float error = App_Task_Abs(App_Task_X(object) - APP_TASK_AIM_X); // 计算候选偏离车头中心x的误差，用于排序。
        if(object->type != RESCUE_OBJECT_SUPPLY){ continue; } // 绿色仍是当前可落框目标，其它类别不作为候选。
        if(best < 0){ best = i; continue; } // 首个合格绿色直接保存。
        if(App_Task_Bottom(object) > App_Task_Bottom(&task_scene.objects[best]) // 框底边更靠下的绿色优先。
           || (App_Task_Bottom(object) == App_Task_Bottom(&task_scene.objects[best]) // 底边相同时比较横向偏差。
               && error < App_Task_Abs(App_Task_X(&task_scene.objects[best]) - APP_TASK_AIM_X))){ best = i; } // 优先选择更靠近瞄准中心x的绿色。
    } // 结束绿色候选遍历循环。
    if(best >= 0){ // 找到绿色后锁定，沿用原单件对准、接近和落框流程。
        search_empty_frames = 0; // 清除连续无候选计数。
        select_wait = "ok"; select_reported = 0;
        App_Task_Lock((uint8_t)best, 0, 0); // 锁定绿色物资；不按相邻关系标记为撞散用途。
        return 1; // 返回已锁定目标，由上层停车并切换到跟随。
    } // 结束找到绿色分支。
    select_wait = "no_green";
    App_Task_LogWait("SELECT", select_wait, &select_reported);
    return ++search_empty_frames >= APP_TASK_SEARCH_EMPTY_FRAMES ? 0 : -1; // 保留原无候选返回约定，启动旋转搜索收到0仍继续转。
} // 结束App_Task_Vision_Select函数体。

// @简介：报告已锁定目标用途；只有动作结束重检才会重新选择单件或撞散分支。
uint8_t App_Task_Vision_IsMixed(void) // 函数入口：报告已锁定目标用途；只有动作结束重检才会重新选择单件或撞散分支。
{ // 开始App_Task_Vision_IsMixed函数体。
    return target_mixed; // 返回锁定时确定的撞散用途标志，不随列表顺序改变。
} // 结束App_Task_Vision_IsMixed函数体。

// @简介：物块/本队整区共用的非阻塞接近函数，封装首次对准、目标关联及视觉PID。
// @返回：DONE=到位停车复核通过；FAILED=连续丢失/超时/阻挡；其它结果继续调用。
// @说明：首次原地对准后只向前差速，不再原地纠偏；帧间只跑速度闭环。
App_Task_Result App_Task_Vision_Follow(void) // 函数入口：物块/本队整区共用的非阻塞接近函数，封装首次对准、目标关联及视觉PID。
{ // 开始App_Task_Vision_Follow函数体。
    diagnostic_target = target; // 保存判断前原目标，供物块跟随失败打印。
    scatter_reason = "not_checked"; // 本次未执行的子检查不沿用上次的ok或失败原因。
    match_reason = "not_checked"; memset(match_reject, 0, sizeof(match_reject));
    match_best_score = match_second_score = 0.0f;
    if(target_zone){ return App_Task_ZoneFollow(); }
    int index; // 保存本帧与原锁定目标关联成功的下标。
    uint8_t near; // 记录目标是否进入停车门限附近的减速范围。
    uint8_t capture = target_zone == 0 && target_mixed == 0; // 仅单独绿色落框使用已确认的Y目标±10，旧物资堆保持原门限。
    float y; // 保存目标中心的归一化y坐标，向下增大。
    float speed; // 保存本次视觉驱动的基础线速度，单位cm/s。
    float stop_y = target_mixed ? LINE_PID_SCATTER_STOP_Y : LINE_PID_STOP_Y;
    if(capture){ stop_y -= LINE_PID_STOP_Y_BAND; } // 旧路径的门限计算；单独绿色在下方直接使用公共XY控制及自己的Y目标。
    if(capture && green_waiting && (uint32_t)(HAL_GetTick() - lost_time) >= APP_TASK_LOST_TIMEOUT_MS){
        return App_Task_GreenMissing(0, "lost_timeout"); // 到期先后退重搜，不接受本次迟到的目标。
    }
    if(App_Task_ReadScene() == 0){
        return capture ? App_Task_GreenMissing(0, scene_reason) : App_Task_FollowFailed("scene_invalid");
    }
    if(!capture && lost_frames != 0 && (uint32_t)(HAL_GetTick() - lost_time) >= APP_TASK_LOST_TIMEOUT_MS){ // 旧mixed用途保持原等待上限。
        return App_Task_FollowFailed("lost_timeout"); // 输出等待原目标恢复超时。
    } // 结束丢失等待超时判断。
    if(!capture && (task_scene.image_width != target_image_w || task_scene.image_height != target_image_h)){
        return App_Task_FollowFailed("image_size_changed"); // 输出锁定后分辨率发生变化。
    } // 结束分辨率判断。
    if(App_Task_NewFrame() == 0){ App_Task_Move_Proc(); return APP_STEP_RUNNING; } // 没有新图时仅继续速度内环，不重复计算视觉误差或到位帧数。
    index = App_Task_MatchTarget(); // 通过类型、ID、二维位移及框尺寸判断本帧哪个候选是原目标。
    if(index < 0){ return App_Task_TargetMissing(); } // 未可靠匹配原目标时先停车，再累计连续丢失新帧。
    if(target_mixed != 0 && App_Task_ScatterSceneClear() == 0){
        return App_Task_FollowFailed("scatter_unsafe"); // 撞散用途的整幅场景检查失败时仍报告原原因。
    } // 结束用途检查，普通绿色不再检查相邻实体或拥挤情况。
    target = *App_Task_ObjectAt(index); // 每帧更新已关联的真实物块检测。
    if(capture){
        target_image_w = task_scene.image_width;
        target_image_h = task_scene.image_height;
    }
    if(lost_frames != 0 || (capture && green_waiting)){ // 坏图恢复也要求有效新图匹配原目标。
        lost_frames = 0; // 清零原目标连续未匹配的新帧计数。
        green_waiting = 0;
        App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
        App_LinePID_Reset(); // 清空视觉PID历史误差及死区状态，避免旧输出干扰新阶段。
        return APP_STEP_WAIT; // 找回原目标后保持停车一帧，下一张新图恢复原阶段，不再次原地对准
    } // 结束条件分支。
    y = App_Task_Y(&target); // 取原目标本帧中心y，作为纵向控制和停车判断依据。
    if(capture){ // 绿色与整区共用首次X对准及原Y轴P控制。
        return App_Task_CaptureFollow(App_Task_X(&target), y, LINE_PID_AIM_X, APP_TASK_ARRIVE_X,
                                      LINE_PID_STOP_Y, LINE_PID_STOP_Y_BAND, &speed);
    } // 结束单独绿色跟随分支。
    if(y >= stop_y && App_Task_Abs(App_Task_X(&target) - APP_TASK_AIM_X) <= APP_TASK_ARRIVE_X){ // 到达中心Y门限且横向进入落框容差后，才开始到位停车复核。
        return App_Task_Arrive(App_Task_X(&target), y); // 停车并复核位置稳定性、连续新帧数及等待时长。
    } // 结束到位位置判断。
    if(capture == 0 && arrive_frames != 0 && y >= stop_y - APP_TASK_ARRIVE_Y_BACK){ // 旧物资堆保留原少量回落容差；绿色由上方公共XY控制处理。
        return App_Task_Arrive(App_Task_X(&target), y); // 保持停车并复核目标位置。
    } // 结束复核回落容差判断。
    arrive_frames = 0; // 目标回落或尚未对准时清除旧到位计数。
    if(y >= stop_y){ // 已到前进停车门限但横向仍偏离时，禁止继续向前。
        App_Task_VisualDrive(App_Task_X(&target), 0.0f, 0); // 暂停平移，仅用视觉PID原地把目标转到前框中心。
        return APP_STEP_RUNNING; // 对准后由下一张新图进入到位停车复核。
    } // 结束停车门限处的横向对准处理。
    near = stop_y - y <= LINE_PID_SLOW_MARGIN_Y; // 目标中心y距停车门限较近时，启用减速和近处纠偏参数。
    if(follow_forward == 0){ // 首次对准尚未完成时，基础前进速度保持为零。
        if(App_LinePID_InDeadband(App_Task_X(&target))){ // 目标x进入视觉PID死区时结束首次对准。
            App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
            App_LinePID_Reset(); // 清空视觉PID历史误差及死区状态，避免旧输出干扰新阶段。
            follow_forward = 1; // 标记已进入持续前进阶段，之后不再返回首次原地对准。
            return APP_STEP_RUNNING; // 下一张新图才前进
        } // 结束条件分支。
        speed = 0.0f; // 首次未对准时基础前进速度为零，仅由差速产生原地转向。
    }else { // 前述条件不满足时执行此分支。
        speed = near ? LINE_PID_SLOW_SPEED : LINE_PID_BASE_SPEED; // 持续前进时按距停车门限的远近，选择慢速或正常基础轮速。
    } // 结束其它情况分支。
    App_Task_VisualDrive(App_Task_X(&target), speed, near && follow_forward); // 根据目标x输出左右轮差速，近处前进使用近处PID且限制两轮保持向前。
    return APP_STEP_RUNNING; // 动作仍在进行，主循环需继续调用本函数。
} // 结束App_Task_Vision_Follow函数体。

// @简介：报告本次复检失败并保留现场；任务层缓存终态，落框满500ms后按次数重试或恢复。
static App_Task_Result App_Task_VerifyFailed(const char *reason, App_Task_VerifyFailure failure)
{
    verify_failure = failure;
    if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[VERIFY] failed=%s\r\n", reason); }
    App_Task_Vision_LogDetails("VERIFY");
    return APP_STEP_FAILED;
}

App_Task_VerifyFailure App_Task_Vision_GetVerifyFailure(void)
{
    return verify_failure;
}

// @简介：收齐三张有效新图后至少一张恰好一个绿色才通过；空区域计入窗口，冲突立即失败。
App_Task_Result App_Task_Vision_Verify(void)
{
    int result;
    if(App_Task_ReadScene() == 0){
        verify_frames = 0; verify_hits = 0; // 原窗口清零规则保持不变。
        ++verify_invalid; verify_wait = scene_reason;
        App_Task_LogWait("VERIFY", verify_wait, &verify_reported);
        return APP_STEP_WAIT;
    }
    if(App_Task_NewFrame() == 0){ ++verify_duplicates; verify_wait = "duplicate_frame"; return APP_STEP_WAIT; }
    ++verify_new_total;
    result = App_Task_CheckCargo();
    if(result < 0){
        verify_wait = cargo_reason;
        return App_Task_VerifyFailed(cargo_reason, cargo_count > 1 ? APP_VERIFY_MULTIPLE : APP_VERIFY_WRONG_TYPE);
    }
    verify_frames++;
    if(result == 1){ verify_hits++; verify_reported = 0; } // 有效合格图后，再次出现异常时重新提示。
    verify_wait = result == 1 ? "collecting" : "cargo_empty";
    if(result == 0){ App_Task_LogWait("VERIFY", verify_wait, &verify_reported); }
    if(verify_frames < APP_TASK_VERIFY_FRAMES){ return APP_STEP_WAIT; }
    return verify_hits >= APP_TASK_VERIFY_MIN_HITS ? APP_STEP_DONE
         : App_Task_VerifyFailed("no_hit_in_window", APP_VERIFY_NO_HIT);
}

// @简介：旋转搜索本队整块安全区；找到后停车，下一张新图开始首次X对准。
App_Task_Result App_Task_Vision_SearchZone(void)
{
    if(App_Task_ReadZoneScene() > 0){
        int index = App_Task_FindWholeZone();
        if(index >= 0){
            App_Task_Lock((uint8_t)index, 1, 0);
            zone_missing = zone_stopped = zone_confirming = 0;
            zone_confirm_frames = zone_confirm_hits = 0;
            zone_drive_speed = 0.0f;
            App_Task_Move_Stop();
            if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[ZONE] stage=whole whole=%u\r\n", (unsigned int)target.type); }
            return APP_STEP_DONE;
        }
    }
    App_Task_Move_Speed(APP_TASK_ZONE_SPIN_SPEED, -APP_TASK_ZONE_SPIN_SPEED);
    return APP_STEP_RUNNING;
}

// @简介：接触撞散时目标会移动/遮挡，停止单目标关联，保留图像时效与整幅场景检查。
uint8_t App_Task_Vision_ScatterClear(void)
{
    if(App_Task_ReadScene() == 0){ scatter_reason = scene_reason; return 0; }
    if(camera_angle != SERVO2_LIFT){ scatter_reason = "camera_mode"; return 0; }
    if(task_scene.image_width != target_image_w || task_scene.image_height != target_image_h){
        scatter_reason = "image_size_changed"; return 0;
    }
    if(!(App_Task_Abs(servo_1.Angle - SERVO1_LIFT) < APP_TASK_SERVO_TOL)){
        scatter_reason = "frame_not_lifted"; return 0;
    }
    return App_Task_ScatterSceneClear();
}
