//
// Created by gaoxu on 2026/10/13.
//

#include "App_Task.h" // 引入任务启动和状态机接口。
#include "app_task_move.h" // 引入运动动作及统一结果类型。
#include "app_task_vision.h" // 引入视觉选择、跟随和复检接口。
#include "app_task_scatter.h" // 引入撞散前推及旋转接口。
#include "app_usart.h" // 引入串口1格式化调试输出接口。

typedef enum { // 定义任务状态枚举，状态机按下列步骤组织运行。
    TASK_IDLE = 0,    // 停车等待按键
    TASK_FORWARD,     // 启动直行
    TASK_SEARCH_SPIN = 4, // 启动直行后持续顺时针旋转找绿色，沿用原4号搜索日志。
    TASK_RESELECT = 6, // 撞散结束后重新观察，保留原状态编号。
    TASK_FOLLOW,      // 首次对准+视觉PID接近，细节由视觉模块执行
    TASK_SCATTER,     // 复检多件/非绿色：前推，再抬框并旋转一圈
    TASK_VERIFY = 11, // 落框同时复检，提前结果等落框满500ms；原9/10号等待状态不再使用。
    TASK_ZONE_SEARCH = 14, // 寻找本队整块安全区。
    TASK_ZONE_FOLLOW, // 本队整区首次X对准、XY接近、丢失恢复及三图确认。
    TASK_ZONE_DONE, // 整区确认完成，保持载物停车。
    TASK_ZONE_TEST_PREPARE, // 开机测试落框等待，随后进入与正式版相同的整区流程。
    TASK_BACK_SEARCH = 19, // 绿色丢失/复检无命中或超时：抬框同时定时后退。
} App_Task_State; // 将以上状态枚举命名为App_Task_State。

static App_Task_State task_state = TASK_IDLE; // 当前任务状态，程序启动时默认为停车等待。
static uint32_t task_time;    // 当前任务步骤起点，ms；子动作自己维护内部计时
static uint32_t round_time;   // 第一次按键起点，后续重试不延长整轮时间
static uint8_t round_started; // 整轮是否已开始计时，静态初值为0；后续重试不清零。
static uint8_t cargo_pending; // 落框后置1；恢复动作抬框时清零，其它异常仍禁止按键盲目重启。
static uint32_t verify_attempt; // 本次落框已经开始的复检次数，重试不改变整轮起点。
static uint32_t frame_drop_time; // 本次落框命令起点；仅首次捕获更新，重试不重复机械等待。
static App_Task_Result verify_result; // WAIT表示尚无终态；提前DONE/FAILED缓存到落框满500ms。
static App_Task_VerifyFailure verify_result_failure;
static const char *verify_result_reason; // 缓存失败原因，防止后续图像或超时覆盖提前结果。

// @简介：切换任务步骤并记录开始时刻；只用于状态流转，不控制底层硬件。
static void App_Task_Change(App_Task_State state) // 函数入口：切换任务步骤并记录开始时刻；只用于状态流转，不控制底层硬件。
{ // 开始App_Task_Change函数体。
    task_state = state; // 保存新任务状态，下次处理时执行对应分支。
    task_time = HAL_GetTick(); // 记录该步骤开始的毫秒时刻，供等待及超时判断。
    if(APP_TASK_DEBUG_ENABLE){ // 仅在开启任务调试宏时输出本次调试信息。
        App_USART1_Printf("[TASK] state=%u\r\n", (unsigned int)state); // 通过串口1打印新状态编号，便于对应枚举观察流程。
    } // 结束条件分支。
} // 结束App_Task_Change函数体。

// @简介：结束本次任务并停车；不清除落框标志，异常载物需人工处理后复位。
static void App_Task_Stop(const char *reason) // 先停车，再打印退出原因及原状态，最后回到空闲。
{
    uint32_t now_ms = HAL_GetTick(); // 保存打印前的时间，避免串口耗时改变本次现场记录。
    App_Task_Move_Stop(); // 先撤销运动，不让失败打印延后停车。
    if(APP_TASK_DEBUG_ENABLE){
        App_USART1_Printf("[TASK] stop=%s state=%u dt=%lu round=%lu\r\n", reason, (unsigned int)task_state,
                         (unsigned long)(now_ms - task_time),
                         (unsigned long)(round_started ? now_ms - round_time : 0U));
    }
    App_Task_Change(TASK_IDLE); // 保留载物锁及整轮起点。
}

// @简介：停车后开始正常俯角的新视角检测；用于撞散后重选或搜索整区。
static void App_Task_BeginDetect(App_Task_State state) // 启动旋转另走保持运动的新视角入口。
{ // 开始App_Task_BeginDetect函数体。
    if(state == TASK_ZONE_SEARCH){
        App_Task_Move_Speed(APP_TASK_ZONE_SPIN_SPEED, -APP_TASK_ZONE_SPIN_SPEED);
        App_Task_Vision_BeginMoving(SERVO2_LIFT, 0U); // 直接旋转进入整区搜索，不先清零轮速。
    }else {
        App_Task_Vision_Begin(SERVO2_LIFT, APP_TASK_SETTLE_TIME_MS);
    }
    App_Task_Change(state); // 切换到指定检测状态，并记录本步骤起点。
} // 结束App_Task_BeginDetect函数体。

// 恢复动作完成后直接旋转找绿色，不经过旧静止重选的等待、空图或超时出口。
static void App_Task_BeginSearch(void)
{
    App_Task_Move_Speed(APP_TASK_ZONE_SPIN_SPEED, -APP_TASK_ZONE_SPIN_SPEED);
    App_Task_Vision_BeginMoving(SERVO2_LIFT, 0U);
    App_Task_Change(TASK_SEARCH_SPIN);
}

static void App_Task_BeginBackSearch(void)
{
    App_Task_Frame_Set(0);
    cargo_pending = 0;
    App_Task_Move_Speed(-APP_TASK_SEARCH_BACK_SPEED, -APP_TASK_SEARCH_BACK_SPEED);
    App_Task_Change(TASK_BACK_SEARCH);
}

static void App_Task_BeginVerify(void)
{
    uint32_t begin_time = verify_attempt == 1U ? frame_drop_time : HAL_GetTick();
    verify_result = APP_STEP_WAIT;
    verify_result_failure = APP_VERIFY_NONE;
    verify_result_reason = "not_checked";
    App_Task_Vision_BeginMoving(SERVO2_LIFT, 0U); // 当前已在到位停车中；重试不发机械动作或额外停车命令。
    App_Task_Change(TASK_VERIFY);
    task_time = begin_time; // 首次与落框共用起点；发送视角/打印耗时也计入本次500ms。
    if(APP_TASK_DEBUG_ENABLE){
        App_USART1_Printf("[VERIFY] begin attempt=%u/%u\r\n", (unsigned int)verify_attempt,
                         (unsigned int)APP_TASK_VERIFY_MAX_ATTEMPTS);
    }
}

static void App_Task_VerifyRetryOrRecover(App_Task_VerifyFailure failure, const char *reason)
{
    if(verify_attempt < APP_TASK_VERIFY_MAX_ATTEMPTS){
        if(APP_TASK_DEBUG_ENABLE){
            App_USART1_Printf("[VERIFY] attempt=%u/%u failed=%s next=retry\r\n",
                             (unsigned int)verify_attempt, (unsigned int)APP_TASK_VERIFY_MAX_ATTEMPTS, reason);
        }
        ++verify_attempt;
        App_Task_BeginVerify(); // 新编号、新计数和独立500ms期限，不抬框、不重复机械等待。
    }else if(failure == APP_VERIFY_MULTIPLE || failure == APP_VERIFY_WRONG_TYPE){
        App_Task_Scatter_Start(); // 立即按当前框状态前推；走够距离后才抬框。
        App_Task_Change(TASK_SCATTER);
        if(APP_TASK_DEBUG_ENABLE){
            App_USART1_Printf("[VERIFY] attempt=%u/%u failed=%s next=scatter\r\n",
                             (unsigned int)verify_attempt, (unsigned int)APP_TASK_VERIFY_MAX_ATTEMPTS, reason);
        }
    }else {
        App_Task_BeginBackSearch(); // 抬框与后退同次开始，只按时长完成，不等待舵机。
        if(APP_TASK_DEBUG_ENABLE){
            App_USART1_Printf("[VERIFY] attempt=%u/%u failed=%s next=back_search\r\n",
                             (unsigned int)verify_attempt, (unsigned int)APP_TASK_VERIFY_MAX_ATTEMPTS, reason);
        }
    }
}

// 正式启动和测试启动共用队伍、载物锁和整轮起点检查。
static uint8_t App_Task_Prepare(uint8_t team)
{ // 开始App_Task_Start函数体。
    uint32_t now_ms = HAL_GetTick(); // 读取当前毫秒计时，作为本次处理的时间基准。
    const char *reject = 0; // 只记录原有拒绝条件，不新增启动限制。
    if(task_state != TASK_IDLE){ reject = "busy"; }
    else if(cargo_pending != 0){ reject = "cargo_pending"; }
    else if(team != APP_TASK_TEAM_RED && team != APP_TASK_TEAM_BLUE){ reject = "invalid_team"; }
    else if(round_started != 0 && (uint32_t)(now_ms - round_time) >= APP_TASK_ROUND_TIME_MS){ reject = "round_timeout"; }
    if(reject != 0){
        if(APP_TASK_DEBUG_ENABLE){
            App_USART1_Printf("[TASK] reject=%s state=%u team=%u cargo=%u\r\n",
                             reject, (unsigned int)task_state, (unsigned int)team, (unsigned int)cargo_pending);
        }
        return 0;
    }
    if(round_started == 0){ // 仅第一次接受启动时建立整轮计时起点。
        round_started = 1; // 标记整轮已启动，后续重试不再重置起点。
        round_time = now_ms; // 保存本轮首次启动的毫秒时间。
    } // 结束条件分支。
    App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
    App_Task_Vision_Init(team); // 锁存本队整区类型，复位视觉PID及用途标志。
    return 1;
}

void App_Task_Start(uint8_t team)
{
    if(!App_Task_Prepare(team)){ return; }
    App_Task_Frame_Set(0); // 命令前框抬起，搜索或退离时保持抬起。
    App_Task_Camera_Set(0); // 命令摄像头恢复正常观察角度。
    App_Task_Move_Speed(APP_TASK_FORWARD_SPEED, APP_TASK_FORWARD_SPEED); // 左右轮均下发启动轮速，正值前进，使用速度闭环。
    App_Task_Change(TASK_FORWARD); // 进入启动直行状态，开始计算该段直行持续时间。
} // 结束App_Task_Start函数体。

void App_Task_StartZoneTest(uint8_t team)
{
    if(!App_Task_Prepare(team)){ return; }
    cargo_pending = 1;
    App_Task_Frame_Set(1);
    App_Task_Camera_Set(0);
    App_Task_Change(TASK_ZONE_TEST_PREPARE);
}

// @简介：非阻塞执行小车任务；只写步骤、条件与状态切换，具体动作调用封装函数。
// @说明：启动直行及旋转搜索不中途停车；找到绿色后恢复原整轮到时检查，计时起点不重置。
void App_Task_Proc(void) // 函数入口：非阻塞执行小车任务；只写步骤、条件与状态切换，具体动作调用封装函数。
{ // 开始App_Task_Proc函数体。
    App_Task_Result result; // 保存子动作的运行、等待、完成或失败结果。
    uint32_t elapsed = (uint32_t)(HAL_GetTick() - task_time); // 用无符号时间差计算当前步骤已经持续的毫秒数。
    if(task_state != TASK_FORWARD && task_state != TASK_SEARCH_SPIN // 启动直行及旋转搜索不因整轮到时中断。
       && round_started != 0 && (uint32_t)(HAL_GetTick() - round_time) >= APP_TASK_ROUND_TIME_MS){ // 其余阶段沿用原整轮时限和起点。
        if(task_state != TASK_IDLE){ App_Task_Stop("round_timeout"); } // 若仍有任务在运行则停车并回到空闲状态。
        return; // 结束本次调用，不再执行后续语句。
    } // 结束条件分支。
    switch(task_state){ // 根据当前状态执行一个非阻塞步骤。
    case TASK_FORWARD: // 处理启动后的定时直行。
        if(elapsed >= APP_TASK_FORWARD_TIME_MS){ // 直行时间达到宏定义门限后开始旋转搜索。
            App_Task_Move_Speed(APP_TASK_ZONE_SPIN_SPEED, -APP_TASK_ZONE_SPIN_SPEED); // 与寻找本队整区共用轮速，左正右负顺时针旋转。
            App_Task_Vision_BeginMoving(SERVO2_LIFT, 0U); // 已切为旋转，再发送新视角；串口等待不延长直行、不先清零轮速。
            App_Task_Change(TASK_SEARCH_SPIN); // 直接切换为持续顺时针搜索，找到绿色前不中途停车。
        }else { // 前述条件不满足时执行此分支。
            App_Task_Move_Proc(); // 维持当前目标轮速，继续执行左右轮速度闭环。
        } // 结束其它情况分支。
        break; // 结束当前状态分支，等待下一次任务处理。

    case TASK_SEARCH_SPIN: // 持续顺时针旋转，边转边用当前视角新图选择绿色优先的目标。
        if(App_Task_Vision_Select() > 0){ // 成功锁定合格目标后停止搜索旋转，交给原跟随流程。
            App_Task_Move_Stop(); // 进入首次对准前清除搜索轮速，避免继续沿用旋转命令。
            App_Task_Change(TASK_FOLLOW); // 切换到首次对准和视觉差速接近状态。
        }else { // 尚无合格目标或正在等待新图时继续顺时针旋转。
            App_Task_Move_Speed(APP_TASK_ZONE_SPIN_SPEED, -APP_TASK_ZONE_SPIN_SPEED); // 共用整区搜索轮速，不受原两侧2秒/三张空图结束条件限制。
        } // 结束其它情况分支。
        break; // 结束当前状态分支，等待下一次任务处理。

    case TASK_RESELECT: { // 处理撞散后的重选，并为局部变量建立作用域。
        int selected; // 保存目标选择结果：正数已锁定、零无候选、负数继续等待。
        selected = elapsed >= APP_TASK_DETECT_TIMEOUT_MS ? 0 : App_Task_Vision_Select(); // 撞散重选超时视为无候选，否则按当前有效新图尝试选择目标。
        if(selected > 0){ // 成功锁定合格目标时进入跟随流程。
            App_Task_Change(TASK_FOLLOW); // 切换到首次对准和视觉差速接近状态。
        }else if(selected == 0){ App_Task_Vision_LogDetails("SELECT"); App_Task_Stop(elapsed >= APP_TASK_DETECT_TIMEOUT_MS ? "reselect_timeout" : "reselect_empty"); } // 撞散重选连续有效新帧无候选或超时仍停车结束。
        break; // 结束当前状态分支，等待下一次任务处理。
    } // 结束检测状态局部作用域。

    case TASK_FOLLOW: // 处理场地物块的首次对准、差速接近和到位复核。
        result = App_Task_Vision_Follow(); // 执行原目标关联、首次对准、差速接近及到位停车复核。
        if(result == APP_STEP_DONE){ // 本步骤完成时进入后续动作分支。
            if(App_Task_Vision_IsMixed()){ // 锁定用途为撞散时进入物资堆分支。
                App_Task_Scatter_Start(); // 沿用撞散动作入口，当前选目标不会自动标记mixed。
                App_Task_Change(TASK_SCATTER);
            }else { // 前述条件不满足时执行此分支。
                cargo_pending = 1; // 落框前设置载物锁；此后异常也禁止按键盲目重启。
                frame_drop_time = HAL_GetTick();
                App_Task_Frame_Set(1); // 单独绿色走到停车位置后才落框
                verify_attempt = 1U;
                App_Task_BeginVerify(); // 落框命令发出后立即开始新视角复检，与机械动作并行。
            } // 结束其它情况分支。
        }else if(result == APP_STEP_TARGET_LOST){
            App_Task_BeginBackSearch();
        }else if(result == APP_STEP_FAILED){ App_Task_Stop("follow_failed"); } // 跟随模块报告不可恢复的失败时结束任务。
        break; // 结束当前状态分支，等待下一次任务处理。

    case TASK_SCATTER: // 处理额外前推和原地旋转。
        result = App_Task_Scatter_Proc(&cargo_pending); // 模块抬框时同步清除本次载物标志。
        if(result == APP_STEP_DONE){ // 本步骤完成时进入后续动作分支。
            App_Task_BeginSearch(); // 转满一圈后立即继续旋转搜索，不进入旧静止重选。
        }
        break; // 结束当前状态分支，等待下一次任务处理。

    case TASK_VERIFY: { // 每次独立复检；最后一次无命中/超时退后，多件/非绿色撞散。
        if(verify_result == APP_STEP_WAIT){ // 已有终态时不再读图，也不让超时覆盖缓存结果。
            if(elapsed >= APP_TASK_VERIFY_TIMEOUT_MS){ // 无缓存终态时期限优先，不接纳到期调用才处理的图。
                verify_result = APP_STEP_FAILED;
                verify_result_failure = APP_VERIFY_NO_HIT;
                verify_result_reason = "verify_timeout";
                App_Task_Vision_LogDetails("VERIFY");
            }else {
                verify_result = App_Task_Vision_Verify();
                if(verify_result == APP_STEP_FAILED){
                    verify_result_failure = App_Task_Vision_GetVerifyFailure();
                    verify_result_reason = verify_result_failure == APP_VERIFY_MULTIPLE ? "cargo_multiple"
                                         : verify_result_failure == APP_VERIFY_WRONG_TYPE ? "cargo_wrong_type" : "no_hit_in_window";
                }
            }
        }
        if(verify_result == APP_STEP_WAIT
           || (uint32_t)(HAL_GetTick() - frame_drop_time) < APP_TASK_SERVO_WAIT_MS){
            break; // 保持原到位零轮速；首次提前成功/失败均等落框满500ms才分流。
        }
        if(verify_result == APP_STEP_DONE){
            App_Task_BeginDetect(TASK_ZONE_SEARCH); // 复检通过立即发新视角寻找本队整区，不再后退或等待200ms。
        }else if(verify_result == APP_STEP_FAILED){
            App_Task_VerifyRetryOrRecover(verify_result_failure, verify_result_reason);
        }
        break; // 结束当前状态分支，等待下一次任务处理。
    }

    case TASK_BACK_SEARCH:
        if(elapsed >= APP_TASK_SEARCH_BACK_MS){
            App_Task_BeginSearch(); // 满1秒直接将后退轮速切换为顺时针搜索轮速。
        }else {
            App_Task_Move_Proc();
        }
        break;

    case TASK_ZONE_SEARCH:
        if(App_Task_Vision_SearchZone() == APP_STEP_DONE){
            App_Task_Change(TASK_ZONE_FOLLOW);
        }
        break;

    case TASK_ZONE_FOLLOW:
        result = App_Task_Vision_Follow();
        if(result == APP_STEP_DONE){
            App_Task_Move_Stop();
            App_Task_Change(TASK_ZONE_DONE); // 保留载物锁和舵机角度，不卸货、不进入下一轮。
        }else if(result == APP_STEP_RESEARCH){
            App_Task_BeginDetect(TASK_ZONE_SEARCH);
        }
        break;

    case TASK_ZONE_TEST_PREPARE:
        if(elapsed >= APP_TASK_SERVO_WAIT_MS){
            App_Task_BeginDetect(TASK_ZONE_SEARCH);
        }
        break;

    case TASK_ZONE_DONE:
        App_Task_Move_Proc(); // 已锁定零轮速；图像变化不会重新驱动车辆。
        break;

    case TASK_IDLE: // 空闲状态与默认分支共用下方退出语句。
    default: // 未知状态不下发新动作，结束本次状态处理。
        break; // 结束当前状态分支，等待下一次任务处理。
    } // 结束任务状态选择。
} // 结束App_Task_Proc函数体。
