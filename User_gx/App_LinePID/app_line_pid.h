//
// Created by gaoxu on 2026/10/17.
//

#ifndef INTELLIGENT_RESCUE_APP_LINE_PID_H
#define INTELLIGENT_RESCUE_APP_LINE_PID_H

/**
 * @brief 初始化横向视觉 PID 的参数和纠偏状态。
 *
 * @param[in] kp 比例系数，决定当前横向偏差产生多大的修正量。
 * @param[in] ki 积分系数，用于修正长期累积的横向偏差。
 * @param[in] kd 微分系数，用于根据横向偏差的变化速度调整输出。
 * @param[in] max_turn 修正量绝对值的上限，用于轮速差速时单位为 cm/s。
 * @param[in] steer_sign 输出方向。+1 沿用默认符号，-1 反转符号。
 *
 * @pre @p max_turn 大于 0，@p steer_sign 为 +1 或 -1。
 * @note 本函数无返回值。首次计算前调用，初始化后纠偏处于允许状态。
 */
void App_LinePID_Init(float kp, float ki, float kd, float max_turn, float steer_sign);

/**
 * @brief 清除上次采样时间、误差和积分，恢复允许纠偏的状态。
 *
 * @details 保留初始化时设置的 PID 系数、限幅和转向符号。
 * 换目标或切换控制阶段时调用，避免把上一个目标的历史带到下一个目标。
 *
 * @pre 已调用 App_LinePID_Init()。
 * @note 本函数无输入参数，也无返回值。
 */
void App_LinePID_Reset(void);

/**
 * @brief 根据目标中心 X 与期望 X 的偏差计算左右轮差速修正量。
 *
 * @param[in] object_cx 当前检测框中心的 X 坐标，单位为像素。
 * @param[in] aim_x 希望检测框中心对准的 X 坐标，单位为像素。
 * @param[in] enter 内门限，偏差绝对值小于或等于该值时关闭纠偏。
 * @param[in] exit 外门限，偏差在内门限外且绝对值达到该值时恢复纠偏。
 *
 * @return 带符号的修正量，绝对值不超过初始化时设置的限幅。纠偏关闭时返回 0。
 *         调用方用此值计算左右轮速；此函数只计算数值。
 *
 * @details 误差等于 @p object_cx 减 @p aim_x。误差在两个门限之间时，
 *          沿用上一次的开关状态；两门限相等时，边界按内门限处理。
 *
 * @pre 已完成初始化，四个输入使用同一像素尺度，且 0 <= @p enter <= @p exit。
 * @note 每张新图调用一次，连续 PID 计算的采样时间须递增。
 */
float App_LinePID_ComputeTarget(float object_cx, float aim_x, float enter, float exit);

#endif //INTELLIGENT_RESCUE_APP_LINE_PID_H
