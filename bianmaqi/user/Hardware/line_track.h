#ifndef __LINE_TRACK_H
#define __LINE_TRACK_H

#include <stdint.h>

/*
 * 巡线模块接口
 * 基于8路红外传感器 + PID闭环控制 + 麦克纳姆轮运动学
 *
 * 电机布局（俯视图）：
 *     前进方向 ↑
 *   M4(左前)  M3(右前)
 *   M1(左后)  M2(右后)
 */

/* 初始化巡线模块，base_speed为直行基础PWM控制量（建议50） */
void LineTrack_Init(float base_speed);

/* 简单巡线模式初始化（右侧场地，无路口判定） */
void LineTrack_InitSimple(float base_speed);

/* 每个控制周期调用一次（建议10ms），dt为周期时间(秒) */
void LineTrack_Update(float dt);

/* 停止巡线，电机归零，PID复位 */
void LineTrack_Stop(void);

/* 获取当前线位置偏差（调试用），范围约 -1.0 ~ +1.0 */
float LineTrack_GetPosition(void);

#endif
