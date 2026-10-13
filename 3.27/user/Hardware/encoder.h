#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

/* ── 编码器参数 ────────────────────────────────────────── */
#define WHEEL_DIAMETER_MM      97.0f
#define ENCODER_SAMPLE_MS      50

/* 轮周长 = PI × 直径 */
#define WHEEL_CIRCUMFERENCE_MM  ((float)(3.14159265358979323846f * WHEEL_DIAMETER_MM))

/* ── 每路每圈脉冲数 (实测校准) ──────────────────────────── */
extern const float PULSE_PER_REV[4];

/* 每脉冲位移 (mm), 每路独立 */
#define MM_PER_PULSE(n)  (WHEEL_CIRCUMFERENCE_MM / PULSE_PER_REV[n])

/* ── 全局变量 ──────────────────────────────────────────── */
extern volatile int32_t  encoder_pulse[4];   /* 4路累计原始脉冲 */
extern volatile float    distance_mm[4];     /* 4路累计行程 mm */
extern volatile int32_t  raw_count[4];       /* ISR 写入的原始计数值 */
extern volatile uint8_t  enc_prev_state[4];  /* 每路前一状态 */

/* ── 函数声明 ──────────────────────────────────────────── */
void Encoder_Init(void);
void Encoder_Task(void);
void Encoder_ISR_Handler(uint8_t motor, uint8_t a_pin, uint8_t b_pin);
float Encoder_GetAvgDist(void);              /* 获取4轮平均行程 mm */
void Encoder_GetSideDist(float *left, float *right); /* 获取左右侧各自平均行程 mm */
void Encoder_ResetDist(void);                /* 重置距离起算点 */

#endif /* __ENCODER_H */
