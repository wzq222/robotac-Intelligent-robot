/**
  * @file    encoder.c
  * @brief   4路编码器 - EXTI 脉冲计数 + 4倍频正交解码
  *
  * ── 硬件接线 ────────────────────────────────────────────
  *  Motor1 编码器 A 相 → PA15    B 相 → PA12
  *  Motor2 编码器 A 相 → PB4     B 相 → PB3
  *  Motor3 编码器 A 相 → PB8     B 相 → PB5
  *  Motor4 编码器 A 相 → PB10    B 相 → PB9
  * ────────────────────────────────────────────────────────
  */

#include "encoder.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"

/* ── 每路每圈脉冲数 (10圈实测反算) ──────────────────────── */
const float PULSE_PER_REV[4] = {
    1229.0f,  /* Motor1 */
    1231.0f,  /* Motor2 */
    1229.0f,  /* Motor3 */
    1228.0f   /* Motor4 */
};

/* ── 全局变量 ──────────────────────────────────────────── */
volatile int32_t  encoder_pulse[4] = {0};
volatile float    distance_mm[4]   = {0};
volatile int32_t  raw_count[4]     = {0};
volatile uint8_t  enc_prev_state[4] = {0};

/* ── 编码器引脚宏 ─────────────────────────────────────── */
#define M1_A()  GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15)
#define M1_B()  GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12)
#define M2_A()  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)
#define M2_B()  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3)
#define M3_A()  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)
#define M3_B()  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)
#define M4_A()  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)
#define M4_B()  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)

/* ── 正交解码查找表 ───────────────────────────────────── */
static const int8_t quad_lut[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
};

/* ── 距离起算基准 (Encoder_ResetDist 设置) ──────────────── */
static float dist_ref_mm[4] = {0};

/*******************************************************************************
 * @brief  4路编码器初始化 (EXTI 双边沿触发, 软件 4 倍频解码)
 ******************************************************************************/
void Encoder_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;
    EXTI_InitTypeDef   EXTI_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;

    /* ── 使能时钟 ── */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO, ENABLE);

    /* ── 关键：关闭 JTAG，释放 PA15/PB3/PB4 为 GPIO ──
     * 保留 SWD (PA13/PA14) 用于调试，必须放在 AFIO 时钟使能之后，
     * 否则拔掉调试器后 JTAG 引脚浮空，EXTI 中断风暴会淹没 100Hz 控制循环 */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* ── GPIO: 浮空输入 ── */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                                  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ── AFIO EXTI 映射 ── */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource12); /* M1_B */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15); /* M1_A */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);  /* M2_B */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);  /* M2_A */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);  /* M3_B */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);  /* M3_A */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);  /* M4_B */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10); /* M4_A */

    /* ── EXTI 通用配置: 双边沿触发 ── */
    EXTI_InitStructure.EXTI_Mode     = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger  = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd  = ENABLE;

    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line8;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line9;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line10;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line12;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line15;
    EXTI_Init(&EXTI_InitStructure);

    /* ── NVIC ── */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    /* 初始化每路编码器前一状态 */
    enc_prev_state[0] = (M1_A() << 1) | M1_B();
    enc_prev_state[1] = (M2_A() << 1) | M2_B();
    enc_prev_state[2] = (M3_A() << 1) | M3_B();
    enc_prev_state[3] = (M4_A() << 1) | M4_B();
}

/*******************************************************************************
 * @brief  编码器 ISR 核心: 正交解码查表, 更新 raw_count
 ******************************************************************************/
void Encoder_ISR_Handler(uint8_t motor, uint8_t a_pin, uint8_t b_pin)
{
    uint8_t prev = enc_prev_state[motor];
    uint8_t cur  = (a_pin << 1) | b_pin;
    int8_t  step;

    if (cur == prev) return;  /* 去抖 */

    step = quad_lut[(prev << 2) | cur];
    if (step != 0) {
        raw_count[motor] += step;
    }
    enc_prev_state[motor] = cur;
}

/*******************************************************************************
 * @brief  编码器采集任务: 消费 ISR 的 raw_count, 更新 encoder_pulse + distance_mm
 ******************************************************************************/
void Encoder_Task(void)
{
    static int32_t last_raw[4] = {0};
    int32_t delta;
    uint8_t i;

    for (i = 0; i < 4; i++) {
        delta = raw_count[i] - last_raw[i];
        last_raw[i] = raw_count[i];

        if (delta == 0) continue;

        /* M1(左后)和M4(左前)编码器接线与M2/M3反相，前进时计数为负，取反统一方向 */
        if (i == 0 || i == 3) delta = -delta;

        encoder_pulse[i] += delta;
        distance_mm[i] += (float)delta * MM_PER_PULSE(i);
    }
}

/*******************************************************************************
 * @brief  获取4轮平均行程 (自上一次 Encoder_ResetDist 起)
 ******************************************************************************/
float Encoder_GetAvgDist(void)
{
    float sum = 0;
    uint8_t i;
    for (i = 0; i < 4; i++) {
        sum += distance_mm[i] - dist_ref_mm[i];
    }
    return sum / 4.0f;
}

/*******************************************************************************
 * @brief  获取左右侧各自的平均行程 (自上一次 Encoder_ResetDist 起)
 *         left:  (M1 + M4) / 2, right: (M2 + M3) / 2
 ******************************************************************************/
void Encoder_GetSideDist(float *left, float *right)
{
    *left  = ((distance_mm[0] - dist_ref_mm[0]) + (distance_mm[3] - dist_ref_mm[3])) * 0.5f;
    *right = ((distance_mm[1] - dist_ref_mm[1]) + (distance_mm[2] - dist_ref_mm[2])) * 0.5f;
}

/*******************************************************************************
 * @brief  重置距离起算点 (进入新阶段时调用)
 ******************************************************************************/
void Encoder_ResetDist(void)
{
    uint8_t i;
    for (i = 0; i < 4; i++) {
        dist_ref_mm[i] = distance_mm[i];
    }
}
