/*
 * line_track.c - 巡线控制模块（带任务调度）
 *
 * 支持场地左侧和右侧两种启动位置，第一个路口自动检测切换：
 *   - 右侧传感器(x6-x7)检测到白线 → 右转路口 → 使用左侧场地任务表
 *   - 左侧传感器(x0-x1)检测到白线 → 左转路口 → 使用右侧场地任务表
 *
 * 工作流程：
 *   FOLLOW(PID跟线 + 路口检测)
 *      ↓ 任一边沿持续亮 → 查任务表，串口打印 STEP n/31
 *   - 若 PASS：盲走穿过路口，回 FOLLOW
 *   - 若 LEFT/RIGHT：ADVANCE → BRAKE → ROTATE(90°) → [端点] → FOLLOW
 *   - 若 FINDLINE：ADVANCE → BRAKE → FINDLINE_ROTATE(CW转到压线) → FOLLOW
 *   端点类型(转弯/PASS后接着执行)：
 *   - M_END_180  ：巡线直行2s → 停3s → 后退(base_speed) → 自转180° → FOLLOW
 *   - M_END_90   ：巡线直行2s → 停3s → 后退(base_speed) → 自转90° → FOLLOW
 *   - M_END_FINAL：巡线直行3s → 永久停车
 *
 * 电机布局（俯视图）：
 *     前进方向 ↑
 *   M4(左前)  M3(右前)
 *   M1(左后)  M2(右后)
 */
#include "line_track.h"
#include "pid.h"
#include "motor.h"
#include "USART2.h"
#include "claw.h"
#include "encoder.h"
#include <stdlib.h>
#include <stdio.h>

/* ========== 路口拐弯时序参数 ========== */
/* 路口前冲距离 mm */
#define ADVANCE_MM_DEFAULT    70.0f

/* 每个路口独立的前冲距离 mm（索引 0~30，对应 31 个路口），两侧场地配置时填入 */
static float s_advance_mm[31];
/* 当前路口实际使用的前冲距离，由 DispatchEvent / Feature2 从上面数组取值 */
static float s_advance_dist = ADVANCE_MM_DEFAULT;
/* 功能2 各阶段路口前冲距离（索引 0~2 对应 Phase A/B/C） */
static float s_f2_advance_mm[3] = { ADVANCE_MM_DEFAULT, ADVANCE_MM_DEFAULT, ADVANCE_MM_DEFAULT };

#define BRAKE_TICKS_LONG 30U       /* 0.3秒滑停（松电机） */
#define TURN_SPEED 100.0f          /* 自转PWM */

/* 找线拐弯（右拐 CW，转到中间两路压到白线就停，无固定角度） */
#define FINDLINE_MIN_TICKS 60U     /* 起转后至少转这么多 tick 才开始检测，避免立刻压到来线 */
#define FINDLINE_MAX_TICKS 216U    /* 安全上限：转这么久还没找到线就放弃回 FOLLOW */

/* ========== 端点序列距离阈值 (mm) - 改用编码器测距 ========== */
/* 默认值，实际由场地配置函数覆写 */
#define REL_DRIVE_MM_DEFAULT    400.0f   /* 释放：巡线直行 */
#define REL_REVERSE_MM_DEFAULT  400.0f   /* 释放：后退 */
#define GRB_DRIVE_MM_DEFAULT    400.0f   /* 抓取：巡线直行 */
#define GRB_REVERSE_MM_DEFAULT  400.0f   /* 抓取：后退 */

static float s_release_drive_mm   = REL_DRIVE_MM_DEFAULT;
static float s_release_reverse_mm = REL_REVERSE_MM_DEFAULT;
static float s_grab_drive_mm      = GRB_DRIVE_MM_DEFAULT;
static float s_grab_reverse_mm    = GRB_REVERSE_MM_DEFAULT;

/* 当前步骤实际使用的 drive 目标，由 StartEndpoint 根据 s_end_claw_action 选取 */
static float s_end_drive_dist    = REL_DRIVE_MM_DEFAULT;

/* 功能2 高速直行距离 mm */
#define F2_SPRINT_MM           650.0f
/* 功能2 后退距离 mm */
#define F2_REVERSE_MM          400.0f

#define END_STOP_TICKS 300U              /* 停车3s（时间基数） */

/* PASS 通过路口的盲走时间：直行不用PID，避免被横线拽偏到分支 */
#define PASS_THROUGH_TICKS 30U     /* 300ms，穿过横线后再恢复检测 */

/* ========== 巡线参数 ========== */
static const float SENSOR_WEIGHTS[8] = {
    -2.5f, -1.5f, -1.0f, -0.3f, 0.3f, 1.0f, 1.5f, 2.5f
};

#define SENSOR_RAW_MAX 4096U

/* 每个传感器独立的白线/黑线阈值（反相值），可在 ApplyXxxSideConfig 中单独调 */
/* 白线（路口判定）：ReadInverted > 阈值 → 白线；x0左/x7右 */
static uint16_t s_hit_threshold[8] = {
    3796U, 3530U, 3500U, 3520U, 3585U, 3500U, 3535U, 3796U
};
/* 黑线（PID巡线）：ReadInverted < 阈值 → 黑线；x2-x5 */
static uint16_t s_line_threshold[8] = {
    3800U, 3530U, 3500U, 3520U, 3585U, 3500U, 3535U, 3800U
};
#define EDGE_DEBOUNCE_TICKS 6U          /* 60ms 持续亮才触发 */
#define EDGE_REARM_TICKS 7U             /* 触发后须边沿熄灭持续70ms才允许再次检测 */

/* 冷却时间（10ms/tick，冷却期内不检测路口） */
#define COOLDOWN_AFTER_PASS 10U         /* PASS 后 100ms */
#define COOLDOWN_AFTER_TURN 40U         /* 转弯后 400ms */
#define COOLDOWN_AFTER_ENDPOINT 10U     /* 端点序列后 100ms */

/* ========== 任务调度 ========== */

/* 路口动作 */
typedef enum {
    M_LEFT     = 0,    /* 左拐 90° (CCW) */
    M_RIGHT    = 1,    /* 右拐 90° (CW) */
    M_PASS     = 2,    /* 不拐弯，继续直行通过 */
    M_FINDLINE = 3,    /* 找线拐弯：前进后顺时针(CW)自转，中间两路压到白线就停 */
} MissionTurn;

/* 端点序列类型 */
typedef enum {
    M_END_NONE  = 0,   /* 无端点序列，转弯后直接回 FOLLOW */
    M_END_180   = 1,   /* 180°端点：巡线直行2s → 停3s → 纯直行1s → 后退 → 自转180° → 回巡线 */
    M_END_90    = 2,   /* 90°端点：巡线直行2s → 停3s → 纯直行1s → 后退 → 自转90° → 回巡线 */
    M_END_FINAL = 3,   /* 最终端点：巡线直行3s → 永久停车 */
} MissionEnd;

typedef struct {
    MissionTurn turn;
    MissionEnd end;
} MissionEvent;

/*
 * ========== 两套完整任务表（各 31 个事件） ==========
 * 每个事件 = 一次路口检测时执行的动作
 * 固定路口编号见赛道图 1~8
 */

/* 左侧场地任务表（从场地左侧出发 → 转弯方向以右拐为主） */
static const MissionEvent mission_left_side[] = {
    /*  1 */ { M_RIGHT,    M_END_NONE  },   /* 路口1 右拐90° */
    /*  2 */ { M_LEFT,     M_END_180   },   /* 路口2 左拐90° + 180°端点 */
    /*  3 */ { M_PASS,     M_END_NONE  },   /* 路口3 直行通过 */
    /*  4 */ { M_PASS,     M_END_NONE  },   /* 路口4 直行通过 */
    /*  5 */ { M_RIGHT,    M_END_90    },   /* 路口5 右拐90° + 90°端点 */
    /*  6 */ { M_PASS,     M_END_NONE  },   /* 路口4 直行通过 */
    /*  7 */ { M_RIGHT,    M_END_180   },   /* 路口3 右拐90° + 180°端点 */
    /*  8 */ { M_PASS,     M_END_NONE  },   /* 路口4 直行通过 */
    /*  9 */ { M_PASS,     M_END_NONE  },   /* 路口5 直行通过 */
    /* 10 */ { M_RIGHT,    M_END_90    },   /* 路口6 右拐90° + 90°端点 */
    /* 11 */ { M_PASS,     M_END_NONE  },   /* 路口5 直行通过 */
    /* 12 */ { M_RIGHT,    M_END_180   },   /* 路口4 右拐90° + 180°端点 */
    /* 13 */ { M_PASS,     M_END_NONE  },   /* 路口5 直行通过 */
    /* 14 */ { M_PASS,     M_END_NONE  },   /* 路口6 直行通过 */
    /* 15 */ { M_RIGHT,    M_END_90    },   /* 路口7 右拐90° + 90°端点 */
    /* 16 */ { M_PASS,     M_END_NONE  },   /* 路口6 直行通过 */
    /* 17 */ { M_PASS,     M_END_NONE  },   /* 路口5 直行通过 */
    /* 18 */ { M_PASS,     M_END_NONE  },   /* 路口4 直行通过 */
    /* 19 */ { M_PASS,     M_END_NONE  },   /* 路口3 直行通过 */
    /* 20 */ { M_RIGHT,    M_END_180   },   /* 路口2 右拐90° + 180°端点 */
    /* 21 */ { M_PASS,     M_END_NONE  },   /* 路口3 直行通过 */
    /* 22 */ { M_PASS,     M_END_NONE  },   /* 路口4 直行通过 */
    /* 23 */ { M_PASS,     M_END_NONE  },   /* 路口5 直行通过 */
    /* 24 */ { M_PASS,     M_END_NONE  },   /* 路口6 直行通过 */
    /* 25 */ { M_PASS,     M_END_NONE  },   /* 路口7 直行通过 */
    /* 26 */ { M_RIGHT,    M_END_90    },   /* 路口8 右拐90° + 90°端点 */
    /* 27 */ { M_PASS,     M_END_NONE  },   /* 路口7 直行通过 */
    /* 28 */ { M_PASS,     M_END_NONE  },   /* 路口6 直行通过 */
    /* 29 */ { M_PASS,     M_END_NONE  },   /* 路口5 直行通过 */
    /* 30 */ { M_PASS,     M_END_NONE  },   /* 路口4 直行通过 */
    /* 31 */ { M_RIGHT,    M_END_90    },   /* 路口3 右拐90° + 90°端点(进功能2) */
};

/* 右侧场地任务表（从场地右侧出发 → 转弯方向以左拐为主） */
static const MissionEvent mission_right_side[] = {
    /*  1 */ { M_LEFT,     M_END_NONE  },   /* 路口1 左拐90° */
    /*  2 */ { M_RIGHT,    M_END_180   },   /* 路口2 右拐90° + 180°端点 */
    /*  3 */ { M_PASS,     M_END_NONE  },   /* 路口3 直行通过 */
    /*  4 */ { M_PASS,     M_END_NONE  },   /* 路口4 直行通过 */
    /*  5 */ { M_LEFT,     M_END_90    },   /* 路口5 左拐90° + 90°端点 */
    /*  6 */ { M_PASS,     M_END_NONE  },   /* 路口4 直行通过 */
    /*  7 */ { M_LEFT,     M_END_180   },   /* 路口3 左拐90° + 180°端点 */
    /*  8 */ { M_PASS,     M_END_NONE  },   /* 路口4 直行通过 */
    /*  9 */ { M_PASS,     M_END_NONE  },   /* 路口5 直行通过 */
    /* 10 */ { M_LEFT,     M_END_90    },   /* 路口6 左拐90° + 90°端点 */
    /* 11 */ { M_PASS,     M_END_NONE  },   /* 路口5 直行通过 */
    /* 12 */ { M_LEFT,     M_END_180   },   /* 路口4 左拐90° + 180°端点 */
    /* 13 */ { M_PASS,     M_END_NONE  },   /* 路口5 直行通过 */
    /* 14 */ { M_PASS,     M_END_NONE  },   /* 路口6 直行通过 */
    /* 15 */ { M_LEFT,     M_END_90    },   /* 路口7 左拐90° + 90°端点 */
    /* 16 */ { M_PASS,     M_END_NONE  },   /* 路口6 直行通过 */
    /* 17 */ { M_PASS,     M_END_NONE  },   /* 路口5 直行通过 */
    /* 18 */ { M_PASS,     M_END_NONE  },   /* 路口4 直行通过 */
    /* 19 */ { M_PASS,     M_END_NONE  },   /* 路口3 直行通过 */
    /* 20 */ { M_LEFT,     M_END_180   },   /* 路口2 左拐90° + 180°端点 */
    /* 21 */ { M_PASS,     M_END_NONE  },   /* 路口3 直行通过 */
    /* 22 */ { M_PASS,     M_END_NONE  },   /* 路口4 直行通过 */
    /* 23 */ { M_PASS,     M_END_NONE  },   /* 路口5 直行通过 */
    /* 24 */ { M_PASS,     M_END_NONE  },   /* 路口6 直行通过 */
    /* 25 */ { M_PASS,     M_END_NONE  },   /* 路口7 直行通过 */
    /* 26 */ { M_LEFT,     M_END_90    },   /* 路口8 左拐90° + 90°端点 */
    /* 27 */ { M_PASS,     M_END_NONE  },   /* 路口7 直行通过 */
    /* 28 */ { M_PASS,     M_END_NONE  },   /* 路口6 直行通过 */
    /* 29 */ { M_PASS,     M_END_NONE  },   /* 路口5 直行通过 */
    /* 30 */ { M_PASS,     M_END_NONE  },   /* 路口4 直行通过 */
    /* 31 */ { M_LEFT,     M_END_90    },   /* 路口3 左拐90° + 90°端点(进入Phase 0) */
};

/* 活动任务表指针及其长度，在第一个路口时确定 */
static const MissionEvent *s_active_mission = NULL;
static uint16_t s_mission_length = 0;

/* 场地侧检测: 0=未知, 1=左侧场地, 2=右侧场地 */
static uint8_t s_arena_side = 0;

/* 任务进度索引：当前要消耗的事件 */
static uint16_t s_mission_index = 0;

/* ========== 状态机 ========== */
typedef enum {
    STATE_FOLLOW = 0,           /* PID跟线 + 路口检测 */
    STATE_PASS_THROUGH,         /* PASS：盲走直行穿过路口（不用PID） */
    STATE_TURN_ADVANCE,         /* 转弯前推进200mm（PID跟线测距） */
    STATE_TURN_BRAKE,           /* 转弯前刹车 */
    STATE_TURN_ROTATE,          /* 90° 固定自转（左/右拐） */
    STATE_FINDLINE_ROTATE,      /* 找线拐弯：CW自转直到中间两路压线 */
    STATE_END_DRIVE,            /* 端点：巡线直行（2s或3s，按端点类型） */
    STATE_END_STOP,             /* 端点：停3s */
    STATE_END_REVERSE,          /* 端点：后退（base_speed），之后直接进自转 */
    STATE_END_STRAIGHT,         /* 端点 90°：纯直行 1s（速度50）→ 进自转 */
    STATE_END_ROTATE,           /* 端点：自转（90° 或 180°，方向按场地侧） */
    STATE_FINAL_STOP,           /* 任务完成，永久停下 */

    /* ==== 简单巡线模式（无路口判定）==== */
    STATE_SM_INIT,              /* 简单模式初始化入口 */
    STATE_SM_FOLLOW,            /* 简单模式 PID巡线 */
    STATE_SM_STRAIGHT,          /* 简单模式 纯直行 */
    STATE_SM_REVERSE,           /* 简单模式 后退 */
    STATE_SM_ROTATE_FINDLINE,   /* 简单模式 自转找白线 */
    STATE_SM_HARD_90,           /* 简单模式 硬转90° */
    STATE_SM_STRAIGHT_WHITE,    /* 简单模式 直行到x2-x5全白 */
    STATE_SM_CLAW_OPEN,         /* 简单模式 张爪 */
    STATE_SM_CLAW_STOP,         /* 简单模式 夹爪停止+动作 */
    STATE_SM_CLAW_REVERSE,      /* 简单模式 夹爪后退 */
    STATE_SM_CLAW_ROTATE,       /* 简单模式 夹爪自转回线 */

    /* ==== 功能2 状态机 ==== */
    STATE_F2_WAIT_30S,          /* 功能2：等待 30s */
    STATE_F2_ADVANCE,           /* 功能2：巡线前进（检测到路口后右拐） */
    STATE_F2_SPRINT,            /* 功能2：速度50 直行前进 */
    STATE_F2_REVERSE,           /* 功能2：后退 */
    STATE_F2_ROTATE,            /* 功能2：自转（方向可变） */
    STATE_F2_BYPASS,            /* 功能2：PASS 跳过路口 */
} TrackState;

/* 转弯方向 */
typedef enum {
    DIR_NONE = 0,
    DIR_CCW  = 1,    /* 逆时针/左转 */
    DIR_CW   = 2,    /* 顺时针/右转 */
} TurnDir;

static TrackState s_state = STATE_FOLLOW;
static TurnDir s_turn_dir = DIR_NONE;

/* 当前事件的端点类型（在 ROTATE 完成后用来决定下一步） */
static MissionEnd s_pending_end = M_END_NONE;

/* 当前事件的转向类型（区分固定90°拐弯 与 找线拐弯） */
static MissionTurn s_pending_turn = M_PASS;

/* 端点自转方向（运行时按场地侧确定：左侧=CW, 右侧=CCW） */
static TurnDir   s_end_rotate_dir = DIR_CCW;   /* 默认右侧场地值 */

/* PID Ki 已移除（=0），简化调试 */

/* 各类时间计数器 */
static uint32_t s_advance_counter = 0;
static uint32_t s_brake_counter = 0;
static uint32_t s_rotate_counter = 0;
static uint32_t s_end_counter = 0;
static uint32_t s_pass_counter = 0;
static uint32_t s_findline_counter = 0;    /* 找线拐弯自转计数 */
static uint32_t s_end_drive_target = 300;  /* 端点直行目标 tick 数 */
static uint32_t s_rotate_bounce = 0;       /* 检测到白线后反向制动计数（0.05s） */

/* 边沿连续亮的计数器（去抖动用） */
static uint8_t s_left_bright_count = 0;
static uint8_t s_right_bright_count = 0;

/* 消隐复位：触发一次路口后，须等边沿熄灭才重新武装检测 */
static uint8_t s_edge_rearm = 0;
static uint8_t s_edge_off_count = 0;

/* 锁：转弯/端点期间禁止读传感器 */
static volatile uint8_t s_locked = 0;

static uint32_t s_cooldown = 0;          /* 路口冷却计数 */

/* 起步保护：起步后 0.15s 内不检测路口，只做 PID 巡线 */
#define STARTUP_GUARD_TICKS 50U
static uint32_t s_startup_guard = 0;

/* ========== 功能2 控制变量 ========== */
static uint8_t  s_f2_enabled = 0;     /* 功能2 是否激活（右侧场地最后一个端点完成后） */
static uint8_t  s_f2_phase   = 0;     /* 当前阶段: 0=等待30s, 1=Phase A, 2=Phase B, 3=Phase C */
static uint32_t s_f2_counter = 0;     /* 功能2 通用计数器 */
static TurnDir  s_f2_rotate_dir = DIR_NONE; /* 功能2 自转方向 */
static float    s_f2_base_speed  = 50.0f;   /* 功能2 高速直行速度 */
static float s_f2_sprint_dist   = 0;         /* 功能2 直行距离目标 mm */
static uint8_t  s_f2_bypass_done = 0;       /* 功能2 Phase B 的 PASS 是否已完成 */
static uint8_t  s_f2_stall_count = 0;       /* 功能2 堵转确认计数（连续几帧检测到堵转） */
static float    s_f2_last_dist[4] = {0};    /* 功能2 堵转检测：上一帧各轮行程 mm */
static uint8_t  s_end_stall_count = 0;      /* 端点前进堵转确认计数 */
static float    s_end_last_dist[4] = {0};   /* 端点前进堵转检测：上一帧各轮行程 mm */
/* 0=无动作, 1=180°释放(P1=44→1s后P2=90→2s后P1=80), 2=抓取(P1=40→1s后P2=0) */
static uint8_t  s_end_claw_action = 0;
static uint32_t s_end_stop_target = 0;   /* 端点 STOP 目标 tick 数，0=使用默认 */
static uint8_t  s_end_rotate_guard = 0;  /* 端点自转前先转不检测的时间 */

/* 功能2 函数前置声明 */
static void StartFeature2(void);
static void Feature2HandleJunction(void);
static void Feature2AfterTurn(void);
static void ReverseBalanceReset(void);
static uint8_t EndCheckStall(void);

/* ========== 巡线参数 ========== */
static float s_last_position = 0.0f;
static float s_base_speed = 0.0f;
static PID_Controller s_turn_pid;
static uint8_t s_pid_first_call = 1;  /* 首次 RUN 跳过导数避免冲击 */

/* ========== 场地侧切换函数 ========== */

/* 切换为左侧场地配置：切换到左侧出发的任务表及参数 */
static void ApplyLeftSideConfig(void)
{
    uint8_t i;
    s_active_mission    = mission_left_side;
    s_mission_length    = sizeof(mission_left_side) / sizeof(mission_left_side[0]);
    /* 所有路口前冲默认 70mm，可按路口编号单独调 */
    for (i = 0; i < 31; i++) s_advance_mm[i] = ADVANCE_MM_DEFAULT;
    s_advance_mm[0]      = 95.0f;  /* 路口1 */
    s_release_drive_mm   = 450.0f;
    s_release_reverse_mm = 360.0f;
    s_grab_drive_mm      = GRB_DRIVE_MM_DEFAULT;
    s_grab_reverse_mm    = 360.0f;
    s_f2_advance_mm[0]   = ADVANCE_MM_DEFAULT; /* Phase A */
    s_f2_advance_mm[1]   = 60.0f; /* Phase B */
    s_f2_advance_mm[2]   = ADVANCE_MM_DEFAULT; /* Phase C */
    s_end_rotate_dir    = DIR_CW;
}

/* 切换为右侧场地配置（也是默认配置）：切换到右侧出发的任务表及参数 */
static void ApplyRightSideConfig(void)
{
    uint8_t i;
    s_active_mission    = mission_right_side;
    s_mission_length    = sizeof(mission_right_side) / sizeof(mission_right_side[0]);
    /* 所有路口前冲默认 85mm，可按路口编号单独调 */
    for (i = 0; i < 31; i++) s_advance_mm[i] = 85.0f;
    s_advance_mm[0]      = 95.0f;  /* 路口1 */
    s_advance_mm[1]      = 70.0f;  /* 路口2 右拐+180°端点 */
    s_release_drive_mm   = 450.0f;
    s_release_reverse_mm = 360.0f;
    s_grab_drive_mm      = 450.0f;
    s_grab_reverse_mm    = 360.0f;
    s_f2_advance_mm[0]   = 50.0f; /* Phase A */
    s_f2_advance_mm[1]   = ADVANCE_MM_DEFAULT; /* Phase B */
    s_f2_advance_mm[2]   = 50.0f; /* Phase C */
    s_end_rotate_dir    = DIR_CCW;
}

/* ========== 工具函数 ========== */

static void SetMotors(int left_pwm, int right_pwm)
{
    Set_Motor1_RPM(left_pwm);
    Set_Motor4_RPM(left_pwm);
    Set_Motor2_RPM(-right_pwm);
    Set_Motor3_RPM(-right_pwm);
}

static void SetRotateMotors(TurnDir dir)
{
    float speed = (s_f2_enabled) ? s_f2_base_speed : TURN_SPEED;
    if (dir == DIR_CCW)
        SetMotors(-(int)speed, (int)speed);
    else if (dir == DIR_CW)
        SetMotors((int)speed, -(int)speed);
    else
        SetMotors(0, 0);
}

/* 串口打印当前步骤号：STEP n/31\r\n（纯文本，不含 $ #，避免干扰八路模块）*/
static void PrintStep(uint16_t step)
{
    char buf[20];
    sprintf(buf, "STEP %u/%u\r\n", (unsigned)step, (unsigned)s_mission_length);
    USART2_SendString(buf);
}

static uint16_t ReadInverted(uint8_t i)
{
    uint16_t raw = ir_data[i];
    if (raw >= SENSOR_RAW_MAX) return 0;
    return (uint16_t)(SENSOR_RAW_MAX - raw);
}

static uint8_t LeftEdgeIsClearBright(void)
{
    if (ReadInverted(0) <= s_hit_threshold[0]) return 0;
    return 1;
}

static uint8_t RightEdgeIsClearBright(void)
{
    if (ReadInverted(7) <= s_hit_threshold[7]) return 0;
    return 1;
}

/* 找线拐弯用：中间两路(x3,x4)压到白线才算找到主干线 */
static uint8_t MiddleOnLine(void)
{
    if (ReadInverted(3) > s_hit_threshold[3]) return 1;
    return 0;
}

/* 计算线位置（巡线 PID 用）—— x1,x2(左) 和 x5,x6(右) */
static float CalcLinePosition(void)
{
    static const uint8_t pid_sensors[] = {1, 2, 5, 6};
    float sum_weighted = 0.0f;
    float sum_strength = 0.0f;
    uint8_t j;

    /* 路口过滤：|x0 - x7| > 1000 说明一侧白线一侧黑线(路口)，舍弃本帧不做PID */
    {
        uint16_t x0 = ReadInverted(0);
        uint16_t x7 = ReadInverted(7);
        int32_t diff = (int32_t)x0 - (int32_t)x7;
        if (diff < 0) diff = -diff;
        if (diff > 1000)
            return s_last_position;
    }

    for (j = 0; j < 4; j++)
    {
        uint8_t i = pid_sensors[j];
        uint16_t inv = ReadInverted(i);
        if (inv < s_line_threshold[i])
        {
            float v = (float)(s_line_threshold[i] - inv);
            sum_weighted += SENSOR_WEIGHTS[i] * v;
            sum_strength += v;
        }
    }

    if (sum_strength < 1.0f)
        return s_last_position;

    float pos = sum_weighted / sum_strength;
    s_last_position = pos;
    return pos;
}

/* 用 PID 跟线驱动电机（直行 + 巡线纠偏） */
static void DriveWithPID(float dt)
{
    float position = CalcLinePosition();

    /* 首次调用：纯直行，同时用实际位置预填 prev_error，避免微分冲击 */
    if (s_pid_first_call)
    {
        s_pid_first_call = 0;
        s_turn_pid.prev_error = 0.0f - position;

        SetMotors((int)s_base_speed, (int)s_base_speed);
        return;
    }

    float W = PID_Compute(&s_turn_pid, 0.0f, position, dt);
    float reach_edge = 100.0f;
    /* W 限幅：超出 ±30 则钳位，防止纠偏过猛 */
    if (W >  reach_edge) W =  reach_edge;
    if (W < -reach_edge) W = -reach_edge;
    
    int left  = (int)(s_base_speed + W);
    int right = (int)(s_base_speed - W);

    /* 直行纠偏时若某侧PWM为负（倒转），把负值移到对侧加力，同侧置0 */
    if (left < 0)  { right -= left;  left  *= 0.1; }
    if (right < 0) { left  -= right; right *= 0.1; }

    SetMotors(left, right);
}

/* ========== 公开接口 ========== */

float LineTrack_GetPosition(void)
{
    return s_last_position;
}

void LineTrack_Init(float base_speed)
{
    s_base_speed = base_speed;
    s_last_position = 0.0f;
    s_state = STATE_FOLLOW;
    s_cooldown = 0;
    s_left_bright_count = 0;
    s_right_bright_count = 0;
    s_edge_rearm = 0;
    s_edge_off_count = 0;
    s_turn_dir = DIR_NONE;
    s_pending_end = M_END_NONE;
    s_pending_turn = M_PASS;
    s_advance_counter = 0;
    s_brake_counter = 0;
    s_rotate_counter = 0;
    s_end_counter = 0;
    s_pass_counter = 0;
    s_findline_counter = 0;
    s_rotate_bounce = 0;
    s_end_drive_dist = 0;
    s_locked = 0;
    s_mission_index = 0;
    s_end_claw_action = 0;
    s_end_stop_target = END_STOP_TICKS;
    s_end_rotate_guard = 0;
    s_pid_first_call = 1;
    s_startup_guard = STARTUP_GUARD_TICKS;

    /* 默认右侧场地配置（第一个路口检测到左转时不变，检测到右转时切换） */
    s_arena_side = 0;  /* 未知，等待第一个路口检测 */
    ApplyRightSideConfig();

    PID_Init(&s_turn_pid, 18.0f, 0.8f, 1.8f,
             -base_speed * 1.5f, base_speed * 1.5f);

    /* 功能2 变量清零 */
    s_f2_enabled = 0;
    s_f2_phase   = 0;
    s_f2_counter = 0;
}

/* ========== 端点启动：按端点类型配置直行/自转目标，进入 END_DRIVE ========== */
static void StartEndpoint(void)
{
    s_end_counter = 0;

    /* 路口六（STEP 10）端点巡线容易震荡，临时降低 Kp */
    if (s_mission_index == 10) s_turn_pid.Kp = 25.0f;

    /* 根据释放/抓取类型选择前进距离 */
    if (s_end_claw_action == 1)
        s_end_drive_dist = s_release_drive_mm;
    else if (s_end_claw_action == 2)
        s_end_drive_dist = s_grab_drive_mm;
    else
        s_advance_dist = s_release_drive_mm; /* 兜底 */

    s_state = STATE_END_DRIVE;
    s_locked = 1;   /* 端点全程不检测路口 */
    Encoder_ResetDist();

    /* 重置端点前进堵转检测基准 */
    {
        uint8_t i;
        for (i = 0; i < 4; i++)
            s_end_last_dist[i] = distance_mm[i];
        s_end_stall_count = 0;
    }
}

/* ========== 端点爪子动作查表 ==========
 * 返回: 0=无动作, 1=180°释放(P1=44→P2=90→P1=80), 2=抓取(P1=40→P2=0)
 * 同时设置 stop_ticks（400=4s，0=使用默认 300=3s） */
static uint8_t GetEndpointClawConfig(uint16_t step_index, uint32_t *stop_ticks)
{
    switch (step_index) {
        case 1:  /* STEP 2: 释放 */
        case 6:  /* STEP 7: 释放 */
        case 11: /* STEP 12: 释放 */
        case 19: /* STEP 20: 释放 */
        case 30: /* STEP 31: 释放 */
            *stop_ticks = 400;
            return 1;
        case 4:  /* STEP 5: 抓取 */
        case 9:  /* STEP 10: 抓取 */
        case 14: /* STEP 15: 抓取 */
        case 25: /* STEP 26: 抓取 */
            *stop_ticks = 150;   /* 1.5s：闭合1s + 转水平0.5s */
            return 2;
        default:
            *stop_ticks = 0;
            return 0;
    }
}

/* ========== 任务调度核心：消耗一个事件并设置后续状态 ========== */
static void DispatchEvent(MissionEvent ev)
{
    /* 配置端点爪子动作 */
    s_end_rotate_guard = 0;
    if (ev.end != M_END_NONE)
    {
        s_end_claw_action = GetEndpointClawConfig(s_mission_index, &s_end_stop_target);
        if (s_end_stop_target == 0)
            s_end_stop_target = END_STOP_TICKS;

        /* 路口2/3/4 的180°端点：放完东西后车头可能歪，自转前先不检测0.5s */
        if (ev.end == M_END_180)
        {
            switch (s_mission_index) {
                case 1:  /* STEP 2: 路口2 */
                case 6:  /* STEP 7: 路口3 */
                case 11: /* STEP 12: 路口4 */
                case 19: /* STEP 20: 路口2 */
                    s_end_rotate_guard = 50;  /* 0.5s 先纯转不检测，跳过旧来线 */
                    break;
                default:
                    break;
            }
        }
        /* 90°端点：自转前先不检测0.3s，跳过旧来线 */
        else if (ev.end == M_END_90)
        {
            s_end_rotate_guard = 10;  /* 0.1s */
        }
    }
    else
    {
        s_end_claw_action = 0;
        s_end_stop_target = END_STOP_TICKS;
    }

    /* 先记下端点类型，等转弯/PASS 完成后用 */
    s_pending_end = ev.end;

    if (ev.turn == M_PASS)
    {
        /* PASS：不转弯。若带端点直接进端点序列，否则盲走穿过路口 */
        if (s_pending_end != M_END_NONE)
        {
            StartEndpoint();
        }
        else
        {
            s_state = STATE_PASS_THROUGH;
            s_pass_counter = 0;
            s_locked = 1;
        }
    }
    else
    {
        /* 左拐/右拐/找线拐弯 → 都先 ADVANCE → BRAKE → 各自的 ROTATE */
        if (ev.turn == M_LEFT)
            s_turn_dir = DIR_CCW;
        else if (ev.turn == M_RIGHT)
            s_turn_dir = DIR_CW;
        else /* M_FINDLINE：前进后顺时针找线 */
            s_turn_dir = DIR_CW;

        /* 用一个标志区分"固定90°"和"找线"：找线时 s_pending_turn 记 M_FINDLINE */
        s_pending_turn = ev.turn;
        s_state = STATE_TURN_ADVANCE;
        s_advance_dist     = s_advance_mm[s_mission_index]; /* 从路口专属数组取值 */
        s_advance_counter = 0;     /* 清零前进计时 */

        s_locked = 1;
        Encoder_ResetDist();       /* 编码器测距起点 */
    }
}

/* 是否允许路口检测（冷却结束 + 边沿消隐复位完成） */
static uint8_t JunctionDetectionAllowed(void)
{
    static uint8_t s_rearm_stuck = 0;  /* rearm 超时计数 */

    if (s_cooldown > 0U)
        return 0U;

    if (!s_edge_rearm)
        return 1U;

    /* 触发后须等左右边沿均不亮，并持续 EDGE_REARM_TICKS 才重新武装 */
    if (!LeftEdgeIsClearBright() && !RightEdgeIsClearBright())
    {
        if (s_edge_off_count < 255U)
            s_edge_off_count++;
    }
    else
    {
        s_edge_off_count = 0U;
    }

    if (s_edge_off_count >= EDGE_REARM_TICKS)
    {
        s_edge_rearm = 0U;
        s_edge_off_count = 0U;
        s_rearm_stuck = 0U;
        return 1U;
    }

    /* rearm 超时保护：若一直检测到白线无法清除，强制解锁（30 tick = 300ms） */
    if (s_edge_off_count == 0U)
    {
        s_rearm_stuck++;
        if (s_rearm_stuck > 30U)
        {
            s_edge_rearm = 0U;
            s_rearm_stuck = 0U;
            return 1U;
        }
    }
    else
    {
        s_rearm_stuck = 0U;
    }

    return 0U;
}

/* ========== 状态机：FOLLOW（PID跟线 + 路口检测） ========== */
static void StateFollow(float dt)
{
    /* 锁住时不读传感器，按固定模式直行 */
    if (s_locked)
    {
        SetMotors((int)s_base_speed, (int)s_base_speed);
        return;
    }

    /* 任务已经全部执行完 → 永久停下（功能2 模式下不判定） */
    if (!s_f2_enabled && s_mission_index >= s_mission_length)
    {
        SetMotors(0, 0);
        return;
    }

    /* 起步保护：上电后 0.5s 内不检测路口，只做 PID 巡线 */
    if (s_startup_guard > 0U)
    {
        s_startup_guard--;
        DriveWithPID(dt);
        return;
    }

    if (JunctionDetectionAllowed())
    {
        /* 双边都做去抖检测
         * 第一个路口(s_arena_side==0)：只用 x0 或 x7 单传感器 */
        if (s_arena_side == 0)
        {
            if (ReadInverted(0) > s_hit_threshold[0])
                { if (s_left_bright_count < 255) s_left_bright_count++; }
            else
                s_left_bright_count = 0;

            if (ReadInverted(7) > s_hit_threshold[7])
                { if (s_right_bright_count < 255) s_right_bright_count++; }
            else
                s_right_bright_count = 0;
        }
        else
        {
            if (LeftEdgeIsClearBright())
                { if (s_left_bright_count < 255) s_left_bright_count++; }
            else
                s_left_bright_count = 0;

            if (RightEdgeIsClearBright())
                { if (s_right_bright_count < 255) s_right_bright_count++; }
            else
                s_right_bright_count = 0;
        }

        /* 任一边沿持续亮 N 帧 → 触发路口检测，查任务表 */
        if (s_left_bright_count >= EDGE_DEBOUNCE_TICKS ||
            s_right_bright_count >= EDGE_DEBOUNCE_TICKS)
        {
            /*
             * 第一个路口检测：根据传感器触发侧确定场地
             *   右侧传感器(x7)变亮 → 右转路口 → 切换到左侧场地任务表
             *   左侧传感器(x0)变亮 → 左转路口 → 保持右侧场地任务表（已默认）
             */
            if (s_arena_side == 0)
            {
                if (s_right_bright_count >= EDGE_DEBOUNCE_TICKS)
                {
                    ApplyLeftSideConfig();
                    s_arena_side = 1;
                }
                else
                {
                    s_arena_side = 2;
                }
            }

            s_left_bright_count = 0;
            s_right_bright_count = 0;
            s_edge_rearm = 1U;
            s_edge_off_count = 0U;

            if (s_f2_enabled)
            {
                if (s_f2_phase == 2 && !s_f2_bypass_done)
                {
                    s_f2_bypass_done = 1;
                    s_state = STATE_F2_BYPASS;
                    s_f2_counter = 0;
                    s_locked = 1;
                    return;
                }
                Feature2HandleJunction();
                return;
            }

            PrintStep(s_mission_index + 1);
            DispatchEvent(s_active_mission[s_mission_index]);
            s_mission_index++;
            return;
        }

        /* 任一边沿开始变亮 → 临时纯直行；否则 PID 巡线 */
        if ((s_left_bright_count > 0 || s_right_bright_count > 0) && !s_f2_enabled)
        {
            SetMotors((int)s_base_speed, (int)s_base_speed);
        }
        else
        {
            DriveWithPID(dt);
        }
    }
    else
    {
        s_left_bright_count = 0;
        s_right_bright_count = 0;
        DriveWithPID(dt);
    }
}

/* ========== 状态机：PASS_THROUGH（盲走穿过路口） ==========
 * 任务表中标记为 PASS 的路口在此处理：
 *   不读传感器、不调 PID，仅以 base_speed 直行 PASS_THROUGH_TICKS 个周期
 *   走完后再回 FOLLOW 启动 PID 跟线，避免被横线分支拉偏
 */
static void StatePassThrough(void)
{
    s_pass_counter++;
    if (s_pass_counter >= PASS_THROUGH_TICKS)
    {
        s_state = STATE_FOLLOW;
        s_cooldown = COOLDOWN_AFTER_PASS;
        s_locked = 0;
        PID_Reset(&s_turn_pid);
        s_last_position = 0.0f;
        s_pid_first_call = 1;

        /* 路口7 PASS 后强制巡线 0.5s 不检测路口，稳定车姿再进入下一个转弯 */
        if (s_mission_index == 25)
            s_startup_guard = 50U;

        return;
    }
    SetMotors((int)s_base_speed, (int)s_base_speed);
}

/* ========== 状态机：TURN_ADVANCE（纯直行前进，编码器测距） ==========
 * 检测到路口后，以 base_speed 纯直行前进 s_advance_dist mm，再刹车转弯。
 */
static void StateTurnAdvance(float dt)
{
    (void)dt;

    /* 调试：每 10 次打印当前距离 */
    {
        static uint32_t dbg_cnt = 0;
        if (++dbg_cnt % 10 == 0) {
            char buf[32];
            sprintf(buf, "ADV:%.0f/%.0f\r\n", (double)Encoder_GetAvgDist(), (double)s_advance_dist);
            USART2_SendString(buf);
        }
    }

    if (Encoder_GetAvgDist() >= s_advance_dist)
    {
        s_state = STATE_TURN_BRAKE;
        s_brake_counter = 0;
        return;
    }
    /* 纯直行（不跟线），左右轮同速 */
    SetMotors((int)s_base_speed, (int)s_base_speed);
}

/* ========== 状态机：TURN_BRAKE（松电机滑停，不反向） ==========
 * base_speed 很低、惯性小，不做主动反向刹车（会导致后退），
 * 仅断电靠摩擦滑停 BRAKE_TICKS_LONG 个 tick，再分流到转弯
 * 刹停后按转向类型分流：找线拐弯→FINDLINE_ROTATE，固定拐弯→TURN_ROTATE
 */
static void StateTurnBrake(void)
{
    s_brake_counter++;

    /* 全程松开电机，靠摩擦自然停下，不再反向 */
    SetMotors(0, 0);

    if (s_brake_counter >= BRAKE_TICKS_LONG)
    {
        if (s_pending_turn == M_FINDLINE)
        {
            s_state = STATE_FINDLINE_ROTATE;
            s_findline_counter = 0;
        }
        else
        {
            s_state = STATE_TURN_ROTATE;
            s_rotate_counter = 0;
        }
    }
}

/* ========== 状态机：TURN_ROTATE（路口拐弯：按方向自转，检测到中间压白线停止） ==========
 * 检测到白线后先反向制动 0.05s，再根据 s_pending_end 决定去 FOLLOW 还是端点序列
 */
static void StateTurnRotate(void)
{
    /* 防御性检查：方向必须有效，否则直接退回 FOLLOW 避免无效停留 */
    if (s_turn_dir != DIR_CCW && s_turn_dir != DIR_CW)
    {
        s_state = STATE_FOLLOW;
        s_cooldown = COOLDOWN_AFTER_TURN;
        s_locked = 0;
        s_turn_dir = DIR_NONE;
        s_pid_first_call = 1;
        return;
    }

    /* 检测到白线后先反向制动 5 tick（0.05s），抵消惯性 */
    if (s_rotate_bounce > 0)
    {
        s_rotate_bounce--;
        /* 反向转：原来 CW 就 CCW，原来 CCW 就 CW */
        TurnDir bounce = (s_turn_dir == DIR_CW) ? DIR_CCW : DIR_CW;
        SetRotateMotors(bounce);
        if (s_rotate_bounce == 0)
        {
            goto turn_rotate_done;
        }
        return;
    }

    s_rotate_counter++;

    /* 转过最小 tick 后才开始检测，避免一起转就压到旧线 */
    if (s_rotate_counter >= FINDLINE_MIN_TICKS && MiddleOnLine())
    {
        s_rotate_bounce = 5;  /* 启动反向制动 0.05s */
        return;
    }
    /* 超时保护 */
    if (s_rotate_counter >= FINDLINE_MAX_TICKS)
    {
turn_rotate_done:
        s_rotate_bounce = 0;
        PID_Reset(&s_turn_pid);
        s_last_position = 0.0f;
        s_pid_first_call = 1;

        /* 功能2 转弯完成 → 进入高速直行 */
        if (s_f2_enabled)
        {
            Feature2AfterTurn();
            return;
        }

        if (s_pending_end != M_END_NONE)
        {
            StartEndpoint();
        }
        else
        {
            s_state = STATE_FOLLOW;
            s_cooldown = COOLDOWN_AFTER_TURN;
            s_locked = 0;
            s_turn_dir = DIR_NONE;
        }
        return;
    }
    SetRotateMotors(s_turn_dir);
}

/* ========== 状态机：FINDLINE_ROTATE（找线拐弯：CW自转到中间两路压线） ==========
 * 前进并刹停后进入此状态：顺时针自转，转过最小角度后开始检测，
 * 中间两路(第4、5路)任一压到白线就停，然后反向制动；超时保护防止空转
 */
static void StateFindLineRotate(void)
{
    /* 制动期：反向转 5 tick 抵消惯性 */
    if (s_rotate_bounce > 0)
    {
        s_rotate_bounce--;
        SetRotateMotors(DIR_CCW);  /* CW转动后反向用 CCW */
        if (s_rotate_bounce == 0)
        {
            goto findline_rotate_done;
        }
        return;
    }

    s_findline_counter++;

    /* 转过最小 tick 后才开始检测，避免一起转就压到来时的线 */
    if (s_findline_counter >= FINDLINE_MIN_TICKS && MiddleOnLine())
    {
        s_rotate_bounce = 5;
        return;
    }
    /* 超时保护：转太久还没找到线，强制回 FOLLOW（避免永久空转） */
    if (s_findline_counter >= FINDLINE_MAX_TICKS)
    {
findline_rotate_done:
        s_rotate_bounce = 0;
        s_state = STATE_FOLLOW;
        s_cooldown = COOLDOWN_AFTER_TURN;
        s_locked = 0;
        s_turn_dir = DIR_NONE;
        PID_Reset(&s_turn_pid);
        s_last_position = 0.0f;
        s_pid_first_call = 1;
        return;
    }
    SetRotateMotors(DIR_CW);   /* 找线拐弯固定顺时针 */
}

/* ========== 状态机：END_DRIVE（端点：巡线直行） ========== */
static void StateEndDrive(float dt)
{
    s_end_counter++;
    if (s_end_counter >= s_end_drive_target)
    {
        s_state = STATE_END_STOP;
        s_end_counter = 0;
        return;
    }
    DriveWithPID(dt);
}

/* ========== 状态机：END_STOP（端点：停 N 秒，按端点配置） ========== */
static void StateEndStop(void)
{
    SetMotors(0, 0);

    /* 爪子动作 */
    if (s_end_claw_action != 0)
    {
        if (s_end_counter == 0)
        {
            if (s_end_claw_action == 1)  /* 180°释放(3步)：先张到44 */
            {
                g_claw_pwm1_angle = 44;
                Claw_ApplyAngles();
            }
            else if (s_end_claw_action == 2)  /* 抓取：先闭合 */
            {
                g_claw_pwm1_angle = 40;
                Claw_ApplyAngles();
            }
        }
        /* 释放(1)第2步在1s，抓取(2)第2步在1s */
        else if (s_end_counter == 100 && s_end_claw_action == 1)
        {
            g_claw_pwm2_angle = 90;
            Claw_ApplyAngles();
        }
        else if (s_end_counter == 100 && s_end_claw_action == 2)
        {
            g_claw_pwm2_angle = 0;
            Claw_ApplyAngles();
        }
        else if (s_end_counter == 200 && s_end_claw_action == 1)  /* 2s：释放最后松到80 */
        {
            g_claw_pwm1_angle = 80;
            Claw_ApplyAngles();
        }
    }

    s_end_counter++;
    if (s_end_counter >= s_end_stop_target)
    {
        if (s_pending_end == M_END_FINAL)
        {
            s_state = STATE_FINAL_STOP;
        }
        else if (s_pending_end == M_END_90 || s_pending_end == M_END_180)
        {
            s_state = STATE_END_STRAIGHT;
            s_end_counter = 0;
        }
        else
        {
            s_state = STATE_END_REVERSE;
            s_end_counter = 0;
            Encoder_ResetDist();
            ReverseBalanceReset();
        }
    }
}

/* ========== 盲退平衡：编码器瞬时速度差做 P 修正，保持直线后退 ========== */
static float s_rb_prev_left  = 0.0f;
static float s_rb_prev_right = 0.0f;
static uint8_t s_rb_first    = 1;

static void ReverseBalanceReset(void)
{
    s_rb_first = 1;
}

static void ReverseWithBalance(void)
{
    float left, right;
    Encoder_GetSideDist(&left, &right);

    if (s_rb_first)
    {
        s_rb_prev_left  = left;
        s_rb_prev_right = right;
        s_rb_first = 0;
    }

    /* 本帧行程增量（瞬时速度）*/
    float delta_left  = left  - s_rb_prev_left;
    float delta_right = right - s_rb_prev_right;
    s_rb_prev_left  = left;
    s_rb_prev_right = right;

    /* 左侧轮(M1/M4)后退时编码器取反导致delta为负，右侧为正。
       取绝对值后比较，左侧×1.2人为压低PWM。P=15，限幅±40 */
    float abs_left  = (delta_left  < 0.0f) ? -delta_left  : delta_left;
    float abs_right = (delta_right < 0.0f) ? -delta_right : delta_right;
    float diff = abs_left * 1.2f - abs_right;
    int corr = (int)(diff * 15.0f);
    if (corr >  40) corr =  40;
    if (corr < -40) corr = -40;

    int spd = -(int)s_base_speed;
    SetMotors(spd + corr, spd - corr);  /* 左侧减速，右侧加速 */
}

/* ========== 状态机：END_REVERSE（端点：盲退，编码器左右平衡） ========== */
static void StateEndReverse(void)
{
    {
        float dist = Encoder_GetAvgDist();
        if (dist < 0.0f) dist = -dist;

        /* 根据释放/抓取类型选择后退距离 */
        float target = (s_end_claw_action == 2) ? s_grab_reverse_mm : s_release_reverse_mm;
        /* 路口8（STEP 26）单独使用 420mm 后退距离 */
        if (s_mission_index == 26) target = 420.0f;
        if (dist >= target)
        {
            s_state = STATE_END_ROTATE;
            s_end_counter = 0;
            return;
        }
    }

    ReverseWithBalance();
}

/* ========== 状态机：END_STRAIGHT（端点 90°：纯直行 1s，速度50） ========== */
static void StateEndStraight(void)
{
    SetMotors(50, 50);
    s_end_counter++;
    if (s_end_counter >= 100)   /* 1s */
    {
        s_state = STATE_END_REVERSE;
        s_end_counter = 0;
        Encoder_ResetDist();
        ReverseBalanceReset();
    }
}

/* ========== 状态机：END_ROTATE（端点自转：按场地侧方向自转，检测到中间压白线停止） ==========
 * 检测到白线后反向制动 0.05s，转完直接回 FOLLOW 开始正常巡线
 */
static void StateEndRotate(void)
{
    /* 制动期 */
    if (s_rotate_bounce > 0)
    {
        s_rotate_bounce--;
        TurnDir bounce = (s_end_rotate_dir == DIR_CW) ? DIR_CCW : DIR_CW;
        SetRotateMotors(bounce);
        if (s_rotate_bounce == 0)
        {
            goto end_rotate_done;
        }
        return;
    }

    s_end_counter++;

    /* 先走完护盾期（路口2/3/4 的180°端点需要50tick=0.5s），
       纯转不检测白线，避免误读到刚进来的旧线 */
    if (s_end_rotate_guard > 0)
    {
        s_end_rotate_guard--;
        SetRotateMotors(s_end_rotate_dir);
        return;
    }

    if (s_end_counter >= FINDLINE_MIN_TICKS && MiddleOnLine())
    {
        s_rotate_bounce = 5;
        return;
    }
    if (s_end_counter >= FINDLINE_MAX_TICKS)
    {
end_rotate_done:
        s_rotate_bounce = 0;

        /* 最后一步(STEP 31)：自转完成后闭合夹爪，进入功能二 */
        if (s_mission_index >= 31)
        {
            g_claw_pwm1_angle = 40;
            g_claw_pwm2_angle = 90;
            Claw_ApplyAngles();
            StartFeature2();
            return;
        }

        s_state = STATE_FOLLOW;
        s_cooldown = COOLDOWN_AFTER_ENDPOINT;
        s_startup_guard = 50U;   /* 端点后起步保护 0.5s，避免自转后压白线误触发路口 */
        s_locked = 0;
        s_turn_pid.Kp = 22.0f;   /* 恢复正常 Kp */
        s_pending_end = M_END_NONE;
        /* 释放端点结束后恢复爪子P2到-90（与放置时75岔开180°）；抓取端点不恢复，保持-10 */
         if (s_end_claw_action == 1)
         {
             g_claw_pwm2_angle = -90;
            Claw_ApplyAngles();
        }
        PID_Reset(&s_turn_pid);
        s_last_position = 0.0f;
        s_turn_dir = DIR_NONE;
        s_pid_first_call = 1;

        return;
    }
    SetRotateMotors(s_end_rotate_dir);
}

/* ========== 状态机：FINAL_STOP（任务完成，永久停下） ========== */
static void StateFinalStop(void)
{
    SetMotors(0, 0);
}

/* ==================================================================
 * 功能2 状态机（左/右侧场地镜像对称）
 * 后退和自转已在功能一 STEP 31 端点末尾完成，功能二从 30s 等待直接进入巡线。
 *
 * 右侧场地流程:
 * STEP 31 90°端点停完 → 后退 → 自转回线 → 闭合夹爪 → 功能2
 *   Phase 0: 等 30s（爪闭合静止）
 *   Phase A: 巡线 → 路口4右拐(CW) → 直行2s → 后退 → CW自转90°
 *   Phase B: 巡线 → PASS路口3 → 路口2左拐(CCW) → 直行2s → 后退 → CCW自转90°
 *   Phase C: 巡线 → 路口3右拐(CW) → 直行2s → 永久停
 *
 * 左侧场地流程（镜像）:
 * STEP 31 90°端点停完 → 后退 → 自转回线 → 闭合夹爪 → 功能2
 *   Phase 0: 等 30s（爪闭合静止）
 *   Phase A: 巡线 → 路口4左拐(CCW) → 直行2s → 后退 → CCW自转90°
 *   Phase B: 巡线 → PASS路口3 → 路口2右拐(CW) → 直行2s → 后退 → CW自转90°
 *   Phase C: 巡线 → 路口3左拐(CCW) → 直行2s → 永久停
 * ================================================================== */

/* 启动功能2：功能一最后一步自转完毕后调用（夹爪已闭合）
 * 进入 30s 等待，之后直接巡线 */
static void StartFeature2(void)
{
    s_f2_enabled     = 1;
    s_f2_phase       = 0;
    s_f2_counter     = 0;
    s_rotate_bounce  = 0;
    s_state          = STATE_F2_WAIT_30S;
    s_locked         = 1;
}

/* -------- STATE_F2_WAIT_30S: 静等 30s -------- */
static void StateF2Wait30s(void)
{
    SetMotors(0, 0);
    s_f2_counter++;
    if (s_f2_counter >= 3000U)   /* 30s = 3000 tick */
    {
        /* 起步自转已完成，直接进入巡线 */
        PID_Reset(&s_turn_pid);
        s_last_position  = 0.0f;
        s_pid_first_call = 1;
        s_f2_phase       = 1;
        s_state          = STATE_FOLLOW;
        s_cooldown       = COOLDOWN_AFTER_ENDPOINT;
        s_locked         = 0;
        s_mission_index  = 0;
        s_f2_bypass_done = 0;
    }
}

/* 功能2 堵转检测：检查是否有至少2个轮子转速几乎为0
 * 连续 STALL_CONFIRM_TICKS 帧确认才触发，避免颠簸误判 */
#define STALL_DELTA_MM      0.5f   /* 单帧(10ms)行程低于此值视为堵转 */
#define STALL_MOTOR_MIN     2      /* 至少几个轮子堵转才触发 */
#define STALL_CONFIRM_TICKS 5      /* 连续确认帧数 */

static uint8_t F2CheckStall(void)
{
    uint8_t stalled = 0;
    uint8_t i;

    for (i = 0; i < 4; i++) {
        float delta = distance_mm[i] - s_f2_last_dist[i];
        if (delta < 0.0f) delta = -delta;
        if (delta < STALL_DELTA_MM)
            stalled++;
        s_f2_last_dist[i] = distance_mm[i];
    }

    if (stalled >= STALL_MOTOR_MIN)
    {
        if (++s_f2_stall_count >= STALL_CONFIRM_TICKS)
        {
            s_f2_stall_count = 0;
            return 1;  /* 堵转确认 */
        }
    }
    else
    {
        s_f2_stall_count = 0;
    }
    return 0;
}

/* 进入前进程序前重置堵转检测基准 */
static void F2StallReset(void)
{
    uint8_t i;
    for (i = 0; i < 4; i++)
        s_f2_last_dist[i] = distance_mm[i];
    s_f2_stall_count = 0;
}

/* 端点前进堵转检测：仅对释放端点生效 */
static uint8_t EndCheckStall(void)
{
    uint8_t stalled = 0;
    uint8_t i;

    for (i = 0; i < 4; i++) {
        float delta = distance_mm[i] - s_end_last_dist[i];
        if (delta < 0.0f) delta = -delta;
        if (delta < STALL_DELTA_MM)
            stalled++;
        s_end_last_dist[i] = distance_mm[i];
    }

    if (stalled >= STALL_MOTOR_MIN)
    {
        if (++s_end_stall_count >= STALL_CONFIRM_TICKS)
        {
            s_end_stall_count = 0;
            return 1;  /* 堵转确认 */
        }
    }
    else
    {
        s_end_stall_count = 0;
    }
    return 0;
}

/* -------- STATE_F2_ADVANCE: 功能2 转弯前巡线前进（编码器测距）-------- */
static void StateF2Advance(float dt)
{
    /* 堵转检测 */
    if (F2CheckStall())
    {
        s_state = STATE_F2_REVERSE;
        Encoder_ResetDist();
        ReverseBalanceReset();
        return;
    }

    if (Encoder_GetAvgDist() >= s_advance_dist)
    {
        s_state      = STATE_TURN_BRAKE;
        s_f2_counter = 0;
        return;
    }
    DriveWithPID(dt);
}

/* -------- STATE_F2_SPRINT: 功能2 高速直行（巡线PID，编码器测距）-------- */
static void StateF2Sprint(float dt)
{
    /* 堵转检测 */
    if (F2CheckStall())
    {
        s_state = STATE_F2_REVERSE;
        Encoder_ResetDist();
        ReverseBalanceReset();
        return;
    }

    s_f2_counter++;
    if (s_f2_counter >= 300)   /* 固定 3s */
    {
        s_f2_counter = 0;
        if (s_f2_phase == 3)
        {
            /* Phase C 直行完 → 永久停 */
            s_state = STATE_FINAL_STOP;
        }
        else
        {
            s_state = STATE_F2_REVERSE;
            Encoder_ResetDist();
            ReverseBalanceReset();
        }
        return;
    }
    DriveWithPID(dt);
}

/* -------- STATE_F2_REVERSE: 功能2 盲退（编码器左右平衡）-------- */
static void StateF2Reverse(void)
{
    {
        float dist = Encoder_GetAvgDist();
        if (dist < 0.0f) dist = -dist;
        if (dist >= F2_REVERSE_MM)
        {
            s_f2_counter = 0;
            s_state      = STATE_F2_ROTATE;
            return;
        }
    }

    ReverseWithBalance();
}

/* -------- STATE_F2_ROTATE: 功能2 自转（按 s_f2_rotate_dir）-------- */
static void StateF2Rotate(void)
{
    /* 制动期 */
    if (s_rotate_bounce > 0)
    {
        s_rotate_bounce--;
        TurnDir bounce = (s_f2_rotate_dir == DIR_CW) ? DIR_CCW : DIR_CW;
        SetRotateMotors(bounce);
        if (s_rotate_bounce == 0)
        {
            goto f2_rotate_done;
        }
        return;
    }

    /* 保护期：先纯转0.1s不检测白线，跳过路口旧来线 */
    if (s_f2_counter < 10)
    {
        s_f2_counter++;
        SetRotateMotors(s_f2_rotate_dir);
        return;
    }

    s_f2_counter++;
    if (s_f2_counter >= FINDLINE_MIN_TICKS && MiddleOnLine())
    {
        s_rotate_bounce = 5;
        return;
    }
    if (s_f2_counter >= FINDLINE_MAX_TICKS)
    {
f2_rotate_done:
        s_rotate_bounce = 0;
        PID_Reset(&s_turn_pid);
        s_last_position  = 0.0f;
        s_pid_first_call = 1;

        if (s_f2_phase == 3)
        {
            s_state = STATE_FINAL_STOP;
        }
        else
        {
            /* 自转结束 → 进入下一 Phase 巡线 */
            s_f2_phase++;
            s_state          = STATE_FOLLOW;
            s_cooldown       = COOLDOWN_AFTER_ENDPOINT;
            s_locked         = 0;
            s_mission_index  = 0;
            s_f2_bypass_done = 0;
        }
        return;
    }
    SetRotateMotors(s_f2_rotate_dir);
}

/* -------- STATE_F2_BYPASS: 功能2 盲走穿过路口 -------- */
static void StateF2Bypass(void)
{
    s_f2_counter++;
    if (s_f2_counter >= PASS_THROUGH_TICKS)
    {
        s_f2_counter = 0;
        s_state      = STATE_FOLLOW;
        s_cooldown   = COOLDOWN_AFTER_PASS;
        s_locked     = 0;
        PID_Reset(&s_turn_pid);
        s_last_position  = 0.0f;
        s_pid_first_call = 1;
        return;
    }
    SetMotors((int)s_f2_base_speed, (int)s_f2_base_speed);
}

/* 功能2 路口检测处理
 * 右侧场地: Phase A→右拐CW, Phase B→左拐CCW, Phase C→右拐CW
 * 左侧场地: Phase A→左拐CCW, Phase B→右拐CW, Phase C→左拐CCW */
static void Feature2HandleJunction(void)
{
    s_locked     = 1;
    s_f2_counter = 0;

    switch (s_f2_phase)
    {
        case 1: /* Phase A: 路口4 → 拐弯 → 直行 */
            if (s_arena_side == 2)
            {
                s_turn_dir         = DIR_CW;
                s_pending_turn     = M_RIGHT;
                s_f2_rotate_dir    = DIR_CW;
            }
            else /* 左侧场地 */
            {
                s_turn_dir         = DIR_CCW;
                s_pending_turn     = M_LEFT;
                s_f2_rotate_dir    = DIR_CCW;
            }
            s_state          = STATE_F2_ADVANCE;
            s_advance_dist   = s_f2_advance_mm[0];
            s_f2_sprint_dist = F2_SPRINT_MM;
            Encoder_ResetDist();
            F2StallReset();
            break;

        case 2: /* Phase B: PASS路口3 → 路口2拐弯 → 直行 */
            if (s_arena_side == 2)
            {
                s_turn_dir         = DIR_CCW;
                s_pending_turn     = M_LEFT;
                s_f2_rotate_dir    = DIR_CCW;
            }
            else /* 左侧场地 */
            {
                s_turn_dir         = DIR_CW;
                s_pending_turn     = M_RIGHT;
                s_f2_rotate_dir    = DIR_CW;
            }
            s_state          = STATE_F2_ADVANCE;
            s_advance_dist   = s_f2_advance_mm[1];
            s_f2_sprint_dist = F2_SPRINT_MM;
            Encoder_ResetDist();
            F2StallReset();
            break;

        case 3: /* Phase C: 路口3 → 拐弯 → 直行 → 永久停 */
            if (s_arena_side == 2)
            {
                s_turn_dir         = DIR_CW;
                s_pending_turn     = M_RIGHT;
            }
            else /* 左侧场地 */
            {
                s_turn_dir         = DIR_CCW;
                s_pending_turn     = M_LEFT;
            }
            s_state          = STATE_F2_ADVANCE;
            s_advance_dist   = s_f2_advance_mm[2];
            s_f2_sprint_dist = F2_SPRINT_MM;
            Encoder_ResetDist();
            F2StallReset();
            break;

        default:
            break;
    }
}

/* 功能2 转弯完成后 → 进入高速直行 */
static void Feature2AfterTurn(void)
{
    s_state      = STATE_F2_SPRINT;
    s_f2_counter = 0;
    s_locked     = 1;
    s_turn_dir   = DIR_NONE;
    Encoder_ResetDist();
    F2StallReset();
}

/* ========== 主控制函数 ========== */
void LineTrack_Update(float dt)
{
    static TrackState s_prev_state = STATE_FOLLOW;

    /* 编码器采集任务：消费 ISR 中断中的 raw_count */
    Encoder_Task();

    /* 简单模式：走独立的步骤状态机 */
    if (s_sm_enabled)
    {
        SimpleMissionUpdate(dt);
        return;
    }

    /* 冷却计时（仅 FOLLOW 状态用） */
    if (s_cooldown > 0) s_cooldown--;

    /* 刚切回 FOLLOW 时重置 PID，避免 D 项尖峰引发震荡 */
    if (s_state == STATE_FOLLOW && s_prev_state != STATE_FOLLOW)
    {
        s_pid_first_call = 1;
    }
    s_prev_state = s_state;

    switch (s_state)
    {
        case STATE_FOLLOW:          StateFollow(dt);       break;
        case STATE_PASS_THROUGH:    StatePassThrough();    break;
        case STATE_TURN_ADVANCE:    StateTurnAdvance(dt);  break;
        case STATE_TURN_BRAKE:      StateTurnBrake();      break;
        case STATE_TURN_ROTATE:     StateTurnRotate();     break;
        case STATE_FINDLINE_ROTATE: StateFindLineRotate(); break;
        case STATE_END_DRIVE:       StateEndDrive(dt);     break;
        case STATE_END_STOP:        StateEndStop();        break;
        case STATE_END_REVERSE:     StateEndReverse();     break;
        case STATE_END_STRAIGHT:    StateEndStraight();    break;
        case STATE_END_ROTATE:      StateEndRotate();      break;
        case STATE_FINAL_STOP:      StateFinalStop();      break;
        /* 功能2 */
        case STATE_F2_WAIT_30S:     StateF2Wait30s();      break;
        case STATE_F2_ADVANCE:      StateF2Advance(dt);    break;
        case STATE_F2_SPRINT:       StateF2Sprint(dt);      break;
        case STATE_F2_REVERSE:      StateF2Reverse();      break;
        case STATE_F2_ROTATE:       StateF2Rotate();       break;
        case STATE_F2_BYPASS:       StateF2Bypass();       break;
        default:                    s_state = STATE_FOLLOW; break;
    }
}

/* ==================================================================
 * 简单巡线模式（右侧场地，无路口判定）
 *
 * 流程概述：
 *   初始化夹爪 → 巡线800mm → CW自转 → 前进3s → 张爪 → 前进1s(反转) →
 *   后退2850mm → 硬转90°CW → 直行到白线 → 巡线3s → 合爪+后退+CCW →
 *   巡线1800 → CCW → 巡线3s → 释放+后退+CCW →
 *   巡线2400 → CCW → 巡线3s → 合爪+后退+CCW →
 *   巡线1700 → CCW → 巡线3s → 释放+后退+CCW →
 *   巡线1600 → CCW → 巡线3s → 合爪+后退+CCW →
 *   巡线3000 → CCW → 巡线3s → 释放+后退+CCW →
 *   巡线3600 → CCW → 巡线3s → 合爪+后退+CCW →
 *   巡线2900 → CCW → 巡线3s → 释放+后退+CCW → 永久停
 *
 * 每个 CLAW_GRAB 内部: stop(1.5s, P1=40@0, P2=0@1s) → reverse 450mm → CCW自转回线
 * 每个 CLAW_RELEASE 内部: stop(4s, P1=44@0, P2=90@1s, P1=80@2s) → reverse 450mm → CCW自转回线(P2=-90)
 * ================================================================== */

/* ── 简单巡线步骤类型 ──────────────────────────────── */
typedef enum {
    SM_FOLLOW_DIST,       /* PID巡线直行到距离 mm */
    SM_FOLLOW_TIME,       /* PID巡线直行到时间 ticks(10ms) */
    SM_STRAIGHT_TIME,     /* 纯直行(无PID)到时间 */
    SM_REVERSE_DIST,      /* 后退到距离 */
    SM_ROTATE_CW,         /* CW自转找白线 */
    SM_ROTATE_CCW,        /* CCW自转找白线 */
    SM_HARD_90_CW,        /* 硬转90°CW(编码器控制) */
    SM_CLAW_OPEN,         /* 张爪+启动反转180° */
    SM_CLAW_GRAB,         /* 抓取：stop+闭合+后退450+CCW自转 */
    SM_CLAW_RELEASE,      /* 释放：stop+释放+后退450+CCW自转 */
    SM_STRAIGHT_WHITE,    /* 纯直行到x2-x5全白 */
    SM_FINAL_STOP,        /* 永久停 */
} SimpleStepType;

typedef struct {
    SimpleStepType type;
    float   dist_mm;       /* 距离 mm */
    uint32_t time_ticks;   /* 时间 10ms/tick */
} SimpleStep;

/* ── 右侧场地简单巡线任务表 (39步) ─────────────────── */
static const SimpleStep sm_right_side[] = {
    /*  0 */ { SM_FOLLOW_DIST,    800.0f,  0 },
    /*  1 */ { SM_ROTATE_CW,      0,       0 },
    /*  2 */ { SM_STRAIGHT_TIME,  0,       300 },   /* 前进3s */
    /*  3 */ { SM_CLAW_OPEN,      0,       0 },
    /*  4 */ { SM_STRAIGHT_TIME,  0,       100 },   /* 前进1s(夹爪反转在此完成) */
    /*  5 */ { SM_REVERSE_DIST,   2850.0f, 0 },
    /*  6 */ { SM_HARD_90_CW,     0,       0 },
    /*  7 */ { SM_STRAIGHT_WHITE, 0,       0 },
    /*  8 */ { SM_FOLLOW_TIME,    0,       300 },   /* 巡线3s */
    /*  9 */ { SM_CLAW_GRAB,      0,       0 },      /* 合爪+后退450+CCW */
    /* 10 */ { SM_FOLLOW_DIST,    1800.0f, 0 },
    /* 11 */ { SM_ROTATE_CCW,     0,       0 },
    /* 12 */ { SM_FOLLOW_TIME,    0,       300 },   /* 巡线3s */
    /* 13 */ { SM_CLAW_RELEASE,   0,       0 },      /* 释放+后退450+CCW */
    /* 14 */ { SM_FOLLOW_DIST,    2400.0f, 0 },
    /* 15 */ { SM_ROTATE_CCW,     0,       0 },
    /* 16 */ { SM_FOLLOW_TIME,    0,       300 },   /* 巡线3s */
    /* 17 */ { SM_CLAW_GRAB,      0,       0 },
    /* 18 */ { SM_FOLLOW_DIST,    1700.0f, 0 },
    /* 19 */ { SM_ROTATE_CCW,     0,       0 },
    /* 20 */ { SM_FOLLOW_TIME,    0,       300 },   /* 巡线3s */
    /* 21 */ { SM_CLAW_RELEASE,   0,       0 },
    /* 22 */ { SM_FOLLOW_DIST,    1600.0f, 0 },
    /* 23 */ { SM_ROTATE_CCW,     0,       0 },
    /* 24 */ { SM_FOLLOW_TIME,    0,       300 },   /* 巡线3s */
    /* 25 */ { SM_CLAW_GRAB,      0,       0 },
    /* 26 */ { SM_FOLLOW_DIST,    3000.0f, 0 },
    /* 27 */ { SM_ROTATE_CCW,     0,       0 },
    /* 28 */ { SM_FOLLOW_TIME,    0,       300 },   /* 巡线3s */
    /* 29 */ { SM_CLAW_RELEASE,   0,       0 },
    /* 30 */ { SM_FOLLOW_DIST,    3600.0f, 0 },
    /* 31 */ { SM_ROTATE_CCW,     0,       0 },
    /* 32 */ { SM_FOLLOW_TIME,    0,       300 },   /* 巡线3s */
    /* 33 */ { SM_CLAW_GRAB,      0,       0 },
    /* 34 */ { SM_FOLLOW_DIST,    2900.0f, 0 },
    /* 35 */ { SM_ROTATE_CCW,     0,       0 },
    /* 36 */ { SM_FOLLOW_TIME,    0,       300 },   /* 巡线3s */
    /* 37 */ { SM_CLAW_RELEASE,   0,       0 },
    /* 38 */ { SM_FINAL_STOP,     0,       0 },
};

#define SM_STEP_COUNT (sizeof(sm_right_side) / sizeof(sm_right_side[0]))

/* ── 简单巡线状态变量 ──────────────────────────────── */
static uint8_t  s_sm_enabled  = 0;       /* 简单模式开关 */
static uint16_t s_sm_index    = 0;       /* 当前步骤索引 */
static uint32_t s_sm_counter  = 0;       /* 通用计时器 */
static float    s_sm_dist_target = 0;    /* 距离目标 */
static TurnDir  s_sm_rotate_dir = DIR_NONE; /* 自转方向 */
static uint8_t  s_sm_claw_action = 0;    /* 0=无, 1=释放, 2=抓取 */
static uint32_t s_sm_claw_stop_target = 0; /* 夹爪停止tick目标 */
static float    s_sm_hard90_start[4] = {0}; /* 硬转90°起始编码器值 */

/* ── 检查x2,x3,x4,x5是否全检测到白线 ──────────────── */
static uint8_t AllMidOnWhite(void)
{
    if (ReadInverted(2) > s_hit_threshold[2] &&
        ReadInverted(3) > s_hit_threshold[3] &&
        ReadInverted(4) > s_hit_threshold[4] &&
        ReadInverted(5) > s_hit_threshold[5])
        return 1;
    return 0;
}

/* ── 执行当前步骤 ─────────────────────────────────── */
static void SimpleStepStart(void)
{
    if (s_sm_index >= SM_STEP_COUNT)
    {
        s_state = STATE_FINAL_STOP;
        return;
    }

    const SimpleStep *step = &sm_right_side[s_sm_index];
    s_sm_counter = 0;
    s_sm_rotate_dir = DIR_NONE;
    s_sm_claw_action = 0;
    s_locked = 1;

    /* 串口打印当前步骤 */
    {
        char buf[32];
        sprintf(buf, "SM STEP %u/%u\r\n", (unsigned)(s_sm_index + 1), (unsigned)SM_STEP_COUNT);
        USART2_SendString(buf);
    }

    switch (step->type)
    {
        case SM_FOLLOW_DIST:
            s_state = STATE_SM_FOLLOW;
            s_sm_dist_target = step->dist_mm;
            Encoder_ResetDist();
            PID_Reset(&s_turn_pid);
            s_last_position = 0.0f;
            s_pid_first_call = 1;
            break;

        case SM_FOLLOW_TIME:
            s_state = STATE_SM_FOLLOW;
            s_sm_dist_target = 0;  /* 无距离限制 */
            s_sm_counter = 0;
            PID_Reset(&s_turn_pid);
            s_last_position = 0.0f;
            s_pid_first_call = 1;
            break;

        case SM_STRAIGHT_TIME:
            s_state = STATE_SM_STRAIGHT;
            s_sm_counter = 0;
            break;

        case SM_REVERSE_DIST:
            s_state = STATE_SM_REVERSE;
            s_sm_dist_target = step->dist_mm;
            Encoder_ResetDist();
            ReverseBalanceReset();
            break;

        case SM_ROTATE_CW:
            s_state = STATE_SM_ROTATE_FINDLINE;
            s_sm_rotate_dir = DIR_CW;
            s_sm_counter = 0;
            s_rotate_bounce = 0;
            break;

        case SM_ROTATE_CCW:
            s_state = STATE_SM_ROTATE_FINDLINE;
            s_sm_rotate_dir = DIR_CCW;
            s_sm_counter = 0;
            s_rotate_bounce = 0;
            break;

        case SM_HARD_90_CW:
            s_state = STATE_SM_HARD_90;
            s_sm_rotate_dir = DIR_CW;
            s_sm_counter = 0;
            s_rotate_bounce = 0;
            /* 存编码器起始值 */
            {
                uint8_t ii;
                for (ii = 0; ii < 4; ii++)
                    s_sm_hard90_start[ii] = distance_mm[ii];
            }
            break;

        case SM_CLAW_OPEN:
            s_state = STATE_SM_CLAW_OPEN;
            /* 张爪 + 启动反转：P1开，P2转到-90 */
            g_claw_pwm1_angle = 80;
            g_claw_pwm2_angle = -90;
            Claw_ApplyAngles();
            /* 直接进入下一步 */
            s_sm_index++;
            SimpleStepStart();
            return;

        case SM_CLAW_GRAB:
            s_state = STATE_SM_CLAW_STOP;
            s_sm_claw_action = 2;
            s_sm_claw_stop_target = 150;  /* 1.5s */
            s_sm_counter = 0;
            SetMotors(0, 0);
            break;

        case SM_CLAW_RELEASE:
            s_state = STATE_SM_CLAW_STOP;
            s_sm_claw_action = 1;
            s_sm_claw_stop_target = 400;  /* 4s */
            s_sm_counter = 0;
            SetMotors(0, 0);
            break;

        case SM_STRAIGHT_WHITE:
            s_state = STATE_SM_STRAIGHT_WHITE;
            break;

        case SM_FINAL_STOP:
            s_state = STATE_FINAL_STOP;
            SetMotors(0, 0);
            break;

        default:
            s_sm_index++;
            SimpleStepStart();
            break;
    }
}

/* ── 当前步完成后，前进到下一步 ────────────────────── */
static void SimpleStepComplete(void)
{
    s_sm_index++;
    SimpleStepStart();
}

/* ========== STATE_SM_FOLLOW: PID巡线 ─────────======= */
static void StateSmFollow(float dt)
{
    const SimpleStep *step = &sm_right_side[s_sm_index];

    if (step->type == SM_FOLLOW_DIST)
    {
        if (Encoder_GetAvgDist() >= s_sm_dist_target)
        {
            SimpleStepComplete();
            return;
        }
    }
    else /* SM_FOLLOW_TIME */
    {
        s_sm_counter++;
        if (s_sm_counter >= step->time_ticks)
        {
            SimpleStepComplete();
            return;
        }
    }
    DriveWithPID(dt);
}

/* ========== STATE_SM_STRAIGHT: 纯直行 ─────────======= */
static void StateSmStraight(void)
{
    const SimpleStep *step = &sm_right_side[s_sm_index];
    s_sm_counter++;
    if (s_sm_counter >= step->time_ticks)
    {
        SimpleStepComplete();
        return;
    }
    SetMotors((int)s_base_speed, (int)s_base_speed);
}

/* ========== STATE_SM_REVERSE: 盲退 ──────────────── */
static void StateSmReverse(void)
{
    float dist = Encoder_GetAvgDist();
    if (dist < 0.0f) dist = -dist;
    if (dist >= s_sm_dist_target)
    {
        SimpleStepComplete();
        return;
    }
    ReverseWithBalance();
}

/* ========== STATE_SM_ROTATE_FINDLINE: 自转找白线 ── */
static void StateSmRotateFindLine(void)
{
    /* 反向制动期 */
    if (s_rotate_bounce > 0)
    {
        s_rotate_bounce--;
        TurnDir bounce = (s_sm_rotate_dir == DIR_CW) ? DIR_CCW : DIR_CW;
        SetRotateMotors(bounce);
        if (s_rotate_bounce == 0)
        {
            SimpleStepComplete();
        }
        return;
    }

    s_sm_counter++;

    /* 最小转动tick后才开始检测 */
    if (s_sm_counter >= FINDLINE_MIN_TICKS && MiddleOnLine())
    {
        s_rotate_bounce = 5;
        return;
    }

    /* 超时保护 */
    if (s_sm_counter >= FINDLINE_MAX_TICKS)
    {
        SimpleStepComplete();
        return;
    }

    SetRotateMotors(s_sm_rotate_dir);
}

/* ========== STATE_SM_HARD_90: 硬转90°CW ────────
 * 每轮弧长: 150mm × π/2 = 235.62mm
 * 用编码器绝对值平均判断旋转是否到位
 * ================================================= */
#define HARD_90_ARC_MM  235.62f
#define HARD_90_TIMEOUT 300U   /* 3s超时保护 */

static void StateSmHard90(void)
{
    s_sm_counter++;
    if (s_sm_counter >= HARD_90_TIMEOUT)
    {
        SimpleStepComplete();
        return;
    }

    /* 计算绝对值平均弧长 */
    float sum = 0;
    uint8_t i;
    for (i = 0; i < 4; i++) {
        float d = distance_mm[i] - s_sm_hard90_start[i];
        if (d < 0.0f) d = -d;
        sum += d;
    }
    float avg_arc = sum / 4.0f;

    if (avg_arc >= HARD_90_ARC_MM)
    {
        /* 短制动 */
        if (s_rotate_bounce == 0)
        {
            s_rotate_bounce = 5;
            SetRotateMotors(DIR_CCW);  /* CW转后用CCW制动 */
        }
        else
        {
            s_rotate_bounce--;
            if (s_rotate_bounce == 0)
            {
                SimpleStepComplete();
            }
        }
        return;
    }
    s_rotate_bounce = 0;
    SetRotateMotors(s_sm_rotate_dir);
}

/* ========== STATE_SM_STRAIGHT_WHITE: 直行到x2-x5全白 ── */
static void StateSmStraightWhite(void)
{
    if (AllMidOnWhite())
    {
        SimpleStepComplete();
        return;
    }
    SetMotors((int)s_base_speed, (int)s_base_speed);
}

/* ========== STATE_SM_CLAW_STOP: 停止+爪子动作 ────────── */
static void StateSmClawStop(void)
{
    SetMotors(0, 0);

    /* 爪子动作时序 */
    if (s_sm_claw_action == 1)  /* 释放: P1=44 → P2=90@1s → P1=80@2s */
    {
        if (s_sm_counter == 0)
        {
            g_claw_pwm1_angle = 44;
            Claw_ApplyAngles();
        }
        else if (s_sm_counter == 100)  /* 1s */
        {
            g_claw_pwm2_angle = 90;
            Claw_ApplyAngles();
        }
        else if (s_sm_counter == 200)  /* 2s */
        {
            g_claw_pwm1_angle = 80;
            Claw_ApplyAngles();
        }
    }
    else if (s_sm_claw_action == 2)  /* 抓取: P1=40 → P2=0@1s */
    {
        if (s_sm_counter == 0)
        {
            g_claw_pwm1_angle = 40;
            Claw_ApplyAngles();
        }
        else if (s_sm_counter == 100)  /* 1s */
        {
            g_claw_pwm2_angle = 0;
            Claw_ApplyAngles();
        }
    }

    s_sm_counter++;
    if (s_sm_counter >= s_sm_claw_stop_target)
    {
        /* 进入后退子步骤 */
        s_state = STATE_SM_CLAW_REVERSE;
        s_sm_counter = 0;
        s_sm_dist_target = 450.0f;
        Encoder_ResetDist();
        ReverseBalanceReset();
    }
}

/* ========== STATE_SM_CLAW_REVERSE: 爪子后退450mm ─────── */
static void StateSmClawReverse(void)
{
    float dist = Encoder_GetAvgDist();
    if (dist < 0.0f) dist = -dist;
    if (dist >= s_sm_dist_target)
    {
        /* 进入自转子步骤 */
        s_state = STATE_SM_CLAW_ROTATE;
        s_sm_counter = 0;
        s_sm_rotate_dir = DIR_CCW;  /* 右侧场地固定CCW自转回线 */
        s_rotate_bounce = 0;
        return;
    }
    ReverseWithBalance();
}

/* ========== STATE_SM_CLAW_ROTATE: 自转回线 ───────────── */
static void StateSmClawRotate(void)
{
    /* 反向制动期 */
    if (s_rotate_bounce > 0)
    {
        s_rotate_bounce--;
        TurnDir bounce = (s_sm_rotate_dir == DIR_CW) ? DIR_CCW : DIR_CW;
        SetRotateMotors(bounce);
        if (s_rotate_bounce == 0)
        {
            /* 释放动作：自转完成后P2=-90（反转180°完成）*/
            if (s_sm_claw_action == 1)
            {
                g_claw_pwm2_angle = -90;
                Claw_ApplyAngles();
            }
            /* 进入下一步 */
            s_sm_claw_action = 0;
            s_locked = 0;
            SimpleStepComplete();
        }
        return;
    }

    s_sm_counter++;

    if (s_sm_counter >= FINDLINE_MIN_TICKS && MiddleOnLine())
    {
        s_rotate_bounce = 5;
        return;
    }

    if (s_sm_counter >= FINDLINE_MAX_TICKS)
    {
        if (s_sm_claw_action == 1)
        {
            g_claw_pwm2_angle = -90;
            Claw_ApplyAngles();
        }
        s_sm_claw_action = 0;
        s_locked = 0;
        SimpleStepComplete();
        return;
    }

    SetRotateMotors(s_sm_rotate_dir);
}

/* ========== 简单巡线主调度 ========== */
static void SimpleMissionUpdate(float dt)
{
    switch (s_state)
    {
        case STATE_SM_INIT:          SimpleStepStart();                   break;
        case STATE_SM_FOLLOW:        StateSmFollow(dt);                   break;
        case STATE_SM_STRAIGHT:      StateSmStraight();                   break;
        case STATE_SM_REVERSE:       StateSmReverse();                    break;
        case STATE_SM_ROTATE_FINDLINE: StateSmRotateFindLine();          break;
        case STATE_SM_HARD_90:       StateSmHard90();                     break;
        case STATE_SM_STRAIGHT_WHITE: StateSmStraightWhite();             break;
        case STATE_SM_CLAW_STOP:     StateSmClawStop();                   break;
        case STATE_SM_CLAW_REVERSE:  StateSmClawReverse();                break;
        case STATE_SM_CLAW_ROTATE:   StateSmClawRotate();                 break;
        default:                     s_state = STATE_FOLLOW;              break;
    }
}

/* ========== 简单巡线模式初始化 ========== */
void LineTrack_InitSimple(float base_speed)
{
    /* 先做一次基础初始化 */
    s_base_speed = base_speed;
    s_last_position = 0.0f;
    s_locked = 0;

    PID_Init(&s_turn_pid, 23.0f, 0.8f, 1.8f,
             -base_speed * 1.5f, base_speed * 1.5f);

    /* 简单模式变量 */
    s_sm_enabled  = 1;
    s_sm_index    = 0;
    s_sm_counter  = 0;
    s_sm_dist_target = 0;
    s_sm_rotate_dir = DIR_NONE;
    s_sm_claw_action = 0;
    s_rotate_bounce = 0;

    /* 功能2 变量清零 */
    s_f2_enabled = 0;
    s_f2_phase   = 0;
    s_f2_counter = 0;

    /* 设置起始状态 */
    s_state = STATE_SM_INIT;
}

void LineTrack_Stop(void)
{
    PID_Reset(&s_turn_pid);
    SetMotors(0, 0);
    s_f2_enabled = 0;
    s_f2_phase   = 0;
    s_f2_counter = 0;
    s_sm_enabled = 0;
    s_state = STATE_FOLLOW;
    s_last_position = 0.0f;
    s_cooldown = 0;
    s_locked = 0;
    s_edge_rearm = 0;
    s_edge_off_count = 0;
}
