/* 防止头文件重复包含 */
#pragma once

#include <iostream>

/*
 * 状态枚举：定义机器人所有可能的状态
 * 每个状态对应一种运动控制模式
 */
enum class State {
    IDLE = 0,                          /* 空闲状态 */
    ZERO = 1,                          /* 零位 */
    RL_walk = 2,                       /* 走路 */
    RL_mimic = 3,                      /* 招手 */
    RL_long_motion = 4,                /* 出拳舞 */
    RL_forward_punch = 5,              /* 打拳 */
    RL_forward_kick = 6,               /* 踢腿 */
    RL_climb_up = 7,                   /* 爬起 */
    RL_crane_down = 8,                 /* 趴下 */
    RL_taichi = 9,                     /* 太极 */
    RL_kongfu = 10,                    /* 功夫 */
    
    /*------------------------- 导览(LowerCPG) -------------------------*/
    RL_introduct_left = 11,          /* 左介绍 */
    RL_introduct_double_2 = 12,      /* 双手介绍2 */
    RL_introduct_right = 14,         /* 右介绍 */
    RL_scratch_head = 15,            /* 挠头 */
    RL_introduct_double_1 = 16,      /* 双手介绍1 */
    RL_wave_double = 17,             /* 双手挥手 */
    RL_handshake = 18,               /* 握手 */
    /*----------------------------------------------------------------*/
    RL_introduct = 19,               /* 自我介绍 */
    RL_dance = 20,                   /* 舞蹈 */
    /*----------------------------- ASE -------------------------------*/
    RL_ASE_combinations = 23,        /* ase-组合拳 */
    RL_ASE_sidekick = 43,            /* ase-侧踢 */
    RL_ASE_uppercut = 56,            /* ase-上勾拳 */
    RL_ASE_stand = 60,               /* ase-站立 */
};

/*
 * 事件枚举：定义状态机可以触发的事件类型
 * START - 初始化状态
 * RUN   - 持续运行状态
 * EXIT  - 退出当前状态
 */
enum class Event {
    START,  /* 启动 */
    RUN,    /* 运行 */
    EXIT    /* 退出 */
};
