//
// Created by fish on 2025/4/4.
//

#pragma once

#include "app_music_def.h"

// 定义低音
#define A1  131
#define A2  147
#define A3  165
#define A4  175
#define A5  196
#define A6  220
#define A7  247

// 定义中音
#define B1  262
#define B2  296
#define B3  330
#define B4  349
#define B5  392
#define B6  440
#define B7  494

// 定义高音
#define C1  523
#define C2  587
#define C3  659
#define C4  698
#define C4p 741
#define C5  784
#define C6  880
#define C7  988

// 定义高二度
#define D1  1047
#define D2  1175
#define D3  1319
#define D4  1397
#define D5  1568
#define D6  1760
#define D7  1976

// 定义节拍
#define OneBeat   200
#define HalfBeat  100

inline app_music_note_t app_music_notes_boot[] = {
    // {D7, 0.5, 125}, {0, 0, 50}, {D7, 0.5, 125},

    // {0, 0, 2},
    // {B1, 0.1, HalfBeat * 2},
    // {0, 0, 3},
    // {C3, 0.1, HalfBeat * 2},
    // {0, 0, 3},
    // {D5, 0.1, HalfBeat * 2},
    // {0, 0, 4},
    // {D1, 0.05, HalfBeat * 2},
    // {0, 0, 4},
    // {D1, 0.05, HalfBeat * 2},
    // {0, 0, 3},
    // {D5, 0.1, HalfBeat * 2},
    // {0, 0, 3},
    // {C3, 0.1, HalfBeat * 2},
    // {0, 0, 2},
    // {B1, 0.1, HalfBeat * 2},



    // 短暂静音作为起始
    {0, 0, 5},

    // 上升启动感：中高音逐级上升
    {B3, 0.12f, HalfBeat},      // 330 Hz，轻一点的“滴”
    {C3, 0.14f, HalfBeat},      // 659 Hz，明显更亮
    {D3, 0.16f, HalfBeat},      // 1319 Hz，更尖，像能量提升

    {0, 0, 10},                 // 稍明显一点的停顿

    // 科幻感三连音：高频快速闪一下
    {D5, 0.18f, HalfBeat},      // 1568 Hz，“滴”
    {0, 0, HalfBeat / 2},       // 间隔 50 ms
    {D6, 0.20f, HalfBeat},      // 1760 Hz，更高“滴”
    {0, 0, HalfBeat / 2},       // 间隔 50 ms
    {D5, 0.18f, HalfBeat},      // 回落一点，“滴”

    {0, 0, 10},                 // 短停顿

    // 下滑收尾：像系统锁定/就绪的感觉
    {C3, 0.14f, HalfBeat * 2},  // 659 Hz，拉长一点
    {B2, 0.12f, HalfBeat * 2},  // 296 Hz，稍微低一点作为结束





    // {B7,  0.4f, OneBeat},       // xi (Ti)
    // {B7,  0.4f, OneBeat},       // xi (重复)
    // {B5,  0.3f, OneBeat},       // so (G)
    // {415, 0.35f, OneBeat},        // 升so (G# / 415 Hz)，稍作拉长作为结束



};


inline app_music_note_t app_music_notes_switch_mode1[] = {
    {B3, 0.5f, HalfBeat},      // 330 Hz，轻一点的“滴”
    {0, 0, HalfBeat / 2},       // 间隔 50 ms
    {B3, 0.5f, HalfBeat},      // 330 Hz，轻一点的“滴”
};

inline app_music_note_t app_music_notes_switch_mode2[] = {
    {C3, 0.5f, HalfBeat},      // 659 Hz，明显更亮
    {0, 0, HalfBeat / 2},       // 间隔 50 ms
    {C3, 0.5f, HalfBeat},      // 659 Hz，明显更亮

};

inline app_music_note_t app_music_notes_switch_mode3[] = {
    {D6, 0.5f, HalfBeat},      // 1760 Hz，更高“滴”
    {0, 0, HalfBeat / 2},       // 间隔 50 ms
    {D6, 0.5f, HalfBeat},      // 1760 Hz，更高“滴”
};