//
// Created by fish on 2025/4/4.
//

#pragma once

enum app_sys_music_e {
    E_MUSIC_BOOT,
    E_MUSIC_YOU,
    E_MUSIC_MODE1,
    E_MUSIC_MODE2,
    E_MUSIC_MODE3
};

void app_sys_music_play(app_sys_music_e e);
void app_sys_music_stop();