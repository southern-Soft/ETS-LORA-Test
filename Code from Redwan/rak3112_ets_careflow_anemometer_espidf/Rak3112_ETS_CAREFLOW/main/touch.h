#pragma once

void touch_init(unsigned short int w, unsigned short int h, unsigned char r);
bool touch_touched(void);
bool touch_has_signal(void);
bool touch_released(void);

extern int touch_last_x;
extern int touch_last_y;