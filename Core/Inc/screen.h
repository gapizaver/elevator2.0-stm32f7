// LCD Double Buffer manager for STM32
// @author toxi
// https://gist.github.com/postspectacular/61f17333c17b0206a73e4591cd5ce59b

#pragma once

#include "stm32f769i_discovery_lcd.h"

typedef struct {
	uint32_t addr[2];
	uint32_t width;
	uint32_t height;
	uint32_t front;
} Screen;

Screen* ct_screen_init();
void ct_screen_flip_buffers(Screen *screen);
uint32_t* ct_screen_backbuffer_ptr(Screen *screen);

static inline uint32_t ct_screen_backbuffer_id(Screen *screen) {
	return 1 - screen->front;
}
