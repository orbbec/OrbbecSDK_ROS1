#pragma once

extern "C" {
void yuv422InitMemory(int width, int height);

void yuv422FreeMemory();

void yuv422ToRgb(uint8_t* y_plane, uint8_t* u_plane, uint8_t* v_plane, unsigned char* rgb, int width, int height);
}