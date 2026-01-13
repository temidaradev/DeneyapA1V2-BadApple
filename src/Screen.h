#ifndef SCREEN_H
#define SCREEN_H

#include <Arduino.h>
#include <Wire.h>
#include <Deneyap_OLED.h>
#include <FS.h>

void initScreen();
void playVideo(fs::FS &fs, const char *path);

#endif
