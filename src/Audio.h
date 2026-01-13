#ifndef AUDIO_H
#define AUDIO_H

#include <Arduino.h>
#include <FS.h>
#include <SD.h>

void initAudio();
bool loadAudio(fs::FS &fs, const char* path);
void startAudioPlayback();
void stopAudioPlayback();

#endif
