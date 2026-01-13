#ifndef FILESYSTEM_H
#define FILESYSTEM_H

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <FS.h> // Ensure FS is included

bool initSD();
void listDir(fs::FS &fs, const char *dirname, uint8_t levels);

#endif
