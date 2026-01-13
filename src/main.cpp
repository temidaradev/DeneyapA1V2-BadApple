#include <Arduino.h>
#include "Filesystem.h"
#include "Audio.h"
#include "Screen.h"

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n========================================");
  Serial.println("   Deneyap-A1-V2 BadApple Refactored");
  Serial.println("========================================");

  initScreen();
  
  if (!initSD()) {
      return; 
  }

  Serial.print("Total space: ");
  Serial.println(SD.totalBytes());
  Serial.print("Used space: ");
  Serial.println(SD.usedBytes());
  listDir(SD, "/", 0);

  initAudio();
  loadAudio(SD, "/badapple.wav");

  Serial.println("\n[MAIN] Starting video playback...");
  playVideo(SD, "/video.hs");

  Serial.println("=== PLAYBACK FINISHED ===");
  Serial.println("Restarting in 5 seconds...");
  delay(5000);
  ESP.restart();
}

void loop() {
  delay(1000);
}
