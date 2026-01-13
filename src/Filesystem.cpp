#include "Filesystem.h"

#define SD_SCK_PIN 13
#define SD_MISO_PIN 14
#define SD_MOSI_PIN 12 
#define SD_CS_PIN 11

bool initSD() {
    Serial.println("\n=== Initializing SD Card ===");
    Serial.printf("Pins - SCK: %d, MISO: %d, MOSI: %d, CS: %d\n", SD_SCK_PIN,
                  SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    delay(100);

    Serial.println("Attempting to mount SD card...");
    if (!SD.begin(SD_CS_PIN, SPI, 4000000)) {
        Serial.println("Failed at 4MHz, trying 1MHz...");
        if (!SD.begin(SD_CS_PIN, SPI, 1000000)) {
            Serial.println("\n❌ SD Card mount failed!");
            return false;
        }
    }
    Serial.println("✓ SD Card mounted successfully!");
    return true;
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}
