#include "FS.h"
#include "heatshrink_decoder.h"
#include <Deneyap_OLED.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

// SD Card SPI pins for Deneyap Kart 1A v2 (ESP32-S3)
// Correct pins for onboard SD card slot
#define SD_SCK_PIN 13  // CLK (D16)
#define SD_MISO_PIN 14 // MISO (D19)
#define SD_MOSI_PIN 12 // MOSI (D17)
#define SD_CS_PIN 11   // CS (D18)

// Speaker output pin (PWM for audio)
#define SPEAKER_PIN 15 // A4 (GPIO 15) - PWM output to speaker
#define AUDIO_PWM_CHANNEL 0
#define AUDIO_PWM_FREQ 44100   // 44.1kHz sample rate
#define AUDIO_PWM_RESOLUTION 8 // 8-bit resolution

// Deneyap OLED display
OLED display;

#if HEATSHRINK_DYNAMIC_ALLOC
#error HEATSHRINK_DYNAMIC_ALLOC must be false for static allocation test suite.
#endif

static heatshrink_decoder hsd;

// Frame buffer for 128x64 display (1024 bytes)
uint8_t frameBuffer[1024];

// global storage for putPixels
int16_t curr_x = 0;
int16_t curr_y = 0;

// global storage for decodeRLE
int32_t runlength = -1;
int32_t c_to_dup = -1;

// WAV audio file for rhythm
File audioFile;
bool audioAvailable = false;
uint32_t audioDataStart = 0;
uint32_t audioDataSize = 0;
uint16_t audioSampleRate = 44100;
uint16_t audioBytesPerSample = 2;
uint32_t audioSamplesRead = 0;

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

uint32_t lastRefresh = 0;

void putPixels(uint8_t c, int32_t len) {
  uint8_t b = 0;
  while (len--) {
    b = 128;
    for (int i = 0; i < 8; i++) {
      if (c & b) {
        // Set pixel in frame buffer
        int page = curr_y / 8;
        int bit = curr_y % 8;
        int bufferIndex = page * 128 + curr_x;
        frameBuffer[bufferIndex] |= (1 << bit);
      }
      b >>= 1;
      curr_x++;
      if (curr_x >= 128) {
        curr_x = 0;
        curr_y++;
        if (curr_y >= 64) {
          curr_y = 0;
          // Write frame buffer directly to I2C in optimized chunks
          Wire.beginTransmission(0x7A >> 1);
          Wire.write(0x40);            // Data mode
          Wire.write(frameBuffer, 31); // Write first 31 bytes
          Wire.endTransmission();

          // Write remaining bytes in 32-byte chunks
          for (int i = 31; i < 1024; i += 32) {
            Wire.beginTransmission(0x7A >> 1);
            Wire.write(0x40);
            int remaining = (1024 - i) > 32 ? 32 : (1024 - i);
            Wire.write(&frameBuffer[i], remaining);
            Wire.endTransmission();
          }

          // Bad Apple Rhythm: Sync RGB LED with rapid brightness shifts
          uint32_t setBits = 0;
          for (int i = 0; i < 1024; i++) {
            uint8_t n = frameBuffer[i];
            setBits += __builtin_popcount(n); // Ultra fast bit count
          }

          static uint32_t lastBits = 0;
          static uint32_t frameCount = 0;
          frameCount++;

          // Calculate the difference between frames for a "strobe" effect
          int32_t delta = abs((int32_t)setBits - (int32_t)lastBits);
          lastBits = setBits;

          // Read audio amplitude for rhythm sync
          uint8_t audioIntensity = 0;
          static uint32_t totalSamplesProcessed = 0;

          if (audioAvailable && audioFile && audioFile.available()) {
            // Read ~100 samples per frame for smooth rhythm
            int16_t samples[100];
            size_t samplesToRead = 100 * audioBytesPerSample;
            size_t bytesRead =
                audioFile.read((uint8_t *)samples, samplesToRead);

            if (bytesRead > 0) {
              // Play audio samples through PWM
              int numSamples = bytesRead / audioBytesPerSample;
              for (int i = 0; i < numSamples; i++) {
                // Convert 16-bit signed to 8-bit unsigned (0-255)
                uint8_t pwmValue = (samples[i] >> 8) + 128;
                ledcWrite(AUDIO_PWM_CHANNEL, pwmValue);
                delayMicroseconds(23); // ~44.1kHz timing
              }

              // Calculate RMS (Root Mean Square) for LED intensity
              uint32_t sumSquares = 0;
              for (int i = 0; i < numSamples; i++) {
                int32_t sample = abs(samples[i]);
                sumSquares += (sample * sample) >> 16;
              }
              uint32_t rms = sqrt(sumSquares / numSamples);
              audioIntensity = (rms > 255) ? 255 : rms;
              totalSamplesProcessed += numSamples;

              // Log every 30 frames (~2 seconds at 15fps)
              if (frameCount % 30 == 0) {
                Serial.printf("[FRAME %lu] Pixels:%lu | AudioRMS:%d | "
                              "Motion:%ld | Samples:%lu\n",
                              frameCount, setBits, audioIntensity, delta,
                              totalSamplesProcessed);
              }
            }
          } else if (frameCount % 30 == 0) {
            Serial.printf("[FRAME %lu] Pixels:%lu | Motion:%ld | (No audio)\n",
                          frameCount, setBits, delta);
          }

          // Rainbow Cycle: Smoothly transition between colors
          static uint16_t colorPos = 0;
          colorPos = (colorPos + 6) % 768; // Speed of color change

          uint8_t r_base, g_base, b_base;
          if (colorPos < 256) {
            r_base = 255 - colorPos;
            g_base = colorPos;
            b_base = 0;
          } else if (colorPos < 512) {
            r_base = 0;
            g_base = 255 - (colorPos - 256);
            b_base = colorPos - 256;
          } else {
            r_base = colorPos - 512;
            g_base = 0;
            b_base = 255 - (colorPos - 512);
          }

          // Modulation: Audio-driven rhythm or video fallback
          uint32_t intensity = 0;
          if (audioAvailable && audioIntensity > 0) {
            intensity = (audioIntensity * 220 / 255) + (delta * 35 / 1000);
          } else {
            intensity = (setBits * 40 / 8192) + (delta * 180 / 1000);
          }
          if (intensity > 255)
            intensity = 255;

          // Color strobe effect
          neopixelWrite(48, (r_base * intensity) / 255,
                        (g_base * intensity) / 255, (b_base * intensity) / 255);

          // Clear frame buffer for next frame
          memset(frameBuffer, 0, 1024);
          // No frame rate limiting - run at maximum speed
          lastRefresh = millis();
        }
      }
    }
  }
}

void decodeRLE(uint8_t c) {
  if (c_to_dup == -1) {
    if ((c == 0x55) || (c == 0xaa)) {
      c_to_dup = c;
    } else {
      putPixels(c, 1);
    }
  } else {
    if (runlength == -1) {
      if (c == 0) {
        putPixels(c_to_dup & 0xff, 1);
        c_to_dup = -1;
      } else if ((c & 0x80) == 0) {
        if (c_to_dup == 0x55) {
          putPixels(0, c);
        } else {
          putPixels(255, c);
        }
        c_to_dup = -1;
      } else {
        runlength = c & 0x7f;
      }
    } else {
      runlength = runlength | (c << 7);
      if (c_to_dup == 0x55) {
        putPixels(0, runlength);
      } else {
        putPixels(255, runlength);
      }
      c_to_dup = -1;
      runlength = -1;
    }
  }
}

#define RLEBUFSIZE 4096
#define READBUFSIZE 2048
void readFile(fs::FS &fs, const char *path) {
  static uint8_t rle_buf[RLEBUFSIZE];
  size_t rle_bufhead = 0;
  size_t rle_size = 0;

  size_t filelen = 0;
  size_t filesize;
  static uint8_t compbuf[READBUFSIZE];

  Serial.printf("\n=== READING VIDEO FILE: %s ===\n", path);
  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("[ERROR] Failed to open video file");
    display.setTextXY(2, 0);
    display.putString("File open error");
    display.setTextXY(3, 0);
    display.putString("Upload video.hs");
    return;
  }
  filelen = file.size();
  filesize = filelen;
  Serial.printf("[VIDEO] File size: %d bytes\n", filelen);
  Serial.printf("[VIDEO] Starting decompression...\n");

  // init display, putPixels and decodeRLE
  display.clearDisplay();
  memset(frameBuffer, 0, 1024);
  curr_x = 0;
  curr_y = 0;
  runlength = -1;
  c_to_dup = -1;
  lastRefresh = millis();

  // init decoder
  heatshrink_decoder_reset(&hsd);
  size_t count = 0;
  uint32_t sunk = 0;
  size_t toRead;
  size_t toSink = 0;
  uint32_t sinkHead = 0;

  // Go through file...
  while (filelen) {
    if (toSink == 0) {
      toRead = filelen;
      if (toRead > READBUFSIZE)
        toRead = READBUFSIZE;
      file.read(compbuf, toRead);
      filelen -= toRead;
      toSink = toRead;
      sinkHead = 0;
    }

    // uncompress buffer
    HSD_sink_res sres;
    sres = heatshrink_decoder_sink(&hsd, &compbuf[sinkHead], toSink, &count);
    toSink -= count;
    sinkHead = count;
    sunk += count;
    if (sunk == filesize) {
      heatshrink_decoder_finish(&hsd);
    }

    HSD_poll_res pres;
    do {
      rle_size = 0;
      pres = heatshrink_decoder_poll(&hsd, rle_buf, RLEBUFSIZE, &rle_size);
      if (pres < 0) {
        Serial.print("POLL ERR! ");
        Serial.println(pres);
        return;
      }

      rle_bufhead = 0;
      while (rle_size) {
        rle_size--;
        if (rle_bufhead >= RLEBUFSIZE) {
          Serial.println("RLE_SIZE ERR!");
          return;
        }
        decodeRLE(rle_buf[rle_bufhead++]);
      }
    } while (pres == HSDR_POLL_MORE);
  }
  file.close();

  Serial.println("\n========================================");
  Serial.println("[VIDEO] Playback complete!");
  Serial.printf("[VIDEO] File processed: %d bytes\n", filesize);

  if (audioAvailable && audioFile) {
    Serial.println("[AUDIO] Closing audio file.");
    audioFile.close();
  }

  Serial.println("========================================");
  Serial.println("=== PLAYBACK FINISHED ===");
  Serial.println("Restarting in 5 seconds...");
  delay(5000);
  ESP.restart(); // Auto-restart for loop playback
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n=== ESP32 BadApple Starting ===");

  // Initialize PWM for speaker output
  Serial.println("\n=== Initializing Speaker ===");
  ledcSetup(AUDIO_PWM_CHANNEL, AUDIO_PWM_FREQ, AUDIO_PWM_RESOLUTION);
  ledcAttachPin(SPEAKER_PIN, AUDIO_PWM_CHANNEL);
  ledcWrite(AUDIO_PWM_CHANNEL, 128); // Center position (silence)
  Serial.printf("Speaker on GPIO %d (PWM Channel %d)\n", SPEAKER_PIN,
                AUDIO_PWM_CHANNEL);

  // Initialize Deneyap OLED display
  if (!display.begin(0x7A)) {
    delay(3000);
    Serial.println("I2C connection failed");
    return;
  }

  // Increase I2C speed to 400kHz for faster transfers
  Wire.setClock(400000);

  // Initialize SD card with correct pins
  Serial.println("\n=== Initializing SD Card ===");
  Serial.printf("Pins - SCK: %d, MISO: %d, MOSI: %d, CS: %d\n", SD_SCK_PIN,
                SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  delay(100);

  Serial.println("Attempting to mount SD card...");
  if (!SD.begin(SD_CS_PIN, SPI, 4000000)) { // 4MHz
    Serial.println("Failed at 4MHz, trying 1MHz...");
    if (!SD.begin(SD_CS_PIN, SPI, 1000000)) { // 1MHz
      Serial.println("\n❌ SD Card mount failed!");
      Serial.println("Please check:");
      Serial.println("  1. SD card is properly inserted");
      Serial.println("  2. SD card is formatted as FAT32");
      Serial.println("  3. SD card is working (test in PC)");
      display.clearDisplay();
      display.setTextXY(1, 0);
      display.putString("SD Card");
      display.setTextXY(2, 0);
      display.putString("Mount Failed!");
      delay(5000);
      return;
    }
  }

  Serial.println("✓ SD Card mounted successfully!");

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    display.clearDisplay();
    display.setTextXY(2, 0);
    display.putString("No SD card!");
    delay(3000);
    return;
  }

  Serial.println("SD Card mounted successfully");
  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC)
    Serial.println("MMC");
  else if (cardType == CARD_SD)
    Serial.println("SDSC");
  else if (cardType == CARD_SDHC)
    Serial.println("SDHC");

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  // Set display to horizontal addressing mode for fast writes
  display.setHorizontalMode();
  display.setTextXY(0, 0);

  pinMode(0, INPUT_PULLUP);
  Serial.print("Total space: ");
  Serial.println(SD.totalBytes());
  Serial.print("Used space: ");
  Serial.println(SD.usedBytes());
  listDir(SD, "/", 0);

  // Try to open WAV audio file for rhythm sync
  Serial.println("\n=== Loading Audio File ===");
  audioFile = SD.open("/badapple.wav");
  if (audioFile && audioFile.size() > 44) {
    // Parse WAV header
    uint8_t header[44];
    audioFile.read(header, 44);

    // Check RIFF header
    if (header[0] == 'R' && header[1] == 'I' && header[2] == 'F' &&
        header[3] == 'F') {
      audioSampleRate = header[24] | (header[25] << 8) | (header[26] << 16) |
                        (header[27] << 24);
      audioBytesPerSample = (header[34] | (header[35] << 8)) / 8;
      audioDataStart = 44;
      audioDataSize = audioFile.size() - 44;
      audioAvailable = true;

      Serial.println("✓ Audio file loaded!");
      Serial.printf("  Sample Rate: %d Hz\n", audioSampleRate);
      Serial.printf("  Bytes/Sample: %d\n", audioBytesPerSample);
      Serial.printf("  Duration: ~%d seconds\n",
                    audioDataSize / (audioSampleRate * audioBytesPerSample));
    } else {
      Serial.println("✗ Invalid WAV format");
      audioFile.close();
    }
  } else {
    Serial.println("✗ badapple.wav not found - using video rhythm only");
  }

  display.clearDisplay();
  display.setTextXY(2, 0);
  display.putString(" BAD APPLE ");
  display.setTextXY(3, 0);
  display.putString("Starting...");
  delay(1000);

  Serial.println("\n[MAIN] Starting video playback...");
  readFile(SD, "/video.hs");
}

void loop() {
  // Main loop - video playback handled in readFile()
  delay(1000);
}
