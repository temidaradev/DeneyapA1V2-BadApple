#include "FS.h"
#include "heatshrink_decoder.h"
#include <Deneyap_OLED.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

// SD Card SPI pins for Deneyap Kart 1A v2 (ESP32-S3)
#define SD_SCK_PIN 13  // CLK (D16)
#define SD_MISO_PIN 14 // MISO (D19)
#define SD_MOSI_PIN 12 // MOSI (D17)
#define SD_CS_PIN 11   // CS (D18)

// ============================================
// DENEYAP SPEAKER MODULE WIRING:
// ============================================
// Deneyap 3.3V ───> Module 3V3
// Deneyap GND  ───> Module GND
// Deneyap GPIO15 (A4) ───> Module IN+
// Deneyap 3.3V ───> Module SD (enable)
// Module OUT+/OUT- ───> Speaker
// ============================================

#define SPEAKER_PIN 15 // PWM output to speaker module IN+

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

// WAV audio file
File audioFile;
bool audioAvailable = false;
uint32_t audioDataStart = 0;
uint32_t audioDataSize = 0;
uint32_t audioSampleRate = 22050;
uint16_t audioBytesPerSample = 2;
uint16_t audioChannels = 1;
uint16_t audioBitsPerSample = 16;

// Audio playback task
TaskHandle_t audioTaskHandle = NULL;
volatile bool audioPlaying = false;

// Audio task - runs on core 0, plays audio via PWM
void audioTask(void *parameter) {
  Serial.println("[AUDIO] Task started on core 0");

  // Configure LEDC for audio PWM
  // Using 40kHz PWM - high enough to not be audible, low enough to work with
  // most amps
  ledcSetup(0, 40000, 8); // Channel 0, 40kHz, 8-bit resolution
  ledcAttachPin(SPEAKER_PIN, 0);
  ledcWrite(0, 0); // Start at 0 (silence for single-ended input)

  // Calculate delay between samples in microseconds
  uint32_t sampleDelayUs = 1000000 / audioSampleRate;
  uint32_t lastSampleTime = micros();

  Serial.printf("[AUDIO] Sample rate: %d Hz, delay: %d us\n", audioSampleRate,
                sampleDelayUs);
  Serial.println("[AUDIO] Playing audio...");

  while (audioPlaying) {
    if (!audioFile || !audioFile.available()) {
      ledcWrite(0, 0); // Silence
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }

    // Read a chunk of audio samples
    const int chunkSize = 128;
    int16_t samples[chunkSize];
    int samplesRead = 0;

    for (int i = 0; i < chunkSize && audioFile.available(); i++) {
      if (audioChannels == 2) {
        // Stereo: read both channels and mix to mono
        int16_t left, right;
        if (audioFile.read((uint8_t *)&left, 2) == 2 &&
            audioFile.read((uint8_t *)&right, 2) == 2) {
          samples[i] = (left / 2) + (right / 2);
          samplesRead++;
        }
      } else {
        // Mono
        if (audioFile.read((uint8_t *)&samples[i], 2) == 2) {
          samplesRead++;
        }
      }
    }

    // Play the samples with precise timing
    for (int i = 0; i < samplesRead; i++) {
      // Wait for the right time to output this sample
      while ((micros() - lastSampleTime) < sampleDelayUs) {
        // Busy wait for precise timing
      }
      lastSampleTime = micros();

      // Convert 16-bit signed (-32768 to 32767) to 8-bit unsigned (0-255)
      int32_t sample = samples[i];
      // Scale to 0-255 range
      uint8_t pwmValue = (uint8_t)((sample + 32768) >> 8);

      ledcWrite(0, pwmValue);
    }
  }

  // Silence when stopped
  ledcWrite(0, 0);
  Serial.println("[AUDIO] Task stopped");
  vTaskDelete(NULL);
}

void startAudioPlayback() {
  if (!audioAvailable)
    return;

  audioPlaying = true;

  // Create audio task on core 0 (video runs on core 1)
  xTaskCreatePinnedToCore(audioTask, "AudioTask", 8192, NULL,
                          2, // Higher priority for audio
                          &audioTaskHandle,
                          0 // Core 0
  );
}

void stopAudioPlayback() {
  audioPlaying = false;
  if (audioTaskHandle != NULL) {
    vTaskDelay(200 / portTICK_PERIOD_MS);
    audioTaskHandle = NULL;
  }
  ledcWrite(0, 128); // Silence
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
            setBits += __builtin_popcount(n);
          }

          static uint32_t lastBits = 0;
          static uint32_t frameCount = 0;
          frameCount++;

          int32_t delta = abs((int32_t)setBits - (int32_t)lastBits);
          lastBits = setBits;

          if (frameCount % 30 == 0) {
            Serial.printf("[FRAME %lu] Pixels:%lu | Motion:%ld\n", frameCount,
                          setBits, delta);
          }

          // Rainbow Cycle
          static uint16_t colorPos = 0;
          colorPos = (colorPos + 6) % 768;

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

          uint32_t intensity = (setBits * 40 / 8192) + (delta * 180 / 1000);
          if (intensity > 255)
            intensity = 255;

          neopixelWrite(48, (r_base * intensity) / 255,
                        (g_base * intensity) / 255, (b_base * intensity) / 255);

          memset(frameBuffer, 0, 1024);
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

  display.clearDisplay();
  memset(frameBuffer, 0, 1024);
  curr_x = 0;
  curr_y = 0;
  runlength = -1;
  c_to_dup = -1;
  lastRefresh = millis();

  // Start audio playback on separate core
  if (audioAvailable) {
    startAudioPlayback();
  }

  heatshrink_decoder_reset(&hsd);
  size_t count = 0;
  uint32_t sunk = 0;
  size_t toRead;
  size_t toSink = 0;
  uint32_t sinkHead = 0;

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

  stopAudioPlayback();

  if (audioAvailable && audioFile) {
    Serial.println("[AUDIO] Closing audio file.");
    audioFile.close();
  }

  Serial.println("========================================");
  Serial.println("=== PLAYBACK FINISHED ===");
  Serial.println("Restarting in 5 seconds...");
  delay(5000);
  ESP.restart();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n========================================");
  Serial.println("   ESP32-S3 BadApple + TEA2025B Audio");
  Serial.println("========================================");

  // Initialize Deneyap OLED display
  if (!display.begin(0x7A)) {
    delay(3000);
    Serial.println("I2C connection failed");
    return;
  }

  Wire.setClock(400000);

  // Initialize SD card
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

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC)
    Serial.println("MMC");
  else if (cardType == CARD_SD)
    Serial.println("SDSC");
  else if (cardType == CARD_SDHC)
    Serial.println("SDHC");

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  display.setHorizontalMode();
  display.setTextXY(0, 0);

  pinMode(0, INPUT_PULLUP);
  Serial.print("Total space: ");
  Serial.println(SD.totalBytes());
  Serial.print("Used space: ");
  Serial.println(SD.usedBytes());
  listDir(SD, "/", 0);

  // Load WAV audio file
  Serial.println("\n=== Loading Audio File ===");
  audioFile = SD.open("/badapple.wav");
  if (audioFile && audioFile.size() > 44) {
    uint8_t header[44];
    audioFile.read(header, 44);

    if (header[0] == 'R' && header[1] == 'I' && header[2] == 'F' &&
        header[3] == 'F') {
      audioChannels = header[22] | (header[23] << 8);
      audioSampleRate = header[24] | (header[25] << 8) | (header[26] << 16) |
                        (header[27] << 24);
      audioBitsPerSample = header[34] | (header[35] << 8);
      audioBytesPerSample = audioBitsPerSample / 8;
      audioDataStart = 44;
      audioDataSize = audioFile.size() - 44;

      audioAvailable = true;

      Serial.println("✓ Audio file loaded!");
      Serial.printf("  Channels: %d (%s)\n", audioChannels,
                    audioChannels == 1 ? "Mono" : "Stereo");
      Serial.printf("  Sample Rate: %d Hz\n", audioSampleRate);
      Serial.printf("  Bits/Sample: %d\n", audioBitsPerSample);
      Serial.printf("  Duration: ~%d seconds\n",
                    audioDataSize / (audioSampleRate * audioBytesPerSample *
                                     audioChannels));

      Serial.println("\n=== TEA2025B Wiring ===");
      Serial.printf("  GPIO%d --> 10K resistor --> TEA2025B INPUT\n",
                    SPEAKER_PIN);
      Serial.println("                        |");
      Serial.println("                     100nF cap");
      Serial.println("                        |");
      Serial.println("                       GND");

    } else {
      Serial.println("✗ Invalid WAV format");
      audioFile.close();
    }
  } else {
    Serial.println("✗ badapple.wav not found - video only");
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

void loop() { delay(1000); }
