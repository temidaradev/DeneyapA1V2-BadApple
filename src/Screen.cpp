#include "Screen.h"
#include "Audio.h"
#include "heatshrink_decoder.h"

OLED display;
static heatshrink_decoder hsd;

uint8_t frameBuffer[1024];
int16_t curr_x = 0;
int16_t curr_y = 0;

int32_t runlength = -1;
int32_t c_to_dup = -1;
uint32_t lastRefresh = 0;

void putPixels(uint8_t c, int32_t len);
void decodeRLE(uint8_t c);

void initScreen() {
    if (!display.begin(0x7A)) {
        Serial.println("I2C connection failed");
        return;
    }
    Wire.setClock(400000);
    display.setHorizontalMode();
    display.setTextXY(0, 0);
}

void putPixels(uint8_t c, int32_t len) {
  uint8_t b = 0;
  while (len--) {
    b = 128;
    for (int i = 0; i < 8; i++) {
      if (c & b) {
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
          Wire.beginTransmission(0x7A >> 1);
          Wire.write(0x40); 
          Wire.write(frameBuffer, 31);
          Wire.endTransmission();

          for (int i = 31; i < 1024; i += 32) {
            Wire.beginTransmission(0x7A >> 1);
            Wire.write(0x40);
            int remaining = (1024 - i) > 32 ? 32 : (1024 - i);
            Wire.write(&frameBuffer[i], remaining);
            Wire.endTransmission();
          }

          uint32_t setBits = 0;
          for(int i=0; i<1024; i++) setBits += __builtin_popcount(frameBuffer[i]);
          
          uint32_t intensity = (setBits * 40 / 8192); 
          if(intensity > 255) intensity = 255;
          neopixelWrite(48, intensity, intensity, intensity);

          memset(frameBuffer, 0, 1024);
          
          while((millis() - lastRefresh) < 33) {
             if ((millis() - lastRefresh) < 30) {
                vTaskDelay(1 / portTICK_PERIOD_MS); 
             }
          }
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

void playVideo(fs::FS &fs, const char *path) {
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
    return;
  }
  filelen = file.size();
  filesize = filelen;

  display.clearDisplay();
  memset(frameBuffer, 0, 1024);
  curr_x = 0;
  curr_y = 0;
  runlength = -1;
  c_to_dup = -1;
  lastRefresh = millis();

  startAudioPlayback();

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
        return;
      }

      rle_bufhead = 0;
      while (rle_size) {
        rle_size--;
        if (rle_bufhead >= RLEBUFSIZE) {
          return;
        }
        decodeRLE(rle_buf[rle_bufhead++]);
      }
    } while (pres == HSDR_POLL_MORE);
  }
  file.close();
  stopAudioPlayback();
}
