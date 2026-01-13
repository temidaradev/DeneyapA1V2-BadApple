#include "Audio.h"

#define SPEAKER_PIN 15 
#define POT_PIN 4      

File audioFile;
bool audioAvailable = false;
uint32_t audioSampleRate = 22050; 
uint16_t audioChannels = 1;
uint16_t audioBitsPerSample = 16;

#define AUDIO_BUFFER_SIZE 16384 
volatile uint8_t audioBuffer[AUDIO_BUFFER_SIZE];
volatile uint32_t bufHead = 0;
volatile uint32_t bufTail = 0;

volatile uint8_t globalVolume = 255; 

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t audioTaskHandle = NULL;
volatile bool audioPlaying = false;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (bufHead != bufTail) {
    uint8_t sample = audioBuffer[bufTail];
    bufTail = (bufTail + 1) % AUDIO_BUFFER_SIZE;
    
    // Apply volume
    uint16_t scaled = (uint16_t)sample * globalVolume;
    uint8_t pwmValue = scaled >> 8;

    ledcWrite(0, pwmValue);
  } else {
    ledcWrite(0, 0); 
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void audioTask(void *parameter) {
  Serial.println("[AUDIO] Feeder task started on core 0");
  
  ledcSetup(0, 40000, 8); 
  ledcAttachPin(SPEAKER_PIN, 0);
  ledcWrite(0, 0);
  
  bufHead = 0;
  bufTail = 0;

  if (timer == NULL) {
    timer = timerBegin(0, 80, true); 
    timerAttachInterrupt(timer, &onTimer, true);
  }
  
  uint32_t alarmValue = 1000000 / audioSampleRate;
  timerAlarmWrite(timer, alarmValue, true);
  timerAlarmEnable(timer);

  Serial.printf("[AUDIO] Timer set for %d Hz (alarm: %d)\n", audioSampleRate, alarmValue);

  const int maxSamples = 256;
  uint8_t rawBuf[maxSamples * 4]; 

  while (audioPlaying) {
    static uint32_t lastVolCheck = 0;
    if (millis() - lastVolCheck > 100) {
        lastVolCheck = millis();
        int val = analogRead(POT_PIN);
        globalVolume = map(val, 0, 4095, 0, 255);
    }

    if (!audioFile || !audioFile.available()) {
       vTaskDelay(10 / portTICK_PERIOD_MS);
       continue;
    }

    uint32_t tail = bufTail; 
    uint32_t head = bufHead;
    uint32_t count = (head >= tail) ? (head - tail) : (AUDIO_BUFFER_SIZE - tail + head);
    uint32_t freeSpace = AUDIO_BUFFER_SIZE - 1 - count;

    if (freeSpace >= maxSamples) {
      int bytesPerSampleRaw = (audioBitsPerSample / 8) * audioChannels;
      int bytesToRead = maxSamples * bytesPerSampleRaw;
      
      int bytesRead = audioFile.read(rawBuf, bytesToRead);
      if (bytesRead > 0) {
        int samplesRead = bytesRead / bytesPerSampleRaw;
        
        portENTER_CRITICAL(&timerMux);
        for(int i=0; i<samplesRead; i++) {
           int16_t sample = 0;
           int baseIdx = i * bytesPerSampleRaw;
           
           if (audioChannels == 2) {
             int16_t left = rawBuf[baseIdx] | (rawBuf[baseIdx+1] << 8);
             int16_t right = rawBuf[baseIdx+2] | (rawBuf[baseIdx+3] << 8);
             sample = (left / 2) + (right / 2);
           } else {
             sample = rawBuf[baseIdx] | (rawBuf[baseIdx+1] << 8);
           }
           
           uint8_t pwmValue = (uint8_t)((sample + 32768) >> 8);
           
           audioBuffer[bufHead] = pwmValue;
           bufHead = (bufHead + 1) % AUDIO_BUFFER_SIZE;
        }
        portEXIT_CRITICAL(&timerMux);
      }
    } else {
       vTaskDelay(2 / portTICK_PERIOD_MS); 
    }
  }
  
  if (timer) {
    timerAlarmDisable(timer);
  }
  ledcWrite(0, 0);
  Serial.println("[AUDIO] Feeder task stopped");
  vTaskDelete(NULL);
}

void initAudio() {
    // Basic setup if any? Pin modes?
    pinMode(POT_PIN, INPUT);
}

bool loadAudio(fs::FS &fs, const char* path) {
    Serial.println("\n=== Loading Audio File ===");
    audioFile = fs.open(path);
    if (audioFile && audioFile.size() > 44) {
        uint8_t header[44];
        audioFile.read(header, 44);

        if (header[0] == 'R' && header[1] == 'I' && header[2] == 'F' && header[3] == 'F') {
            audioChannels = header[22] | (header[23] << 8);
            audioSampleRate = header[24] | (header[25] << 8) | (header[26] << 16) | (header[27] << 24);
            audioBitsPerSample = header[34] | (header[35] << 8);
            
            // audioBytesPerSample = audioBitsPerSample / 8;
            // audioDataSize = audioFile.size() - 44;

            audioAvailable = true;

            Serial.println("✓ Audio file loaded!");
            Serial.printf("  Channels: %d\n", audioChannels);
            Serial.printf("  Sample Rate: %d Hz\n", audioSampleRate);
            Serial.printf("  Bits/Sample: %d\n", audioBitsPerSample);
            return true;
        } else {
            Serial.println("✗ Invalid WAV format");
            audioFile.close();
            return false;
        }
    }
    Serial.println("✗ Audio file not found");
    return false;
}

void startAudioPlayback() {
  if (!audioAvailable) return;
  if (audioPlaying) return;

  audioPlaying = true;
  xTaskCreatePinnedToCore(audioTask, "AudioTask", 4096, NULL, 2, &audioTaskHandle, 0);
}

void stopAudioPlayback() {
  audioPlaying = false;
  if (timer) {
      timerAlarmDisable(timer);
  }
  if (audioTaskHandle != NULL) {
    vTaskDelay(50 / portTICK_PERIOD_MS);
    audioTaskHandle = NULL;
  }
  ledcWrite(0, 128); 
}
