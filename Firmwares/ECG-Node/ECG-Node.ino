  // This program is free software: you can redistribute it and/or modify
  // it under the terms of the GNU General Public License as published by
  // the Free Software Foundation, either version 3 of the License, or
  // (at your option) any later version.
  //
  // This program is distributed in the hope that it will be useful,
  // but WITHOUT ANY WARRANTY; without even the implied warranty of
  // MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  // GNU General Public License for more details.
  //
  // You should have received a copy of the GNU General Public License
  // along with this program. If not, see <https://www.gnu.org/licenses/>.

  // Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
  // Copyright (c) 2025 Deepak Khatri - deepak@upsidedownlabs.tech
  // Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech

  // At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
  // Our mission is to make neuroscience affordable and accessible for everyone.
  // By supporting us with your purchase, you help spread innovation and open science.
  // Thank you for being part of this journey with us!

  // Requires Adafruit NeoPixel library (install via Arduino Library Manager)

  #include <Arduino.h>
  #include "hal/efuse_hal.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"

  #include <esp_now.h>
  #include <WiFi.h>
  #include "esp_timer.h"

  #include <Adafruit_NeoPixel.h>

  #define PIXEL_PIN 15
  // How many NeoPixels are attached to the Arduino?
  #define PIXEL_COUNT 6

  #define TIMER_FREQ 1000000
  #define FS 125
  #define PERIOD_US (TIMER_FREQ / FS)

  #define BAUD_RATE 230400

  #define ECG_HIST 240

  #define MWI_WIN 20
  #define REFRACT_SAMPLES 25
  #define LEARN_SAMPLES (FS * 5)

  #define R_SEARCH_BACK 22
  #define R_SEARCH_FWD 3

  #define TW_MIN 25
  #define TW_MAX 45
  #define TW_SLOPE_RATIO 0.5f

  #define RECOVER_MIN_GAP_SAMPLES (FS / 2)
  #define NO_QRS_ABS_SAMPLES FS
  #define MW_BASE_ALPHA 0.01f
  #define DECAY_SPKI 0.50f

  #define NUM_CH 3

  // Declare NeoPixel strip object:
  Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

  // LUT for 1S LiPo (Voltage in ascending order)
  const float voltageLUT[] = {
    3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.80, 3.82, 
    3.84, 3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.20
  };

  const int percentLUT[] = {
    0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 
    50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100
  };

  const int lutSize = sizeof(voltageLUT) / sizeof(voltageLUT[0]);
  volatile float g_battPct = 0.0f;

  // ---- profiling (updated by samplingTask, printed by loop) ----
  volatile uint32_t prof_winSamples = 0;
  volatile uint32_t prof_winWakes = 0;
  volatile uint32_t prof_winMaxTicksPerWake = 0;

  volatile int32_t  prof_lateMinUs =  2147483647;
  volatile int32_t  prof_lateMaxUs = -2147483647;
  volatile uint32_t prof_lateOver500 = 0;
  volatile uint32_t prof_lateOver1000 = 0;
  volatile uint32_t prof_missed = 0;

  volatile uint32_t prof_procSumUs = 0;
  volatile uint32_t prof_procMaxUs = 0;

  volatile int64_t  prof_schedT0Us = 0;
  volatile bool     prof_schedInit = false;

  static portMUX_TYPE profMux = portMUX_INITIALIZER_UNLOCKED;

  // Linear interpolation function
  float interpolatePercentage(float voltage) {
    // Handle out-of-range voltages
    if (voltage <= voltageLUT[0]) return 0;
    if (voltage >= voltageLUT[lutSize - 1]) return 100;

    // Find the nearest LUT entries
    int i = 0;
    while (i < lutSize - 1 && voltage > voltageLUT[i + 1]) i++;

    // Interpolate
    float v1 = voltageLUT[i], v2 = voltageLUT[i + 1];
    int p1 = percentLUT[i], p2 = percentLUT[i + 1];
    return p1 + (voltage - v1) * (p2 - p1) / (v2 - v1);
  } 

  // S3 Geek MAC Address
  uint8_t masterMAC[] = {0x28, 0x37, 0x2F, 0xF8, 0x1D, 0x24};

  static const uint8_t ECG_PINS[NUM_CH] = { A0, A1, A2 };

  static inline float f_abs(float x) { return x < 0 ? -x : x; }
  static inline uint8_t clamp_u8(int v) { return (v < 0) ? 0 : (v > 255 ? 255 : (uint8_t)v); }

  const uint16_t BEAT_FLASH_MS = 50;
  uint32_t beatOffAtMs[NUM_CH] = {0};

  // Band-Stop Butterworth IIR digital filter
  // Sampling rate: 125.0 Hz, frequency: [49.5, 50.5] Hz
  // Filter is order 2, implemented as second-order sections (biquads)
  // Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
  class Notch {
  private:
    struct BiquadState { float z1 = 0, z2 = 0; };
    BiquadState state0, state1;

  public:
    float process(float input) {
      float output = input;

      float x0 = output - (1.56858163f * state0.z1) - (0.96424138f * state0.z2);
      output = 0.96508099f * x0 + 1.56202714f * state0.z1 + 0.96508099f * state0.z2;
      state0.z2 = state0.z1; state0.z1 = x0;

      float x1 = output - (1.61100358f * state1.z1) - (0.96592171f * state1.z2);
      output = 1.00000000f * x1 + 1.61854514f * state1.z1 + 1.00000000f * state1.z2;
      state1.z2 = state1.z1; state1.z1 = x1;

      return output;
    }

    void reset() { state0.z1 = state0.z2 = 0; state1.z1 = state1.z2 = 0; }
  };


  // Band-Pass Butterworth IIR digital filter
  // Sampling rate: 125.0 Hz, frequency: [0.5, 30.0] Hz
  // Filter is order 4, implemented as second-order sections (biquads)
  // Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
  class ECG {
  private:
    struct BiquadState { float z1 = 0, z2 = 0; };
    BiquadState state0, state1, state2, state3;

  public:
    float process(float input) {
      float output = input;

      float x0 = output - (-0.08958458f * state0.z1) - (0.04351142f * state0.z2);
      output = 0.07842289f * x0 + 0.15684578f * state0.z1 + 0.07842289f * state0.z2;
      state0.z2 = state0.z1; state0.z1 = x0;

      float x1 = output - (-0.09690708f * state1.z1) - (0.45654448f * state1.z2);
      output = 1.00000000f * x1 + 2.00000000f * state1.z1 + 1.00000000f * state1.z2;
      state1.z2 = state1.z1; state1.z1 = x1;

      float x2 = output - (-1.95312373f * state2.z1) - (0.95377004f * state2.z2);
      output = 1.00000000f * x2 + -2.00000000f * state2.z1 + 1.00000000f * state2.z2;
      state2.z2 = state2.z1; state2.z1 = x2;

      float x3 = output - (-1.98068075f * state3.z1) - (0.98131119f * state3.z2);
      output = 1.00000000f * x3 + -2.00000000f * state3.z1 + 1.00000000f * state3.z2;
      state3.z2 = state3.z1; state3.z1 = x3;

      return output;
    }

    void reset() {
      state0.z1 = state0.z2 = 0;
      state1.z1 = state1.z2 = 0;
      state2.z1 = state2.z2 = 0;
      state3.z1 = state3.z2 = 0;
    }
  };

  class EcgDetector {
  public:
    Notch notchF;
    ECG ecgF;

    float ecgHist[ECG_HIST];
    float slopeHist[ECG_HIST];
    uint32_t ecgTime[ECG_HIST];
    uint16_t ecgW = 0;

    float dBuf[5] = {0};
    uint8_t dW = 0;

    float mwiBuf[MWI_WIN] = {0};
    uint8_t mwiW = 0;
    float mwiSum = 0;

    float m0 = 0, m1 = 0, m2 = 0;
    uint32_t t0 = 0, t1 = 0, t2 = 0;

    float SPKI = 0.0f, NPKI = 0.0f;
    float TH1 = 0.0f, TH2 = 0.0f;

    uint32_t lastQRS = 0;
    float lastQRSSlope = 0.0f;

    uint32_t rrBuf[8] = {0};
    uint8_t rrW = 0, rrN = 0;
    float rrAvg = (float)FS;

    float sbPeakVal = 0.0f;
    uint32_t sbPeakTime = 0;

    uint32_t learnCount = 0;
    float learnMax = 0.0f;
    float learnSum = 0.0f;

    bool mwBaseInit = false;
    float mwBase = 0.0f;
    uint32_t lastRecover = 0;

    // BPM
    uint32_t lastRTime = 0;
    uint16_t rrHist[8] = {0};
    uint8_t rrHW = 0, rrHN = 0;
    float bpm = 0.0f;

    volatile bool beatEvent = false;
    uint32_t lastBeatN = 0;     // sample index when beat occurred (optional)

    bool popBeatEvent() {
      if (!beatEvent) return false;
      beatEvent = false;
      return true;
    }

    void reset() {
      beatEvent = false;
      lastBeatN = 0;

      notchF.reset();
      ecgF.reset();

      ecgW = 0;
      dW = 0;

      mwiW = 0;
      mwiSum = 0;

      m0 = m1 = m2 = 0;
      t0 = t1 = t2 = 0;

      SPKI = NPKI = TH1 = TH2 = 0;

      lastQRS = 0;
      lastQRSSlope = 0;

      rrW = rrN = 0;
      rrAvg = (float)FS;

      sbPeakVal = 0;
      sbPeakTime = 0;

      learnCount = 0;
      learnMax = 0;
      learnSum = 0;

      mwBaseInit = false;
      mwBase = 0;
      lastRecover = 0;

      lastRTime = 0;
      rrHW = rrHN = 0;
      bpm = 0;

      for (uint16_t i = 0; i < ECG_HIST; i++) {
        ecgHist[i] = 0;
        slopeHist[i] = 0;
        ecgTime[i] = 0;
      }
      for (uint8_t i = 0; i < 5; i++) dBuf[i] = 0;
      for (uint8_t i = 0; i < MWI_WIN; i++) mwiBuf[i] = 0;
      for (uint8_t i = 0; i < 8; i++) { rrBuf[i] = 0; rrHist[i] = 0; }
    }

    void processSample(uint32_t nNow, uint16_t adc) {
      float x = (float)((int32_t)adc - 2048);
      x = notchF.process(x);
      float y = ecgF.process(x);

      float d = derivative5(y);
      float slope = f_abs(d);
      float s2 = d * d;
      float mw = mwi(s2);

      ecgHist[ecgW] = y;
      slopeHist[ecgW] = slope;
      ecgTime[ecgW] = nNow;
      ecgW = (ecgW + 1) % ECG_HIST;

      m0 = m1; t0 = t1;
      m1 = m2; t1 = t2;
      m2 = mw; t2 = nNow;

      if (nNow >= LEARN_SAMPLES) {
        if (!mwBaseInit) { mwBase = mw; mwBaseInit = true; }
        if (mw < TH1) mwBase = (1.0f - MW_BASE_ALPHA) * mwBase + MW_BASE_ALPHA * mw;
        watchdogRecoverIfBlind(nNow);
      }

      if (nNow >= 2 && (m1 > m0) && (m1 >= m2)) handleMWIPeak(nNow, m1, t1);
      searchbackIfNeeded(nNow);
    }

    uint8_t bpmU8Rounded() const {
      int v = (int)lroundf(bpm);
      return clamp_u8(v);
    }

  private:
    float derivative5(float x) {
      dBuf[dW] = x;
      dW = (dW + 1) % 5;
      uint8_t i = dW;
      float xn2 = dBuf[(i + 3) % 5];
      float xn1 = dBuf[(i + 4) % 5];
      float xp1 = dBuf[(i + 1) % 5];
      float xp2 = dBuf[(i + 2) % 5];
      return (-xn2 - 2.0f * xn1 + 2.0f * xp1 + xp2) / 8.0f;
    }

    float mwi(float x) {
      mwiSum -= mwiBuf[mwiW];
      mwiBuf[mwiW] = x;
      mwiSum += x;
      mwiW = (mwiW + 1) % MWI_WIN;
      return mwiSum / (float)MWI_WIN;
    }

    void updateThresholds() {
      TH1 = NPKI + 0.25f * (SPKI - NPKI);
      TH2 = 0.40f * TH1;
    }

    void rrUpdate(uint32_t rr) {
      rrBuf[rrW] = rr;
      rrW = (rrW + 1) & 7;
      if (rrN < 8) rrN++;
      uint32_t s = 0;
      for (uint8_t i = 0; i < rrN; i++) s += rrBuf[i];
      rrAvg = (rrN > 0) ? (float)s / (float)rrN : (float)FS;
    }

    void bpmUpdate(uint32_t rTime) {
      if (lastRTime == 0) { lastRTime = rTime; return; }
      uint32_t rrSamp = rTime - lastRTime;
      lastRTime = rTime;

      if (rrSamp < 10 || rrSamp > (uint32_t)(FS * 3)) return;

      rrHist[rrHW] = (uint16_t)rrSamp;
      rrHW = (rrHW + 1) & 7;
      if (rrHN < 8) rrHN++;

      uint32_t sum = 0;
      for (uint8_t i = 0; i < rrHN; i++) sum += rrHist[i];
      float rrMean = (float)sum / (float)rrHN;

      bpm = (60.0f * (float)FS) / rrMean; // BPM from RR [web:176]
    }

    float slopeAround(uint32_t timeCenter, uint8_t halfWin) {
      float best = 0.0f;
      for (uint16_t k = 0; k < ECG_HIST; k++) {
        uint16_t idx = (ecgW + ECG_HIST - 1 - k) % ECG_HIST;
        int32_t dt = (int32_t)ecgTime[idx] - (int32_t)timeCenter;
        if (dt > (int32_t)halfWin) continue;
        if (dt < -(int32_t)halfWin) break;
        float v = slopeHist[idx];
        if (v > best) best = v;
      }
      return best;
    }

    bool acceptQRS(uint32_t peakTime) {
      if (lastQRS != 0) {
        uint32_t dt = peakTime - lastQRS;
        if (dt < REFRACT_SAMPLES) return false;

        if (dt >= TW_MIN && dt <= TW_MAX) {
          float slopeNow = slopeAround(peakTime, 2);
          if (lastQRSSlope > 0.0f && slopeNow < (TW_SLOPE_RATIO * lastQRSSlope)) return false;
        }

        if (rrN >= 2) {
          float dtrr = (float)dt;
          if (dtrr < 0.30f * rrAvg) return false;
        }
      }
      return true;
    }

    bool findRpeak(uint32_t qrsTime, uint32_t& rTimeOut) {
      float bestAbs = -1.0f;
      float bestSlope = -1.0f;
      uint32_t bestTime = 0;

      for (uint16_t k = 0; k < ECG_HIST; k++) {
        uint16_t idx = (ecgW + ECG_HIST - 1 - k) % ECG_HIST;
        int32_t dt = (int32_t)ecgTime[idx] - (int32_t)qrsTime;
        if (dt > (int32_t)R_SEARCH_FWD) continue;
        if (dt < -(int32_t)R_SEARCH_BACK) break;

        float v = ecgHist[idx];
        float av = f_abs(v);
        float s = slopeHist[idx];

        if (av > bestAbs || (av == bestAbs && s > bestSlope)) {
          bestAbs = av;
          bestSlope = s;
          bestTime = ecgTime[idx];
        }
      }

      if (bestTime == 0) return false;
      rTimeOut = bestTime;
      return true;
    }

    void watchdogRecoverIfBlind(uint32_t nNow) {
      if (nNow < LEARN_SAMPLES) return;
      if (lastQRS == 0) return;

      uint32_t blindLimit = (uint32_t)(1.5f * rrAvg);
      if (blindLimit < NO_QRS_ABS_SAMPLES) blindLimit = NO_QRS_ABS_SAMPLES;

      uint32_t sinceQRS = nNow - lastQRS;
      if (sinceQRS <= blindLimit) return;

      if ((nNow - lastRecover) < RECOVER_MIN_GAP_SAMPLES) return;
      lastRecover = nNow;

      SPKI *= DECAY_SPKI;
      NPKI = 0.90f * NPKI + 0.10f * mwBase;

      if (NPKI < 1e-6f) NPKI = 1e-6f;
      if (SPKI < NPKI) SPKI = NPKI;

      updateThresholds();
      sbPeakVal = 0.0f;
      sbPeakTime = 0;
    }

    void handleMWIPeak(uint32_t nNow, float peakVal, uint32_t peakTime) {
      if (nNow < LEARN_SAMPLES) {
        learnCount++;
        learnSum += peakVal;
        if (peakVal > learnMax) learnMax = peakVal;
        if (nNow == LEARN_SAMPLES - 1) {
          SPKI = learnMax;
          NPKI = (learnCount > 0) ? (learnSum / (float)learnCount) : (0.1f * learnMax);
          updateThresholds();
        }
        return;
      }

      bool isQRS = (peakVal >= TH1) && acceptQRS(peakTime);

      if (!isQRS) {
        NPKI = 0.125f * peakVal + 0.875f * NPKI;
        updateThresholds();
        if (peakVal > TH2 && peakVal > sbPeakVal) { sbPeakVal = peakVal; sbPeakTime = peakTime; }
        return;
      }

      uint32_t rr = (lastQRS == 0) ? (uint32_t)rrAvg : (peakTime - lastQRS);
      lastQRS = peakTime;
      rrUpdate(rr);

      lastQRSSlope = slopeAround(peakTime, 2);

      SPKI = 0.125f * peakVal + 0.875f * SPKI;
      updateThresholds();

      sbPeakVal = 0.0f;
      sbPeakTime = 0;

      uint32_t rT = 0;
      if (!findRpeak(peakTime, rT)) rT = peakTime;

      beatEvent = true;     // <-- heartbeat event
      lastBeatN = rT;

      bpmUpdate(rT);
    }

    void searchbackIfNeeded(uint32_t nNow) {
      if (nNow < LEARN_SAMPLES) return;
      if (lastQRS == 0) return;

      float since = (float)(nNow - lastQRS);
      if (since <= 1.66f * rrAvg) return;

      if (sbPeakTime != 0 && sbPeakVal >= TH2 && acceptQRS(sbPeakTime)) {
        uint32_t rr = sbPeakTime - lastQRS;
        lastQRS = sbPeakTime;
        rrUpdate(rr);

        lastQRSSlope = slopeAround(sbPeakTime, 2);

        SPKI = 0.125f * sbPeakVal + 0.875f * SPKI;
        updateThresholds();

        uint32_t rT = 0;
        if (!findRpeak(sbPeakTime, rT)) rT = sbPeakTime;

        beatEvent = true;
        lastBeatN = rT;

        bpmUpdate(rT);

        sbPeakVal = 0.0f;
        sbPeakTime = 0;
      } else {
        sbPeakVal = 0.0f;
        sbPeakTime = 0;
        NPKI *= 0.95f;
        updateThresholds();
      }
    }
  };

  // --- Timer sampling ---
  hw_timer_t* timer_1 = nullptr;

  // FreeRTOS: ISR wakes this task (high priority)
  static TaskHandle_t samplingTaskHandle = nullptr;

  // Sample counter
  uint32_t chiprev = 0;
  volatile uint32_t nGlobal = 0;

  EcgDetector det[NUM_CH];

  // ===== ESP-NOW receive callback =====
  void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
    // respond only to poll 0xA5 from master
    if (len > 0 && data[0] == 0xA5 && memcmp(info->src_addr, masterMAC, 6) == 0) {
      uint8_t currentData[3];
      currentData[0] = det[0].bpmU8Rounded();
      currentData[1] = det[1].bpmU8Rounded();
      currentData[2] = det[2].bpmU8Rounded();
      esp_err_t err = esp_now_send(masterMAC, currentData, 3);
      if (err != ESP_OK) 
      {
        Serial.printf("esp_now_send failed: %d\n", err);
      }
    }
  }

  void samplingTask(void *arg) 
  {
    for (;;) {
      // Wait until the ISR notifies us.
      // Returns how many "ticks" happened since last time (counting semaphore).
      uint32_t ticks = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      prof_winWakes++;
      if (ticks > prof_winMaxTicksPerWake) prof_winMaxTicksPerWake = ticks;

      while (ticks--) {
        int64_t tStartUs = esp_timer_get_time();

        // Initialize schedule anchor once, aligned to sample index
        if (!prof_schedInit) {
          prof_schedInit = true;
          prof_schedT0Us = tStartUs - (int64_t)nGlobal * (int64_t)PERIOD_US;
        }

        // lateness vs ideal schedule
        int64_t expectedUs = prof_schedT0Us + (int64_t)nGlobal * (int64_t)PERIOD_US;
        int32_t lateUs = (int32_t)(tStartUs - expectedUs);

        if (lateUs < prof_lateMinUs) prof_lateMinUs = lateUs;
        if (lateUs > prof_lateMaxUs) prof_lateMaxUs = lateUs;

        int32_t a = (lateUs < 0) ? -lateUs : lateUs;
        if (a > 500)  prof_lateOver500++;
        if (a > 1000) prof_lateOver1000++;
        if (lateUs > (int32_t)PERIOD_US) prof_missed++;

        // 1) Sample ADC (do it in task, not ISR)
        uint16_t adc[NUM_CH];
        for (int i = 0; i < NUM_CH; i++) {
          adc[i] = analogRead(ECG_PINS[i]);
          if (chiprev == 1) adc[i] = map(adc[i], 0, 3249, 0, 4095);
        }

        static uint32_t lastBattN_task = 0;
        if ((uint32_t)(nGlobal - lastBattN_task) >= (uint32_t)FS) {
          lastBattN_task = nGlobal;

          int analogValue = analogRead(A6);
          float voltage = (analogValue / 1000.0f) * 2.0f;
          voltage += 0.022f;
          g_battPct = interpolatePercentage(voltage);
        }

        // 2) Process sample
        for (int i = 0; i < NUM_CH; i++) det[i].processSample(nGlobal, adc[i]);

        // 3) Advance sample index
        nGlobal++;

        uint32_t procUs = (uint32_t)(esp_timer_get_time() - tStartUs);
        prof_procSumUs += procUs;
        if (procUs > prof_procMaxUs) prof_procMaxUs = procUs;

        prof_winSamples++;
      }
    }
  }


  void IRAM_ATTR ADC_ISR() 
  {
    BaseType_t hpWoken = pdFALSE;

    // Increment the sampling task's notification count (counting semaphore)
    vTaskNotifyGiveFromISR(samplingTaskHandle, &hpWoken);

    if (hpWoken) {
      portYIELD_FROM_ISR();  // let the high-priority sampling task run immediately
    }
  }

  void setup() {
    Serial.begin(BAUD_RATE);
    delay(100);

    analogReadResolution(12);

    chiprev = efuse_hal_chip_revision();

    strip.begin();
    strip.clear();
    strip.show();

    // --- ESP-NOW init (WiFi STA) ---
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) 
    {
      Serial.println("esp_now_init failed");
      return;
    }

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, masterMAC, 6);
    if (esp_now_add_peer(&peer) != ESP_OK) 
    {
      Serial.println("esp_now_add_peer failed");
      return;
    }

    esp_now_register_recv_cb(OnDataRecv);

    // --- ECG init ---
    for (int i = 0; i < NUM_CH; i++) det[i].reset();

    // Create the sampling task BEFORE starting the timer ISR
    xTaskCreatePinnedToCore(
      samplingTask,
      "samplingTask",
      8192,                 // stack
      NULL,
      20,                   // high priority
      &samplingTaskHandle,
      0                     
    );

    // Now start the hardware timer that triggers the ISR
    timer_1 = timerBegin(TIMER_FREQ);
    timerAttachInterrupt(timer_1, &ADC_ISR);
    timerAlarm(timer_1, (uint64_t)PERIOD_US, true, 0);
  }

  void loop() 
  {

    static uint32_t lastUiMs = 0;
    static uint32_t lastPrintN = 0;
    static uint32_t lastBattN  = 0;
    static bool ledDirty = false;

    uint32_t nowMs = millis();

    // Run UI at ~100 Hz max (every 10 ms) so it stays low priority/lightweight.
    if ((uint32_t)(nowMs - lastUiMs) < 10) {
      vTaskDelay(1);   // yield
      return;
    }
    lastUiMs = nowMs;

    // --- Beat LED logic (safe: does NOT touch ADC) ---
    // Turn LED OFF when flash time ends
    for (int i = 0; i < NUM_CH; i++) {
      if (beatOffAtMs[i] != 0 && (int32_t)(nowMs - beatOffAtMs[i]) >= 0) {
        strip.setPixelColor(i + 3, 0, 0, 0);
        beatOffAtMs[i] = 0;
        ledDirty = true;
      }
    }

    // If a beat happened, flash red
    for (int i = 0; i < NUM_CH; i++) {
      if (det[i].popBeatEvent()) {
        strip.setPixelColor(i + 3, 20, 0, 0);
        beatOffAtMs[i] = nowMs + BEAT_FLASH_MS;
        ledDirty = true;
      }
    }

    // --- BPM printing (based on nGlobal updated in samplingTask) ---
    if ((uint32_t)(nGlobal - lastPrintN) >= (uint32_t)FS) {
      lastPrintN = nGlobal;

      Serial.print("BPM");
      for (int i = 0; i < NUM_CH; i++) {
        int currentBPM = (int)det[i].bpmU8Rounded();
        Serial.print((i + 1));
        Serial.print(":");
        Serial.print(currentBPM);
        if (i != NUM_CH - 1) Serial.print(" ");
      }
      Serial.println();
    }

    // --- Battery update + status LED ---

    if ((uint32_t)(nGlobal - lastBattN) >= (uint32_t)FS) {  // ~1 Hz (based on sample index)
      lastBattN = nGlobal;

      float batteryPercentage = g_battPct;
      if (batteryPercentage <= 10.0f) {
        strip.setPixelColor(0, 10, 0, 0);   // Red
      } else if (batteryPercentage <= 50.0f) {
        strip.setPixelColor(0, 15, 4, 0);   // Orange
      } else {
        strip.setPixelColor(0, 0, 10, 0);   // Green
      }
      ledDirty = true;
    }

    // Only push NeoPixel data if something changed
    if (ledDirty) {
      strip.show();
      ledDirty = false;
    }

    static uint32_t lastProfMs = 0;
    uint32_t nowMs2 = millis();

    if ((uint32_t)(nowMs2 - lastProfMs) >= 10000) {
      lastProfMs = nowMs2;

      // ---- snapshot + reset atomically (VERY short) ----
      uint32_t s, wakes, maxTicks, o500, o1000, missed, procSum, procMax;
      int32_t lateMin, lateMax;

      taskENTER_CRITICAL(&profMux);
      s       = prof_winSamples;
      wakes   = prof_winWakes;
      maxTicks= prof_winMaxTicksPerWake;
      lateMin = prof_lateMinUs;
      lateMax = prof_lateMaxUs;
      o500    = prof_lateOver500;
      o1000   = prof_lateOver1000;
      missed  = prof_missed;
      procSum = prof_procSumUs;
      procMax = prof_procMaxUs;

      // reset 10s window
      prof_winSamples = 0;
      prof_winWakes = 0;
      prof_winMaxTicksPerWake = 0;
      prof_lateMinUs =  2147483647;
      prof_lateMaxUs = -2147483647;
      prof_lateOver500 = 0;
      prof_lateOver1000 = 0;
      prof_missed = 0;
      prof_procSumUs = 0;
      prof_procMaxUs = 0;
      taskEXIT_CRITICAL(&profMux);
      // ---- end atomic section ----

      uint32_t procAvg = (s > 0) ? (procSum / s) : 0;

      // Print AFTER the critical section (do not block interrupts while printing)
      Serial.printf(
        "[PROF 10s] wakes=%u maxTicksWake=%u samples=%u | late[%d..%d]us >500=%u >1000=%u missed=%u | proc avg/max=%u/%u us\n",
        (unsigned)wakes,
        (unsigned)maxTicks,
        (unsigned)s,
        (int)lateMin, (int)lateMax,
        (unsigned)o500, (unsigned)o1000,
        (unsigned)missed,
        (unsigned)procAvg, (unsigned)procMax
      );
    }

    vTaskDelay(1); // always yield
  }

