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

#include <Arduino.h>
#include "hal/efuse_hal.h"

#define TIMER_FREQ 1000000
#define FS 125
#define PERIOD_US (TIMER_FREQ / FS)

#define BAUD_RATE 230400

#define NUM_CH 3
static const uint8_t ECG_PINS[NUM_CH] = { A0, A1, A2 };

#define OUT_DELAY 35
#define OUT_BUF (OUT_DELAY + 10)

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

// Band-Stop Butterworth IIR digital filter
// Sampling rate: 125.0 Hz, frequency: [49.5, 50.5] Hz
// Filter is order 2, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
class Notch {
private:
  struct BiquadState {
    float z1 = 0, z2 = 0;
  };
  BiquadState state0, state1;

public:
  float process(float input) {
    float output = input;

    float x0 = output - (1.56858163f * state0.z1) - (0.96424138f * state0.z2);
    output = 0.96508099f * x0 + 1.56202714f * state0.z1 + 0.96508099f * state0.z2;
    state0.z2 = state0.z1;
    state0.z1 = x0;

    float x1 = output - (1.61100358f * state1.z1) - (0.96592171f * state1.z2);
    output = 1.00000000f * x1 + 1.61854514f * state1.z1 + 1.00000000f * state1.z2;
    state1.z2 = state1.z1;
    state1.z1 = x1;

    return output;
  }

  void reset() {
    state0.z1 = state0.z2 = 0;
    state1.z1 = state1.z2 = 0;
  }
};

// Band-Pass Butterworth IIR digital filter
// Sampling rate: 125.0 Hz, frequency: [0.5, 30.0] Hz
// Filter is order 4, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
class ECG {
private:
  struct BiquadState {
    float z1 = 0, z2 = 0;
  };
  BiquadState state0, state1, state2, state3;

public:
  float process(float input) {
    float output = input;

    float x0 = output - (-0.08958458f * state0.z1) - (0.04351142f * state0.z2);
    output = 0.07842289f * x0 + 0.15684578f * state0.z1 + 0.07842289f * state0.z2;
    state0.z2 = state0.z1;
    state0.z1 = x0;

    float x1 = output - (-0.09690708f * state1.z1) - (0.45654448f * state1.z2);
    output = 1.00000000f * x1 + 2.00000000f * state1.z1 + 1.00000000f * state1.z2;
    state1.z2 = state1.z1;
    state1.z1 = x1;

    float x2 = output - (-1.95312373f * state2.z1) - (0.95377004f * state2.z2);
    output = 1.00000000f * x2 + -2.00000000f * state2.z1 + 1.00000000f * state2.z2;
    state2.z2 = state2.z1;
    state2.z1 = x2;

    float x3 = output - (-1.98068075f * state3.z1) - (0.98131119f * state3.z2);
    output = 1.00000000f * x3 + -2.00000000f * state3.z1 + 1.00000000f * state3.z2;
    state3.z2 = state3.z1;
    state3.z1 = x3;

    return output;
  }

  void reset() {
    state0.z1 = state0.z2 = 0;
    state1.z1 = state1.z2 = 0;
    state2.z1 = state2.z2 = 0;
    state3.z1 = state3.z2 = 0;
  }
};

// --- Timer sampling ---
hw_timer_t* timer_1 = nullptr;
volatile bool sampleReady = false;
void IRAM_ATTR ADC_ISR() {
  sampleReady = true;
}

uint32_t chiprev = efuse_hal_chip_revision();
uint32_t n = 0;

static inline float f_abs(float x) {
  return x < 0 ? -x : x;
}

// Per-channel filters
Notch notchF[NUM_CH];
ECG ecgF[NUM_CH];

// Per-channel histories
float ecgHist[NUM_CH][ECG_HIST];
float slopeHist[NUM_CH][ECG_HIST];
uint32_t ecgTime[NUM_CH][ECG_HIST];
uint16_t ecgW[NUM_CH] = { 0 };

// Per-channel derivative
float dBuf[NUM_CH][5] = { { 0 } };
uint8_t dW[NUM_CH] = { 0 };

// Per-channel MWI
float mwiBuf[NUM_CH][MWI_WIN] = { { 0 } };
uint8_t mwiW[NUM_CH] = { 0 };
float mwiSum[NUM_CH] = { 0 };

// Per-channel peak tracking on MWI
float m0[NUM_CH] = { 0 }, m1[NUM_CH] = { 0 }, m2[NUM_CH] = { 0 };
uint32_t t0[NUM_CH] = { 0 }, t1[NUM_CH] = { 0 }, t2[NUM_CH] = { 0 };

// Per-channel thresholds
float SPKI[NUM_CH] = { 0 }, NPKI[NUM_CH] = { 0 };
float TH1[NUM_CH] = { 0 }, TH2[NUM_CH] = { 0 };

// Per-channel QRS timing
uint32_t lastQRS[NUM_CH] = { 0 };
float lastQRSSlope[NUM_CH] = { 0 };

// Per-channel RR avg
uint32_t rrBuf[NUM_CH][8] = { { 0 } };
uint8_t rrW[NUM_CH] = { 0 }, rrN[NUM_CH] = { 0 };
float rrAvg[NUM_CH] = { (float)FS, (float)FS, (float)FS };

// Per-channel searchback peak
float sbPeakVal[NUM_CH] = { 0 };
uint32_t sbPeakTime[NUM_CH] = { 0 };

// Output delay buffers (for plotting)
float outSig[NUM_CH][OUT_BUF];
uint8_t outW[NUM_CH] = { 0 };
uint8_t markFlag[NUM_CH][OUT_BUF] = { { 0 } };

// Learning
uint32_t learnCount[NUM_CH] = { 0 };
float learnMax[NUM_CH] = { 0 };
float learnSum[NUM_CH] = { 0 };

// Watchdog baseline
bool mwBaseInit[NUM_CH] = { false, false, false };
float mwBase[NUM_CH] = { 0 };
uint32_t lastRecover[NUM_CH] = { 0 };

// BPM
uint32_t lastRTime[NUM_CH] = { 0 };
uint16_t rrHist[NUM_CH][8] = { { 0 } };
uint8_t rrHW[NUM_CH] = { 0 }, rrHN[NUM_CH] = { 0 };
float bpm[NUM_CH] = { 0 };

float derivative5(uint8_t ch, float x) {
  dBuf[ch][dW[ch]] = x;
  dW[ch] = (dW[ch] + 1) % 5;
  uint8_t i = dW[ch];
  float xn2 = dBuf[ch][(i + 3) % 5];
  float xn1 = dBuf[ch][(i + 4) % 5];
  float xp1 = dBuf[ch][(i + 1) % 5];
  float xp2 = dBuf[ch][(i + 2) % 5];
  return (-xn2 - 2.0f * xn1 + 2.0f * xp1 + xp2) / 8.0f;
}

float mwi(uint8_t ch, float x) {
  mwiSum[ch] -= mwiBuf[ch][mwiW[ch]];
  mwiBuf[ch][mwiW[ch]] = x;
  mwiSum[ch] += x;
  mwiW[ch] = (mwiW[ch] + 1) % MWI_WIN;
  return mwiSum[ch] / (float)MWI_WIN;
}

void rrUpdate(uint8_t ch, uint32_t rr) {
  rrBuf[ch][rrW[ch]] = rr;
  rrW[ch] = (rrW[ch] + 1) & 7;
  if (rrN[ch] < 8) rrN[ch]++;
  uint32_t s = 0;
  for (uint8_t i = 0; i < rrN[ch]; i++) s += rrBuf[ch][i];
  rrAvg[ch] = (rrN[ch] > 0) ? (float)s / (float)rrN[ch] : (float)FS;
}

void bpmUpdate(uint8_t ch, uint32_t rTime) {
  if (lastRTime[ch] == 0) {
    lastRTime[ch] = rTime;
    return;
  }
  uint32_t rrSamp = rTime - lastRTime[ch];
  lastRTime[ch] = rTime;

  if (rrSamp < 10 || rrSamp > (uint32_t)(FS * 3)) return;

  rrHist[ch][rrHW[ch]] = (uint16_t)rrSamp;
  rrHW[ch] = (rrHW[ch] + 1) & 7;
  if (rrHN[ch] < 8) rrHN[ch]++;

  uint32_t sum = 0;
  for (uint8_t i = 0; i < rrHN[ch]; i++) sum += rrHist[ch][i];
  float rrMean = (float)sum / (float)rrHN[ch];

  bpm[ch] = (60.0f * (float)FS) / rrMean;
}

float slopeAround(uint8_t ch, uint32_t timeCenter, uint8_t halfWin) {
  float best = 0.0f;
  for (uint16_t k = 0; k < ECG_HIST; k++) {
    uint16_t idx = (ecgW[ch] + ECG_HIST - 1 - k) % ECG_HIST;
    int32_t dt = (int32_t)ecgTime[ch][idx] - (int32_t)timeCenter;
    if (dt > (int32_t)halfWin) continue;
    if (dt < -(int32_t)halfWin) break;
    float v = slopeHist[ch][idx];
    if (v > best) best = v;
  }
  return best;
}

bool findRpeak(uint8_t ch, uint32_t qrsTime, uint32_t& rTimeOut) {
  float bestAbs = -1.0f;
  float bestSlope = -1.0f;
  uint32_t bestTime = 0;

  for (uint16_t k = 0; k < ECG_HIST; k++) {
    uint16_t idx = (ecgW[ch] + ECG_HIST - 1 - k) % ECG_HIST;
    int32_t dt = (int32_t)ecgTime[ch][idx] - (int32_t)qrsTime;
    if (dt > (int32_t)R_SEARCH_FWD) continue;
    if (dt < -(int32_t)R_SEARCH_BACK) break;

    float v = ecgHist[ch][idx];
    float av = f_abs(v);
    float s = slopeHist[ch][idx];

    if (av > bestAbs || (av == bestAbs && s > bestSlope)) {
      bestAbs = av;
      bestSlope = s;
      bestTime = ecgTime[ch][idx];
    }
  }

  if (bestTime == 0) return false;
  rTimeOut = bestTime;
  return true;
}

void setMarkerAtTime(uint8_t ch, uint32_t t) {
  int32_t dt = (int32_t)n - (int32_t)t;
  if (dt < 0 || dt >= OUT_BUF) return;
  uint8_t idx = (outW[ch] + OUT_BUF - 1 - dt) % OUT_BUF;
  markFlag[ch][idx] = 1;
}

void updateThresholds(uint8_t ch) {
  TH1[ch] = NPKI[ch] + 0.25f * (SPKI[ch] - NPKI[ch]);
  TH2[ch] = 0.40f * TH1[ch];
}

bool acceptQRS(uint8_t ch, uint32_t peakTime) {
  if (lastQRS[ch] != 0) {
    uint32_t dt = peakTime - lastQRS[ch];
    if (dt < REFRACT_SAMPLES) return false;

    if (dt >= TW_MIN && dt <= TW_MAX) {
      float slopeNow = slopeAround(ch, peakTime, 2);
      if (lastQRSSlope[ch] > 0.0f && slopeNow < (TW_SLOPE_RATIO * lastQRSSlope[ch])) return false;
    }

    if (rrN[ch] >= 2) {
      float dtrr = (float)dt;
      if (dtrr < 0.30f * rrAvg[ch]) return false;
    }
  }
  return true;
}

void watchdogRecoverIfBlind(uint8_t ch) {
  if (n < LEARN_SAMPLES) return;
  if (lastQRS[ch] == 0) return;

  uint32_t blindLimit = (uint32_t)(1.5f * rrAvg[ch]);
  if (blindLimit < NO_QRS_ABS_SAMPLES) blindLimit = NO_QRS_ABS_SAMPLES;

  uint32_t sinceQRS = n - lastQRS[ch];
  if (sinceQRS <= blindLimit) return;

  if ((n - lastRecover[ch]) < RECOVER_MIN_GAP_SAMPLES) return;
  lastRecover[ch] = n;

  SPKI[ch] *= DECAY_SPKI;
  NPKI[ch] = 0.90f * NPKI[ch] + 0.10f * mwBase[ch];

  if (NPKI[ch] < 1e-6f) NPKI[ch] = 1e-6f;
  if (SPKI[ch] < NPKI[ch]) SPKI[ch] = NPKI[ch];

  updateThresholds(ch);
  sbPeakVal[ch] = 0.0f;
  sbPeakTime[ch] = 0;
}

void handleMWIPeak(uint8_t ch, float peakVal, uint32_t peakTime) {
  if (n < LEARN_SAMPLES) {
    learnCount[ch]++;
    learnSum[ch] += peakVal;
    if (peakVal > learnMax[ch]) learnMax[ch] = peakVal;
    if (n == LEARN_SAMPLES - 1) {
      SPKI[ch] = learnMax[ch];
      NPKI[ch] = (learnCount[ch] > 0) ? (learnSum[ch] / (float)learnCount[ch]) : (0.1f * learnMax[ch]);
      updateThresholds(ch);
    }
    return;
  }

  bool isQRS = (peakVal >= TH1[ch]) && acceptQRS(ch, peakTime);

  if (!isQRS) {
    NPKI[ch] = 0.125f * peakVal + 0.875f * NPKI[ch];
    updateThresholds(ch);
    if (peakVal > TH2[ch] && peakVal > sbPeakVal[ch]) {
      sbPeakVal[ch] = peakVal;
      sbPeakTime[ch] = peakTime;
    }
    return;
  }

  uint32_t rr = (lastQRS[ch] == 0) ? (uint32_t)rrAvg[ch] : (peakTime - lastQRS[ch]);
  lastQRS[ch] = peakTime;
  rrUpdate(ch, rr);

  lastQRSSlope[ch] = slopeAround(ch, peakTime, 2);

  SPKI[ch] = 0.125f * peakVal + 0.875f * SPKI[ch];
  updateThresholds(ch);

  sbPeakVal[ch] = 0.0f;
  sbPeakTime[ch] = 0;

  uint32_t rT = 0;
  if (findRpeak(ch, peakTime, rT)) {
    setMarkerAtTime(ch, rT);
    bpmUpdate(ch, rT);
  } else {
    setMarkerAtTime(ch, peakTime);
    bpmUpdate(ch, peakTime);
  }
}

void searchbackIfNeeded(uint8_t ch) {
  if (n < LEARN_SAMPLES) return;
  if (lastQRS[ch] == 0) return;

  float since = (float)(n - lastQRS[ch]);
  if (since <= 1.66f * rrAvg[ch]) return;

  if (sbPeakTime[ch] != 0 && sbPeakVal[ch] >= TH2[ch] && acceptQRS(ch, sbPeakTime[ch])) {
    uint32_t rr = sbPeakTime[ch] - lastQRS[ch];
    lastQRS[ch] = sbPeakTime[ch];
    rrUpdate(ch, rr);

    lastQRSSlope[ch] = slopeAround(ch, sbPeakTime[ch], 2);

    SPKI[ch] = 0.125f * sbPeakVal[ch] + 0.875f * SPKI[ch];
    updateThresholds(ch);

    uint32_t rT = 0;
    if (findRpeak(ch, sbPeakTime[ch], rT)) {
      setMarkerAtTime(ch, rT);
      bpmUpdate(ch, rT);
    } else {
      setMarkerAtTime(ch, sbPeakTime[ch]);
      bpmUpdate(ch, sbPeakTime[ch]);
    }

    sbPeakVal[ch] = 0.0f;
    sbPeakTime[ch] = 0;
  } else {
    sbPeakVal[ch] = 0.0f;
    sbPeakTime[ch] = 0;
    NPKI[ch] *= 0.95f;
    updateThresholds(ch);
  }
}

void pushOutput(uint8_t ch, float sig) {
  outSig[ch][outW[ch]] = sig;
  outW[ch] = (outW[ch] + 1) % OUT_BUF;
}

void setup() {
  Serial.begin(BAUD_RATE);
  delay(100);

  analogReadResolution(12);

  for (int ch = 0; ch < NUM_CH; ch++) {
    notchF[ch].reset();
    ecgF[ch].reset();

    for (uint16_t i = 0; i < ECG_HIST; i++) {
      ecgHist[ch][i] = 0;
      slopeHist[ch][i] = 0;
      ecgTime[ch][i] = 0;
    }
    for (uint16_t i = 0; i < OUT_BUF; i++) {
      outSig[ch][i] = 0;
      markFlag[ch][i] = 0;
    }
  }

  timer_1 = timerBegin(TIMER_FREQ);
  timerAttachInterrupt(timer_1, &ADC_ISR);
  timerAlarm(timer_1, (uint64_t)PERIOD_US, true, 0);
}

void loop() {
  if (!sampleReady) return;
  sampleReady = false;

  // Process all 3 channels for the same sample index n
  for (uint8_t ch = 0; ch < NUM_CH; ch++) {
    uint16_t adc = analogRead(ECG_PINS[ch]);
    if (chiprev == 1) adc = map(adc, 0, 3249, 0, 4095);

    float x = (float)((int32_t)adc - 2048);
    x = notchF[ch].process(x);
    float y = ecgF[ch].process(x);

    float d = derivative5(ch, y);
    float slope = f_abs(d);
    float s2 = d * d;
    float mw = mwi(ch, s2);

    ecgHist[ch][ecgW[ch]] = y;
    slopeHist[ch][ecgW[ch]] = slope;
    ecgTime[ch][ecgW[ch]] = n;
    ecgW[ch] = (ecgW[ch] + 1) % ECG_HIST;

    m0[ch] = m1[ch];
    t0[ch] = t1[ch];
    m1[ch] = m2[ch];
    t1[ch] = t2[ch];
    m2[ch] = mw;
    t2[ch] = n;

    if (n >= LEARN_SAMPLES) {
      if (!mwBaseInit[ch]) {
        mwBase[ch] = mw;
        mwBaseInit[ch] = true;
      }
      if (mw < TH1[ch]) mwBase[ch] = (1.0f - MW_BASE_ALPHA) * mwBase[ch] + MW_BASE_ALPHA * mw;
      watchdogRecoverIfBlind(ch);
    }

    if (n >= 2 && (m1[ch] > m0[ch]) && (m1[ch] >= m2[ch])) handleMWIPeak(ch, m1[ch], t1[ch]);
    searchbackIfNeeded(ch);

    // For plotting: offset each channel so they don't overlap too much
    float disp = y + 800.0f;
    pushOutput(ch, disp);
  }

  // Emit ONE line per sample: 3 traces (A0 A1 A2)
  // Serial Plotter reads values on one line as one timestep. [web:362][web:365]
  float yPlot[NUM_CH];
  uint8_t mPlot[NUM_CH];

  for (uint8_t ch = 0; ch < NUM_CH; ch++) {
    uint8_t rd = (outW[ch] + OUT_BUF - OUT_DELAY) % OUT_BUF;
    yPlot[ch] = outSig[ch][rd];
    mPlot[ch] = markFlag[ch][rd];
    markFlag[ch][rd] = 0;
  }

  int b0 = (int)lroundf(bpm[0]);
  int b1 = (int)lroundf(bpm[1]);
  int b2 = (int)lroundf(bpm[2]);

  // Plot ONLY the 3 ECG channels:
  // Serial.print(yPlot[0]); Serial.print(" ");
  // Serial.print(yPlot[1]); Serial.print(" ");
  // Serial.println(yPlot[2]);

  // Plot with peak markers and bpm:
  Serial.print(yPlot[0]);
  Serial.print(" ");
  Serial.print(mPlot[0] ? (yPlot[0] + 200.0f) : 0.0f);
  Serial.print(" ");
  Serial.print(yPlot[1]);
  Serial.print(" ");
  Serial.print(mPlot[1] ? (yPlot[1] + 200.0f) : 0.0f);
  Serial.print(" ");
  Serial.print(yPlot[2]);
  Serial.print(" ");
  Serial.print(mPlot[2] ? (yPlot[2] + 200.0f) : 0.0f);
  Serial.print(" ");
  Serial.print(b0);
  Serial.print(" ");
  Serial.print(b1);
  Serial.print(" ");
  Serial.println(b2);

  n++;
}
