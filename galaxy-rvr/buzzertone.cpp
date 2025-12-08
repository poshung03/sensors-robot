#include "buzzertone.h"

#include <Arduino.h>

#include "arduinoFFT.h"
#include "NewTone.h"
// -------------------- FFT config --------------------
#define SAMPLES            128      // Must be a power of 2
#define SAMPLING_FREQUENCY 5000.0   // Hz



// -------------------- ListenFreq with FFT averaging --------------------
float ListenFreq(uint8_t numAverages = 4) {
  // -------------------- Pin configuration --------------------
  const uint8_t BUZZER_PIN       = A0;   // digital pin for passive buzzer
  const uint8_t SOUND_SENSOR_PIN = A1;  // analog input from sound sensor

  unsigned long sampling_period_us;

  float vReal[SAMPLES];
  float vImag[SAMPLES];
  float magAvg[SAMPLES];  // for averaging magnitudes across FFT frames

  ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SOUND_SENSOR_PIN, INPUT);
  sampling_period_us = (unsigned long)(1000000.0 / SAMPLING_FREQUENCY);

  // Clear accumulator
  for (int i = 0; i < SAMPLES; i++) {
    magAvg[i] = 0.0f;
  }

  for (uint8_t n = 0; n < numAverages; n++) {
    // ---- Collect SAMPLES data points ----
    for (int i = 0; i < SAMPLES; i++) {
      unsigned long t0 = micros();

      // Read analog value from microphone
      vReal[i] = analogRead(SOUND_SENSOR_PIN);
      vImag[i] = 0.0f;

      // Wait for the next sample, robust to micros() overflow
      while ((micros() - t0) < sampling_period_us) {
        // busy wait
      }
    }

    // ---- Remove DC offset (per frame) ----
    float mean = 0.0f;
    for (int i = 0; i < SAMPLES; i++) {
      mean += vReal[i];
    }
    mean /= SAMPLES;

    for (int i = 0; i < SAMPLES; i++) {
      vReal[i] -= mean;
    }

    // ---- FFT ----
    FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.complexToMagnitude(vReal, vImag, SAMPLES);

    // ---- Accumulate magnitudes ----
    for (int i = 0; i < SAMPLES; i++) {
      magAvg[i] += vReal[i];
    }
  }

  // ---- Average magnitudes across frames ----
  for (int i = 0; i < SAMPLES; i++) {
    magAvg[i] /= (float)numAverages;
  }

  // ---- Find the peak frequency from averaged spectrum ----
  float peak = FFT.majorPeak(magAvg, SAMPLES, SAMPLING_FREQUENCY);
  return peak;
}

void writefreq(uint8_t freqwrite){
  const uint8_t BUZZER_PIN = A0;   // digital pin for passive buzzer
  NewTone(BUZZER_PIN, freqwrite);
}