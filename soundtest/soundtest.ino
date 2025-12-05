#include <Arduino.h>
#include "arduinoFFT.h"

// -------------------- Pin configuration --------------------
const uint8_t BUZZER_PIN       = A0;   // digital pin for passive buzzer
const uint8_t SOUND_SENSOR_PIN = A1;  // analog input from sound sensor

// -------------------- Buzzer config --------------------
uint16_t testFrequency = 630;  // Hz – starting square-wave frequency

// -------------------- FFT config --------------------
#define SAMPLES            128      // Must be a power of 2
#define SAMPLING_FREQUENCY 5000.0   // Hz

unsigned long sampling_period_us;

float vReal[SAMPLES];
float vImag[SAMPLES];
float magAvg[SAMPLES];  // for averaging magnitudes across FFT frames

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// -------------------- Setup --------------------
void setup() {
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SOUND_SENSOR_PIN, INPUT);

  // Compute sampling period once
  sampling_period_us = (unsigned long)(1000000.0 / SAMPLING_FREQUENCY);
}

// -------------------- ListenFreq with FFT averaging --------------------
float ListenFreq(uint8_t numAverages = 4) {
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

// -------------------- Main loop --------------------
void loop() {
  // Drive buzzer at the current testFrequency
  tone(BUZZER_PIN, testFrequency);

  // Measure frequency using FFT with averaging
  float measuredFreq = ListenFreq(4); // try 2–8, tradeoff between speed and stability

  // Exponential smoothing of the measured frequency
  static bool  firstSample   = true;
  static float smoothedFreq  = 0.0f;
  const float  alpha         = 0.3f;  // 0..1 (higher = more responsive, lower = smoother)

  if (firstSample) {
    smoothedFreq = measuredFreq;
    firstSample = false;
  } else {
    smoothedFreq = alpha * measuredFreq + (1.0f - alpha) * smoothedFreq;
  }

  // Print results
  Serial.print("Buzzer set to: ");
  Serial.print(testFrequency);
  Serial.print(" Hz, Measured: ");
  Serial.print(measuredFreq);
  Serial.print(" Hz, Smoothed: ");
  Serial.print(smoothedFreq);
  Serial.println(" Hz");

  // Sweep frequency but avoid aliasing (Nyquist = SAMPLING_FREQUENCY / 2)
  uint16_t nyquist = (uint16_t)(SAMPLING_FREQUENCY / 2.0);
  if (testFrequency < nyquist - 100) {   // keep a little margin
    testFrequency += 0;
  }

  delay(500);
}
