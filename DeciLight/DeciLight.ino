#include "esp32-i2s-slm/filters.h"
#include "esp32-i2s-slm/i2s_mic.h"
#include "approx.h" // fast log10f and sincosf approximation
#include <cmath>

static constexpr unsigned SAMPLE_RATE_HZ = 48000; // Hz, fixed to design of IIR filters. Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.
static constexpr unsigned SAMPLE_COUNT = 512;     // ~10ms sample time, must be power-of-two
float samples[SAMPLE_COUNT];                      // Raw microphone sample storage

// NOTE: Some microphones require at least a DC-Blocker filter
#define MIC_EQUALIZER INMP441                   // See below for defined IIR filters or set to 'None' to disable
static constexpr float MIC_OFFSET_DB = 3.0103f; // Default offset (sine-wave RMS vs. dBFS). Modify this value for linear calibration
// Customize these values from microphone datasheet
static constexpr int MIC_SENSITIVITY = -26.0f; // dBFS value expected at MIC_REF_DB (Sensitivity value from datasheet)
static constexpr int MIC_REF_DB = 94.0f;       // Value at which point sensitivity is specified in datasheet (dB)
static constexpr int MIC_OVERLOAD_DB = 120.0f; // dB - Acoustic overload point / Maximum Acoustic Input
static constexpr int MIC_NOISE_DB = 33.0f;     // dB - Noise floor
static constexpr unsigned MIC_BITS = 24;       // valid number of bits in I2S data

constexpr float MIC_REF_AMPL = powf(10.0f, float(MIC_SENSITIVITY) / 20.0f) * ((1 << (MIC_BITS - 1)) - 1); // Microphone reference amplitude value

// I2S pins - Can be routed to almost any (unused) ESP32 pin.
// SD can be any pin, inlcuding input only pins (36-39).
// SCK (i.e. BCLK) and WS (i.e. L/R CLK) must be output capable pins
#define I2S_WS 15
#define I2S_SCK 14
#define I2S_SD 32

// Convert microphone amplitude to dB values
float MicAmplitudeToDb(float v) {
  return MIC_OFFSET_DB + MIC_REF_DB + 20.0f * log10f_fast(v / MIC_REF_AMPL);
}

auto mic = Microphone_I2S<SAMPLE_COUNT, 33, 32, 34, I2S_NUM_0, MIC_BITS, false, SAMPLE_RATE_HZ>(MIC_EQUALIZER);

// ------------------------------------------------------------------------------------------

#include "normalization.h"

static constexpr unsigned NR_OF_BANDS = 32;
static constexpr unsigned MAX_ANALYSIS_FREQUENCY_HZ = 4000;

auto normalization = Normalization<SAMPLE_COUNT, MIC_NOISE_DB, MIC_OVERLOAD_DB, MAX_ANALYSIS_FREQUENCY_HZ, SAMPLE_RATE_HZ>(MicAmplitudeToDb);

// ------------------------------------------------------------------------------------------
// DEFINE FASTLED LIBRARY HERE
// ------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial.println("Running setup");

  // ------------------------------------------------------------------------------------------
  // INITIALIZE FASTLED HERE
  // ------------------------------------------------------------------------------------------

  // initialize microphone
  mic.begin();
  Serial.println("Starting sampling from mic");
  mic.startSampling();
}

#define PRINT_LOOP_TIME
#ifdef PRINT_LOOP_TIME
  long lastLoopTime = 0;
#endif

void loop() {
  Serial.println("Starting loop");
  // Get samples from other ESP32 core that receives the I2S audio data
  while (xQueueReceive(mic.sampleQueue(), &samples, portMAX_DELAY)) {
    // apply A-Weighting filter for perceptive loudness. See: https://www.noisemeters.com/help/faq/frequency-weighting/
    A_weighting.applyFilters(samples, samples, SAMPLE_COUNT);
    A_weighting.applyGain(samples, samples, SAMPLE_COUNT);

    // ------------------------------------------------------------------------------------------
    // RUN FASTLED STUFF HERE
    // ------------------------------------------------------------------------------------------

    Serial.printf("Current normalization value: %.1f\n", normalization);
    Serial.printf("Current MicAmplitudeToDb value: %.1f\n", MicAmplitudeToDb);

    #ifdef PRINT_LOOP_TIME
      auto currentLoopTime = millis();
      Serial.print(currentLoopTime - lastLoopTime);
      Serial.println(" ms");
      lastLoopTime = currentLoopTime;
    #endif

  }
}