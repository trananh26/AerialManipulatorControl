/**
 * ESP32 DOIT DevKit V1 — 2-DOF arm joint ADC → USB serial for ROS arm_fb_bridge.
 *
 * Maps onboard 12-bit ADC through the measured 0/90/180/270° table
 * (ADC falls as angle rises).
 */

#include <Arduino.h>

#include "config.h"

static OutputMode g_mode = BOOT_OUTPUT_MODE;
static float g_ema_shoulder = 0.0f;
static float g_ema_elbow = 0.0f;
static bool g_ema_init = false;

static float clampf(float v, float lo, float hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

static uint16_t readAdcAveraged(int pin) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_OVERSAMPLE; ++i) {
    sum += static_cast<uint32_t>(analogRead(pin));
  }
  return static_cast<uint16_t>(sum / ADC_OVERSAMPLE);
}

/** Piecewise-linear ADC → deg. Table is ADC-descending (0° = high ADC). */
static float adcToDeg(float adc, const CalibPoint *table, uint8_t n) {
  if (n == 0) {
    return DEG_MIN;
  }
  if (adc >= table[0].adc) {
    return table[0].deg;
  }
  if (adc <= table[n - 1].adc) {
    return table[n - 1].deg;
  }
  for (uint8_t i = 0; i + 1 < n; ++i) {
    const float adc0 = table[i].adc;
    const float adc1 = table[i + 1].adc;
    if (adc <= adc0 && adc >= adc1) {
      const float t = (adc0 - adc) / (adc0 - adc1);
      return table[i].deg + t * (table[i + 1].deg - table[i].deg);
    }
  }
  return table[n - 1].deg;
}

static void applyEma(float shoulder_raw, float elbow_raw, float *shoulder_out,
                     float *elbow_out) {
  if (!g_ema_init) {
    g_ema_shoulder = shoulder_raw;
    g_ema_elbow = elbow_raw;
    g_ema_init = true;
  } else {
    g_ema_shoulder =
        EMA_ALPHA * shoulder_raw + (1.0f - EMA_ALPHA) * g_ema_shoulder;
    g_ema_elbow = EMA_ALPHA * elbow_raw + (1.0f - EMA_ALPHA) * g_ema_elbow;
  }
  *shoulder_out = g_ema_shoulder;
  *elbow_out = g_ema_elbow;
}

static void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == 'd' || c == 'D') {
      g_mode = OutputMode::Deg;
    } else if (c == 'r' || c == 'R') {
      g_mode = OutputMode::Raw;
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(PIN_SHOULDER_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_ELBOW_ADC, ADC_11db);

  for (int i = 0; i < 16; ++i) {
    (void)analogRead(PIN_SHOULDER_ADC);
    (void)analogRead(PIN_ELBOW_ADC);
  }

  Serial.println(F("# arm_joint_fb ready"));
  Serial.println(F("# pins A0=GPIO34 shoulder, A1=GPIO35 elbow"));
  Serial.println(F("# calib ESP32 ADC: 3900=0deg 2550=90 1415=180 222=270"));
  Serial.println(F("# mode: send 'd'=deg S:..,E:..  'r'=raw A0:..,A1:.."));
}

void loop() {
  static uint32_t last_us = 0;
  const uint32_t now_us = micros();
  const uint32_t period_us = 1000000UL / SAMPLE_HZ;

  handleSerialCommands();

  if (last_us != 0 && (now_us - last_us) < period_us) {
    return;
  }
  last_us = now_us;

  const uint16_t a0 = readAdcAveraged(PIN_SHOULDER_ADC);
  const uint16_t a1 = readAdcAveraged(PIN_ELBOW_ADC);

  if (g_mode == OutputMode::Raw) {
    Serial.printf("A0:%u,A1:%u\n", static_cast<unsigned>(a0),
                  static_cast<unsigned>(a1));
    return;
  }

  const float shoulder_inst =
      adcToDeg(static_cast<float>(a0), kEsp32Calib, kEsp32CalibN);
  const float elbow_inst =
      adcToDeg(static_cast<float>(a1), kEsp32Calib, kEsp32CalibN);

  float shoulder_deg = 0.0f;
  float elbow_deg = 0.0f;
  applyEma(shoulder_inst, elbow_inst, &shoulder_deg, &elbow_deg);

  Serial.printf("S:%.2f,E:%.2f\n", clampf(shoulder_deg, DEG_MIN, DEG_MAX),
                clampf(elbow_deg, DEG_MIN, DEG_MAX));
}
