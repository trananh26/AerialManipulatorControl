/**
 * ESP32 DOIT DevKit V1 — 2-DOF arm joint ADC → USB serial for ROS arm_fb_bridge.
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

static float adcToDeg(uint16_t adc, float scale, float offset) {
  return clampf(static_cast<float>(adc) * scale + offset, DEG_MIN, DEG_MAX);
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

  // Warm up ADC
  for (int i = 0; i < 16; ++i) {
    (void)analogRead(PIN_SHOULDER_ADC);
    (void)analogRead(PIN_ELBOW_ADC);
  }

  Serial.println(F("# arm_joint_fb ready"));
  Serial.println(F("# pins A0=GPIO34 shoulder, A1=GPIO35 elbow"));
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
      adcToDeg(a0, SHOULDER_ADC_SCALE, SHOULDER_ADC_OFFSET);
  const float elbow_inst = adcToDeg(a1, ELBOW_ADC_SCALE, ELBOW_ADC_OFFSET);

  float shoulder_deg = 0.0f;
  float elbow_deg = 0.0f;
  applyEma(shoulder_inst, elbow_inst, &shoulder_deg, &elbow_deg);

  Serial.printf("S:%.2f,E:%.2f\n", shoulder_deg, elbow_deg);
}
