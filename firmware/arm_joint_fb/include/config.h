/**
 * Arm joint ADC feedback — pin map & calibration for ESP32 DOIT DevKit V1.
 *
 * Serial protocol (newline-terminated), baud 115200 — matches arm_fb_bridge.py:
 *   DEG mode:  S:<shoulder_deg>,E:<elbow_deg>
 *   RAW mode:  A0:<adc>,A1:<adc>
 *
 * Wire pot / FB feedback (0…3.3 V) to ADC1 pins (input-only OK):
 *   GPIO34 = shoulder (A0)
 *   GPIO35 = elbow    (A1)
 * Share GND with pot supply. Prefer 3.3 V rail for pot Vcc (not 5 V).
 *
 * Calibration (same curve for both joints, from bench measurements):
 *
 *   không qua ADS1115 (ESP32 ADC 12-bit, used by this firmware):
 *     0°   PWM  500   ADC 3900   3.13 V
 *    90°   PWM 1166   ADC 2550   2.06 V
 *   180°   PWM 1833   ADC 1415   1.13 V
 *   270°   PWM 2500   ADC  222   0.18 V
 *
 *   đọc bằng ADS1115 (16-bit, optional later):
 *     0°   24208 @ 3.03 V
 *    90°   16972 @ 2.11 V
 *   180°    9565 @ 1.21 V
 *   270°    2214 @ 0.28 V
 */

#pragma once

#include <Arduino.h>
#include <stdint.h>

// ── Pins (ADC1 — usable with WiFi) ──────────────────────────────────────────
static constexpr int PIN_SHOULDER_ADC = 34;  // A0
static constexpr int PIN_ELBOW_ADC    = 35;  // A1

// ── Sampling ────────────────────────────────────────────────────────────────
static constexpr uint32_t SERIAL_BAUD     = 115200;
static constexpr uint32_t SAMPLE_HZ       = 50;
static constexpr uint8_t  ADC_OVERSAMPLE  = 8;     // average N reads per sample
static constexpr float    EMA_ALPHA       = 0.25f; // 0…1, higher = less smoothing

// Mechanical / command range of this servo FB (PWM 500→0°, 2500→270°)
static constexpr float DEG_MIN = 0.0f;
static constexpr float DEG_MAX = 270.0f;

struct CalibPoint {
  float adc;
  float deg;
};

// Midpoints of measured ADC bands (ESP32 onboard ADC).
static constexpr CalibPoint kEsp32Calib[] = {
    {3900.0f, 0.0f},
    {2550.0f, 90.0f},
    {1415.0f, 180.0f},
    {222.0f, 270.0f},
};
static constexpr uint8_t kEsp32CalibN =
    static_cast<uint8_t>(sizeof(kEsp32Calib) / sizeof(kEsp32Calib[0]));

// ADS1115 table kept for later (not used while reading GPIO34/35).
static constexpr CalibPoint kAds1115Calib[] = {
    {24208.5f, 0.0f},
    {16972.0f, 90.0f},
    {9565.0f, 180.0f},
    {2214.0f, 270.0f},
};
static constexpr uint8_t kAds1115CalibN =
    static_cast<uint8_t>(sizeof(kAds1115Calib) / sizeof(kAds1115Calib[0]));

/** Output mode at boot. Toggle at runtime: send 'd' = deg, 'r' = raw. */
enum class OutputMode : uint8_t { Deg = 0, Raw = 1 };
static constexpr OutputMode BOOT_OUTPUT_MODE = OutputMode::Deg;
