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
 */

#pragma once

#include <Arduino.h>

// ── Pins (ADC1 — usable with WiFi) ──────────────────────────────────────────
static constexpr int PIN_SHOULDER_ADC = 34;  // A0
static constexpr int PIN_ELBOW_ADC    = 35;  // A1

// ── Sampling ────────────────────────────────────────────────────────────────
static constexpr uint32_t SERIAL_BAUD     = 115200;
static constexpr uint32_t SAMPLE_HZ       = 50;
static constexpr uint8_t  ADC_OVERSAMPLE  = 8;     // average N reads per sample
static constexpr float    EMA_ALPHA       = 0.25f; // 0…1, higher = less smoothing

// ── Linear map: deg = adc * scale + offset ──────────────────────────────────
// Default: 12-bit ADC 0→0°, 4095→180° (edit after mechanical calibration)
static constexpr float SHOULDER_ADC_SCALE  = 180.0f / 4095.0f;
static constexpr float SHOULDER_ADC_OFFSET = 0.0f;
static constexpr float ELBOW_ADC_SCALE     = 180.0f / 4095.0f;
static constexpr float ELBOW_ADC_OFFSET    = 0.0f;

static constexpr float DEG_MIN = 0.0f;
static constexpr float DEG_MAX = 180.0f;

/** Output mode at boot. Toggle at runtime: send 'd' = deg, 'r' = raw. */
enum class OutputMode : uint8_t { Deg = 0, Raw = 1 };
static constexpr OutputMode BOOT_OUTPUT_MODE = OutputMode::Deg;
