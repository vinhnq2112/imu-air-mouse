// GyroPointer.h
// Converts gyroscope angular velocity from a remote control into a
// smooth cursor delta position (air-mouse style).
//
// User settings (the only two values exposed to the user):
//   - Sensitivity (1..10): how far the cursor moves for a given wrist rotation.
//   - Stability   (1..10): how steady the cursor is (less shaking / drifting).
// Every internal parameter is derived from these two values in SetSettings().
// Default 5 / 5 reproduces the tuned values from testing.
//
// Pipeline (runs once per UpdateControlState() call, i.e. once per frame):
//   1. Read input        : angular velocity (rev/s -> rad/s) + frame time dt (steady_clock)
//   2. Stillness lock    : hysteresis + precision mode -> output 0 when the hand stops
//   3. Low-pass filter   : remove hand tremor (starts from 0 after unlocking)
//   4. Gain curve        : sigmoid gain based on the FILTERED speed
//   5. Compute delta     : delta = -omega * dt * pixelsPerRadian * gain
//   (optional) Sub-pixel : only if the system truncates the delta to integers
//
// Assumptions:
//   - getInputstate(angularVelocityZ / angularVelocityX) returns revolutions
//     per second (1.0 = 360 deg/s). It is converted to rad/s in Step 1.
//   - The sensor already has zero-drift compensation (no bias calibration needed).
//   - float2 has x() / y() accessors returning references.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>

// ============================================================================
// LowPassFilter
// First-order low-pass filter (exponential moving average):
//   filtered = alpha * raw + (1 - alpha) * filtered
// alpha is computed from the cutoff frequency and dt, so the amount of
// smoothing stays the same even if the frame rate changes.
// ============================================================================
class LowPassFilter
{
public:
    // Cutoff frequency (Hz). Lower = smoother but more lag.
    float cutoffHz = 8.0f;

    float Filter(float rawValue, float dt)
    {
        // Convert cutoff frequency to a smoothing factor (0..1).
        constexpr float kPi = 3.14159265358979f;
        const float timeConstant = 1.0f / (2.0f * kPi * cutoffHz);
        const float alpha = dt / (dt + timeConstant);

        // Blend the new sample with the previous filtered value.
        filteredValue = alpha * rawValue + (1.0f - alpha) * filteredValue;
        return filteredValue;
    }

    // Restart from zero. After the cursor unlocks, the output ramps up
    // smoothly from 0 instead of jumping straight to the raw value.
    void Reset() { filteredValue = 0.0f; }

private:
    float filteredValue = 0.0f;
};

// ============================================================================
// GyroPointer
// ============================================================================
class GyroPointer
{
public:
    static constexpr int kMinSetting     = 1;
    static constexpr int kMaxSetting     = 10;
    static constexpr int kDefaultSetting = 5;

    GyroPointer()
    {
        SetSettings(kDefaultSetting, kDefaultSetting);
    }

    // ------------------------------------------------------------------
    // User settings
    //   sensitivity (1..10): higher = cursor moves farther per rotation.
    //   stability   (1..10): higher = steadier cursor (less shaking and
    //                        drifting), but less reactive to tiny movements.
    // ------------------------------------------------------------------
    void SetSettings(int sensitivity, int stability)
    {
        const float s = static_cast<float>(std::clamp(sensitivity, kMinSetting, kMaxSetting));
        const float t = static_cast<float>(std::clamp(stability,   kMinSetting, kMaxSetting));

        // Sensitivity -> degrees of wrist rotation to sweep the screen width.
        //   s = 1 -> 46 deg, s = 5 -> 30 deg, s = 10 -> 10 deg.
        constexpr float kDegToRad = 3.14159265358979f / 180.0f;
        const float degreesPerScreen = 50.0f - 4.0f * s;
        basePixelsPerRadian = kScreenWidthPx / (degreesPerScreen * kDegToRad);

        // Stability -> lock thresholds (rad/s) and filter cutoff (Hz).
        //   t = 5 -> lock 0.10 / 0.13, precision 0.02 / 0.04, filter 8 Hz.
        lockEnterThreshold      = 0.02f * t;
        lockExitThreshold       = 1.3f  * lockEnterThreshold;
        precisionEnterThreshold = 0.2f  * lockEnterThreshold;
        precisionExitThreshold  = 0.4f  * lockEnterThreshold;

        filterYaw.cutoffHz   = 13.0f - t; // t = 1 -> 12 Hz, t = 10 -> 3 Hz
        filterPitch.cutoffHz = 13.0f - t;
    }

    // Call once per frame. Returns the cursor delta position in pixels.
    float2 UpdateControlState()
    {
        float2 deltaPosition = {};

        // ==================================================================
        // Step 1: Read input
        // ==================================================================

        // 1a. Frame time: seconds since the previous call.
        //     steady_clock never jumps backwards, unlike system_clock.
        const auto now = std::chrono::steady_clock::now();
        float dt = 1.0f / 60.0f; // fallback for the very first call
        if (hasPreviousTime)
        {
            dt = std::chrono::duration<float>(now - previousTime).count();
        }
        previousTime    = now;
        hasPreviousTime = true;
        dt = std::clamp(dt, kMinDtSec, kMaxDtSec);

        // 1b. Angular velocity, already drift-compensated by the sensor.
        //     The sensor reports revolutions per second (1.0 = 360 deg/s),
        //     so convert to radians per second: 1 rev = 2*pi rad.
        //     Yaw   (rotation around Z) -> horizontal cursor movement.
        //     Pitch (rotation around X) -> vertical cursor movement.
        constexpr float kRevToRad = 2.0f * 3.14159265358979f;
        const float yawRate      = getInputstate(angularVelocityZ) * kRevToRad;
        const float pitchRate    = getInputstate(angularVelocityX) * kRevToRad;
        const float angularSpeed = std::sqrt(yawRate * yawRate + pitchRate * pitchRate);

        // ==================================================================
        // Step 2: Stillness lock with hysteresis (+ precision mode)
        //   Unlocked -> Locked  : speed < enter threshold for kLockHoldTimeSec
        //   Locked   -> Unlocked: speed > exit threshold
        //   Normal mode uses high thresholds (cuts the sensor's decay tail
        //   after a fast move). Precision mode uses low thresholds so tiny
        //   corrective movements toward a small target are not swallowed.
        // ==================================================================
        if (isLocked)
        {
            // Locked long enough -> the sensor tail is gone -> precision mode.
            lockedTimeSec += dt;
            if (lockedTimeSec >= kPrecisionSettleTimeSec)
            {
                isPrecisionMode = true;
            }

            const float exitThreshold = isPrecisionMode ? precisionExitThreshold
                                                        : lockExitThreshold;
            if (angularSpeed > exitThreshold)
            {
                // Hand started moving again: unlock and continue the pipeline.
                isLocked     = false;
                stillTimeSec = 0.0f;
            }
            else
            {
                // Still locked: cursor does not move.
                return deltaPosition;
            }
        }
        else
        {
            // A big move ends precision mode, so the high thresholds are
            // used again when the hand stops.
            if (angularSpeed > kPrecisionLeaveSpeed)
            {
                isPrecisionMode = false;
            }

            const float enterThreshold = isPrecisionMode ? precisionEnterThreshold
                                                         : lockEnterThreshold;
            if (angularSpeed < enterThreshold)
            {
                stillTimeSec += dt;
                if (stillTimeSec >= kLockHoldTimeSec)
                {
                    // Hand has stopped: lock and clear motion history so the
                    // next movement starts cleanly.
                    isLocked      = true;
                    lockedTimeSec = 0.0f;
                    ResetMotionState();
                    return deltaPosition;
                }
            }
            else
            {
                stillTimeSec = 0.0f;
            }
        }

        // ==================================================================
        // Step 3: Remove hand tremor with a low-pass filter
        // ==================================================================
        const float smoothYawRate   = filterYaw.Filter(yawRate, dt);
        const float smoothPitchRate = filterPitch.Filter(pitchRate, dt);

        // ==================================================================
        // Step 4: Gain curve (sigmoid) based on the FILTERED speed
        //   Slow -> kLowGain  (precise aiming at small targets)
        //   Fast -> kHighGain (cross the screen quickly)
        // ==================================================================
        const float smoothSpeed = std::sqrt(smoothYawRate * smoothYawRate
                                          + smoothPitchRate * smoothPitchRate);
        const float gain = kLowGain + (kHighGain - kLowGain)
                         / (1.0f + std::exp(-kTransitionSteepness
                                            * (smoothSpeed - kTransitionSpeed)));

        // ==================================================================
        // Step 5: Compute delta position (pixels)
        //   angle (rad) = angular velocity (rad/s) * dt (s)
        //   pixels      = angle * pixelsPerRadian * gain
        //   The same pixelsPerRadian is used on both axes so diagonal
        //   movement is not distorted.
        //   Negative sign keeps the same direction as the original code.
        // ==================================================================
        const float pixelsPerRadian = basePixelsPerRadian * gain;
        float deltaX = -smoothYawRate   * dt * pixelsPerRadian;
        float deltaY = -smoothPitchRate * dt * pixelsPerRadian;

        // ------------------------------------------------------------------
        // Optional: sub-pixel accumulation (only if the system truncates).
        // Keep the fractional part for the next frame so slow movements are
        // not rounded away. trunc() rounds toward zero, so left/up and
        // right/down behave the same.
        // ------------------------------------------------------------------
        if (kUseSubpixelAccumulator)
        {
            accumulatedX += deltaX;
            accumulatedY += deltaY;
            deltaX = std::trunc(accumulatedX);
            deltaY = std::trunc(accumulatedY);
            accumulatedX -= deltaX;
            accumulatedY -= deltaY;
        }

        deltaPosition.x() = deltaX;
        deltaPosition.y() = deltaY;
        return deltaPosition;
    }

private:
    // ---------------- Fixed internal constants ----------------

    // Screen width used as the reference for sensitivity.
    static constexpr float kScreenWidthPx = 1920.0f;

    // Valid range for dt (protects against first frame, pauses, hitches).
    static constexpr float kMinDtSec = 0.001f;
    static constexpr float kMaxDtSec = 0.05f;

    // Stillness lock timing.
    static constexpr float kLockHoldTimeSec        = 0.03f; // time below threshold before locking
    static constexpr float kPrecisionSettleTimeSec = 0.4f;  // locked time before precision mode
    static constexpr float kPrecisionLeaveSpeed    = 0.3f;  // rad/s, a big move ends precision mode

    // Gain curve (sigmoid).
    static constexpr float kLowGain             = 0.6f; // slow aiming
    static constexpr float kHighGain            = 3.0f; // fast travel
    static constexpr float kTransitionSpeed     = 1.0f; // rad/s, middle of the transition
    static constexpr float kTransitionSteepness = 3.0f;

    // Set to true ONLY if the system truncates the delta to integers.
    // Test: return delta {0.3, 0} for 100 frames.
    //   cursor moves ~30 px -> system keeps floats -> leave false
    //   cursor moves 0 px   -> system truncates   -> set true
    static constexpr bool kUseSubpixelAccumulator = false;

    // ---------------- Values derived from user settings ----------------
    float basePixelsPerRadian     = 0.0f;
    float lockEnterThreshold      = 0.0f;
    float lockExitThreshold       = 0.0f;
    float precisionEnterThreshold = 0.0f;
    float precisionExitThreshold  = 0.0f;

    // ---------------- Runtime state ----------------

    // Clears filter history and leftover sub-pixels.
    void ResetMotionState()
    {
        filterYaw.Reset();
        filterPitch.Reset();
        accumulatedX = 0.0f;
        accumulatedY = 0.0f;
    }

    // Step 1: timing
    std::chrono::steady_clock::time_point previousTime;
    bool hasPreviousTime = false;

    // Step 2: stillness lock
    bool  isLocked        = false;
    float stillTimeSec    = 0.0f;
    float lockedTimeSec   = 0.0f;
    bool  isPrecisionMode = false;

    // Step 3: filters
    LowPassFilter filterYaw;
    LowPassFilter filterPitch;

    // Optional: sub-pixel accumulators
    float accumulatedX = 0.0f;
    float accumulatedY = 0.0f;
};
