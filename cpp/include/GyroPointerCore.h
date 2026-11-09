// GyroPointerCore.h
// Engine-independent core of the air-mouse pointer algorithm.
//
// The core does not read sensors or clocks by itself: the caller passes the
// frame time and the angular velocity explicitly. This makes the SAME code
// usable in two places:
//   - on the TV, through the thin engine wrapper in GyroPointer.h
//   - offline, through tools/gyro_replay.cpp, which replays a recorded sensor
//     log and writes the result for the Python visualizer
//
// User settings (the only two values exposed to the user):
//   - Sensitivity (1..10): how far the cursor moves for a given wrist rotation.
//   - Stability   (1..10): how steady the cursor is (less shaking / drifting).
// Every internal parameter is derived from these two values in SetSettings().
//
// Pipeline (runs once per Update() call, i.e. once per frame):
//   1. Input             : dt (clamped) + angular velocity (rev/s -> rad/s)
//   2. Stillness lock    : hysteresis + precision mode -> output 0 when the hand stops
//   3. Low-pass filter   : remove hand tremor (starts from 0 after unlocking)
//   4. Gain curve        : sigmoid gain based on the FILTERED speed
//   5. Compute delta     : delta = -omega * dt * pixelsPerRadian * gain
//   (optional) Sub-pixel : only if the system truncates the delta to integers
//
// Input unit: revolutions per second (1.0 = 360 deg/s), the repo-wide standard.

#pragma once

#include <algorithm>
#include <cmath>

namespace gyro_pointer
{

constexpr float kPi = 3.14159265358979f;

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
        const float timeConstant = 1.0f / (2.0f * kPi * cutoffHz);
        const float alpha = dt / (dt + timeConstant);
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
// Result of one Update() call: the cursor delta plus the internal state,
// so offline tools can show WHY the cursor moved (or did not move).
// ============================================================================
struct PointerOutput
{
    float deltaX = 0.0f;          // pixels
    float deltaY = 0.0f;          // pixels

    float dtSec          = 0.0f;  // dt actually used (after clamping)
    float yawRate        = 0.0f;  // rad/s, unfiltered input
    float pitchRate      = 0.0f;  // rad/s, unfiltered input
    float smoothYawRate  = 0.0f;  // rad/s, after the low-pass filter (0 while locked)
    float smoothPitchRate = 0.0f; // rad/s, after the low-pass filter (0 while locked)
    float gain           = 0.0f;  // gain multiplier (0 while locked)

    bool  isLocked        = false; // stillness lock is active (delta = 0)
    bool  isPrecisionMode = false; // low thresholds are in use
    float enterThreshold  = 0.0f;  // rad/s, lock threshold in use this frame
    float exitThreshold   = 0.0f;  // rad/s, unlock threshold in use this frame
};

// ============================================================================
// GyroPointerCore
// ============================================================================
class GyroPointerCore
{
public:
    static constexpr int kMinSetting     = 1;
    static constexpr int kMaxSetting     = 10;
    static constexpr int kDefaultSetting = 5;

    GyroPointerCore()
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
        const float degreesPerScreen = 50.0f - 4.0f * s;
        basePixelsPerRadian = kScreenWidthPx / (degreesPerScreen * kPi / 180.0f);

        // Stability -> lock thresholds (rad/s) and filter cutoff (Hz).
        //   t = 5 -> lock 0.10 / 0.13, precision 0.02 / 0.04, filter 8 Hz.
        lockEnterThreshold      = 0.02f * t;
        lockExitThreshold       = 1.3f  * lockEnterThreshold;
        precisionEnterThreshold = 0.2f  * lockEnterThreshold;
        precisionExitThreshold  = 0.4f  * lockEnterThreshold;

        filterYaw.cutoffHz   = 13.0f - t; // t = 1 -> 12 Hz, t = 10 -> 3 Hz
        filterPitch.cutoffHz = 13.0f - t;
    }

    // Clears all state (lock, filters, sub-pixels). Settings are kept.
    void Reset()
    {
        isLocked        = false;
        stillTimeSec    = 0.0f;
        lockedTimeSec   = 0.0f;
        isPrecisionMode = false;
        ResetMotionState();
    }

    // ------------------------------------------------------------------
    // Process one frame.
    //   dtSec          : seconds since the previous frame
    //   yawRevPerSec   : rotation that moves the cursor horizontally (rev/s)
    //   pitchRevPerSec : rotation that moves the cursor vertically   (rev/s)
    // ------------------------------------------------------------------
    PointerOutput Update(float dtSec, float yawRevPerSec, float pitchRevPerSec)
    {
        PointerOutput out;

        // ==================================================================
        // Step 1: Input
        // ==================================================================
        // 1a. Clamp dt (protects against first frame, app pauses, hitches).
        const float dt = std::clamp(dtSec, kMinDtSec, kMaxDtSec);

        // 1b. Convert revolutions per second to radians per second.
        const float yawRate      = yawRevPerSec   * 2.0f * kPi;
        const float pitchRate    = pitchRevPerSec * 2.0f * kPi;
        const float angularSpeed = std::sqrt(yawRate * yawRate + pitchRate * pitchRate);

        out.dtSec     = dt;
        out.yawRate   = yawRate;
        out.pitchRate = pitchRate;

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
                return FillState(out);
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
                    return FillState(out);
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

        out.deltaX          = deltaX;
        out.deltaY          = deltaY;
        out.smoothYawRate   = smoothYawRate;
        out.smoothPitchRate = smoothPitchRate;
        out.gain            = gain;
        return FillState(out);
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

    // Copies the lock state into the output.
    PointerOutput& FillState(PointerOutput& out) const
    {
        out.isLocked        = isLocked;
        out.isPrecisionMode = isPrecisionMode;
        out.enterThreshold  = isPrecisionMode ? precisionEnterThreshold : lockEnterThreshold;
        out.exitThreshold   = isPrecisionMode ? precisionExitThreshold  : lockExitThreshold;
        return out;
    }

    // Clears filter history and leftover sub-pixels.
    void ResetMotionState()
    {
        filterYaw.Reset();
        filterPitch.Reset();
        accumulatedX = 0.0f;
        accumulatedY = 0.0f;
    }

    // ---------------- Values derived from user settings ----------------
    float basePixelsPerRadian     = 0.0f;
    float lockEnterThreshold      = 0.0f;
    float lockExitThreshold       = 0.0f;
    float precisionEnterThreshold = 0.0f;
    float precisionExitThreshold  = 0.0f;

    // ---------------- Runtime state ----------------

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

} // namespace gyro_pointer
