// GyroPointer.h
// Thin engine wrapper around gyro_pointer::GyroPointerCore.
//
// This is the class used on the TV. It only does the two things that depend
// on the engine, then hands everything else to the core:
//   - reads the angular velocity with getInputstate()
//   - measures the frame time with std::chrono::steady_clock
// The algorithm itself lives in GyroPointerCore.h, which is also what
// tools/gyro_replay.cpp runs offline, so the visualizer shows exactly the
// same behavior as the device.
//
// The engine must declare, BEFORE including this header:
//   - float2 with x() / y() accessors returning references
//   - input ids angularVelocityX / angularVelocityZ
//   - getInputstate(id) returning angular velocity in revolutions per second
//     (1.0 = 360 deg/s)
// Axis mapping: yaw (Z) -> horizontal, pitch (X) -> vertical.

#pragma once

#include <chrono>

#include "GyroPointerCore.h"

class GyroPointer
{
public:
    static constexpr int kMinSetting     = gyro_pointer::GyroPointerCore::kMinSetting;
    static constexpr int kMaxSetting     = gyro_pointer::GyroPointerCore::kMaxSetting;
    static constexpr int kDefaultSetting = gyro_pointer::GyroPointerCore::kDefaultSetting;

    // sensitivity / stability: 1..10 (see GyroPointerCore::SetSettings).
    void SetSettings(int sensitivity, int stability)
    {
        core.SetSettings(sensitivity, stability);
    }

    // Call once per frame. Returns the cursor delta position in pixels.
    float2 UpdateControlState()
    {
        // Frame time: seconds since the previous call.
        // steady_clock never jumps backwards, unlike system_clock.
        const auto now = std::chrono::steady_clock::now();
        float dt = 1.0f / 60.0f; // fallback for the very first call
        if (hasPreviousTime)
        {
            dt = std::chrono::duration<float>(now - previousTime).count();
        }
        previousTime    = now;
        hasPreviousTime = true;

        // Angular velocity in revolutions per second, drift-compensated by the sensor.
        const float yawRevPerSec   = getInputstate(angularVelocityZ);
        const float pitchRevPerSec = getInputstate(angularVelocityX);

        const gyro_pointer::PointerOutput result = core.Update(dt, yawRevPerSec, pitchRevPerSec);

        float2 deltaPosition = {};
        deltaPosition.x() = result.deltaX;
        deltaPosition.y() = result.deltaY;
        return deltaPosition;
    }

private:
    gyro_pointer::GyroPointerCore core;

    std::chrono::steady_clock::time_point previousTime;
    bool hasPreviousTime = false;
};
