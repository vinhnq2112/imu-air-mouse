// main.cpp
// Minimal standalone example for GyroPointer.h.
//
// GyroPointer.h expects three things from the host engine:
//   - a float2 type with x() / y() accessors returning references
//   - input ids angularVelocityX / angularVelocityZ
//   - getInputstate(id) returning angular velocity in revolutions per second
// This file provides simple stand-ins for them and feeds a scripted
// wrist motion at ~60 fps, printing the cursor position each frame.
//
// In your real project, delete these stand-ins and include GyroPointer.h
// after your engine's own float2 / getInputstate declarations.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

// ---------------------------------------------------------------------------
// Stand-ins for the host engine
// ---------------------------------------------------------------------------
struct float2
{
    float v[2] = {0.0f, 0.0f};
    float& x() { return v[0]; }
    float& y() { return v[1]; }
};

enum InputId { angularVelocityX, angularVelocityY, angularVelocityZ };

// Simulated sensor value in revolutions per second (1.0 = 360 deg/s).
static float gSimRevPerSec[3] = {0.0f, 0.0f, 0.0f};

float getInputstate(InputId id) { return gSimRevPerSec[id]; }

#include "GyroPointer.h"

// ---------------------------------------------------------------------------
// Scripted motion:
//   0.0 - 0.5 s : hold still
//   0.5 - 1.0 s : turn wrist to the left at 20 deg/s (yaw)
//   1.0 - 1.5 s : hold still (cursor must stop, no drift)
//   1.5 - 2.0 s : tilt up slowly at 10 deg/s (pitch)
//   2.0 - 2.5 s : hold still
// Small noise is added to mimic hand tremor.
// ---------------------------------------------------------------------------
static void SimulateSensor(float timeSec)
{
    constexpr float kDegPerSecToRevPerSec = 1.0f / 360.0f;
    const float tremor = 0.4f * std::sin(2.0f * 3.14159265f * 10.0f * timeSec); // ~10 Hz, 0.4 deg/s

    float yawDegPerSec   = tremor;
    float pitchDegPerSec = tremor;
    if (timeSec >= 0.5f && timeSec < 1.0f) yawDegPerSec   += 20.0f;
    if (timeSec >= 1.5f && timeSec < 2.0f) pitchDegPerSec += 10.0f;

    gSimRevPerSec[angularVelocityZ] = yawDegPerSec   * kDegPerSecToRevPerSec;
    gSimRevPerSec[angularVelocityX] = pitchDegPerSec * kDegPerSecToRevPerSec;
}

int main()
{
    constexpr float kScreenW = 1920.0f;
    constexpr float kScreenH = 1080.0f;

    GyroPointer pointer;
    pointer.SetSettings(/*sensitivity=*/5, /*stability=*/5);

    // Start the cursor at the center of the screen.
    float cursorX = kScreenW * 0.5f;
    float cursorY = kScreenH * 0.5f;

    const auto start      = std::chrono::steady_clock::now();
    const auto frameTime  = std::chrono::microseconds(16667); // ~60 fps
    auto       nextFrame  = start;

    std::printf("%6s %8s %8s %8s %8s\n", "t(s)", "dx", "dy", "cursorX", "cursorY");
    for (int frame = 0; frame < 150; ++frame)
    {
        const float timeSec =
            std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
        SimulateSensor(timeSec);

        float2 delta = pointer.UpdateControlState();

        // Apply the delta and keep the cursor on screen.
        cursorX = std::fmin(std::fmax(cursorX + delta.x(), 0.0f), kScreenW - 1.0f);
        cursorY = std::fmin(std::fmax(cursorY + delta.y(), 0.0f), kScreenH - 1.0f);

        if (frame % 5 == 0)
        {
            std::printf("%6.2f %8.2f %8.2f %8.1f %8.1f\n",
                        timeSec, delta.x(), delta.y(), cursorX, cursorY);
        }

        nextFrame += frameTime;
        std::this_thread::sleep_until(nextFrame);
    }
    return 0;
}
