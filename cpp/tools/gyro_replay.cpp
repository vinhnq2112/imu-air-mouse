// gyro_replay.cpp
// Replays a recorded sensor log through GyroPointerCore (the exact algorithm
// that runs on the TV) and writes one CSV row per frame for the visualizer.
//
// Usage:
//   gyro_replay LOG.csv [OUT.csv] [options]      (OUT.csv omitted or "-" -> stdout)
//
// Options:
//   --sensitivity N     1..10, default 5
//   --stability N       1..10, default 5
//   --fps F             frame rate of the device, default 60.
//                       The TV reads only the LATEST sample once per frame, so the
//                       log is resampled the same way (zero-order hold).
//                       0 = process every sample with its own dt.
//   --time-unit s|ms    unit of column t, default s
//   --yaw-axis AXIS     gyro column that moves the cursor horizontally, default z
//   --pitch-axis AXIS   gyro column that moves the cursor vertically,   default x
//                       AXIS = x | y | z, optionally prefixed with '-' to flip sign
//   --screen WxH        screen size used to clamp the cursor, default 1920x1080
//
// Input CSV (header row required): t, gx, gy, gz in revolutions per second
// (1.0 = 360 deg/s). Other columns are ignored, except 'phase' which is copied
// to the output.
//
// Output CSV columns:
//   t, phase, dt, yaw_raw, pitch_raw, yaw_filt, pitch_filt (rad/s), gain,
//   locked, precision, enter_thr, exit_thr (rad/s), dx, dy, x, y (pixels)

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "GyroPointerCore.h"

namespace
{

struct Sample
{
    double      t = 0.0;
    float       gyro[3] = {0.0f, 0.0f, 0.0f}; // rev/s
    std::string phase;
};

struct AxisSpec
{
    int   index = 0;
    float sign  = 1.0f;
};

struct Options
{
    std::string inPath;
    std::string outPath = "-";
    int         sensitivity = 5;
    int         stability   = 5;
    double      fps         = 60.0;
    bool        timeInMs    = false;
    AxisSpec    yawAxis{2, 1.0f};
    AxisSpec    pitchAxis{0, 1.0f};
    float       screenW = 1920.0f;
    float       screenH = 1080.0f;
};

[[noreturn]] void Fail(const std::string& message)
{
    std::cerr << "gyro_replay: " << message << "\n";
    std::exit(1);
}

std::string Trim(const std::string& s)
{
    size_t b = 0, e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

std::string Lower(std::string s)
{
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::vector<std::string> SplitCsv(const std::string& line)
{
    std::vector<std::string> cells;
    std::string cell;
    std::istringstream stream(line);
    while (std::getline(stream, cell, ','))
    {
        cells.push_back(Trim(cell));
    }
    if (!line.empty() && line.back() == ',') cells.emplace_back();
    return cells;
}

AxisSpec ParseAxis(const std::string& text)
{
    std::string s = Lower(Trim(text));
    AxisSpec axis;
    if (!s.empty() && (s[0] == '-' || s[0] == '+'))
    {
        axis.sign = (s[0] == '-') ? -1.0f : 1.0f;
        s = s.substr(1);
    }
    if (s == "x") axis.index = 0;
    else if (s == "y") axis.index = 1;
    else if (s == "z") axis.index = 2;
    else Fail("axis must be x, y or z (optionally with '-'), got '" + text + "'");
    return axis;
}

Options ParseArgs(int argc, char** argv)
{
    Options opt;
    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) Fail("missing value for " + arg);
            return argv[++i];
        };
        if (arg == "-h" || arg == "--help")
        {
            std::cout << "Usage: gyro_replay LOG.csv [OUT.csv] [--sensitivity N] [--stability N]\n"
                         "                   [--fps F] [--time-unit s|ms] [--yaw-axis AXIS]\n"
                         "                   [--pitch-axis AXIS] [--screen WxH]\n";
            std::exit(0);
        }
        else if (arg == "--sensitivity") opt.sensitivity = std::atoi(next().c_str());
        else if (arg == "--stability")   opt.stability   = std::atoi(next().c_str());
        else if (arg == "--fps")         opt.fps         = std::atof(next().c_str());
        else if (arg == "--time-unit")
        {
            const std::string unit = Lower(next());
            if (unit != "s" && unit != "ms") Fail("--time-unit must be s or ms");
            opt.timeInMs = (unit == "ms");
        }
        else if (arg == "--yaw-axis")   opt.yawAxis   = ParseAxis(next());
        else if (arg == "--pitch-axis") opt.pitchAxis = ParseAxis(next());
        else if (arg == "--screen")
        {
            const std::string s = Lower(next());
            const size_t xPos = s.find('x');
            if (xPos == std::string::npos) Fail("--screen must look like 1920x1080");
            opt.screenW = std::strtof(s.substr(0, xPos).c_str(), nullptr);
            opt.screenH = std::strtof(s.substr(xPos + 1).c_str(), nullptr);
            if (opt.screenW <= 0 || opt.screenH <= 0) Fail("--screen must look like 1920x1080");
        }
        else if (arg.size() > 1 && arg[0] == '-' && arg != "-") Fail("unknown option " + arg);
        else positional.push_back(arg);
    }
    if (positional.empty() || positional.size() > 2) Fail("usage: gyro_replay LOG.csv [OUT.csv] [options]");
    opt.inPath = positional[0];
    if (positional.size() == 2) opt.outPath = positional[1];
    if (opt.fps < 0) Fail("--fps must be >= 0");
    return opt;
}

std::vector<Sample> LoadLog(const Options& opt)
{
    std::ifstream file(opt.inPath);
    if (!file) Fail("cannot open " + opt.inPath);

    std::string line;
    if (!std::getline(file, line)) Fail("empty file " + opt.inPath);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.size() >= 3 && line.compare(0, 3, "\xEF\xBB\xBF") == 0) line = line.substr(3); // UTF-8 BOM

    const std::vector<std::string> header = SplitCsv(line);
    auto column = [&](const std::string& name) -> int {
        for (size_t i = 0; i < header.size(); ++i)
            if (Lower(header[i]) == name) return static_cast<int>(i);
        return -1;
    };
    const int colT = column("t");
    const int colG[3] = {column("gx"), column("gy"), column("gz")};
    const int colPhase = column("phase");
    if (colT < 0 || colG[0] < 0 || colG[1] < 0 || colG[2] < 0)
        Fail("CSV needs columns t, gx, gy, gz (gyro in revolutions per second)");

    std::vector<Sample> samples;
    size_t lineNumber = 1;
    while (std::getline(file, line))
    {
        ++lineNumber;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (Trim(line).empty()) continue;
        const std::vector<std::string> cells = SplitCsv(line);
        auto value = [&](int col) -> double {
            if (col >= static_cast<int>(cells.size()) || cells[col].empty())
                Fail("missing value on line " + std::to_string(lineNumber));
            char* end = nullptr;
            const double v = std::strtod(cells[col].c_str(), &end);
            if (end == cells[col].c_str())
                Fail("bad number '" + cells[col] + "' on line " + std::to_string(lineNumber));
            return v;
        };
        Sample s;
        s.t = value(colT) * (opt.timeInMs ? 0.001 : 1.0);
        for (int k = 0; k < 3; ++k) s.gyro[k] = static_cast<float>(value(colG[k]));
        if (colPhase >= 0 && colPhase < static_cast<int>(cells.size())) s.phase = cells[colPhase];
        samples.push_back(s);
    }
    if (samples.size() < 2) Fail("need at least 2 samples");
    for (size_t i = 1; i < samples.size(); ++i)
        if (samples[i].t < samples[i - 1].t)
            Fail("column t must be increasing (line " + std::to_string(i + 2) + ")");
    return samples;
}

} // namespace

int main(int argc, char** argv)
{
    const Options opt = ParseArgs(argc, argv);
    const std::vector<Sample> samples = LoadLog(opt);

    std::ofstream fileOut;
    if (opt.outPath != "-")
    {
        fileOut.open(opt.outPath);
        if (!fileOut) Fail("cannot write " + opt.outPath);
    }
    std::ostream& out = (opt.outPath == "-") ? std::cout : fileOut;

    gyro_pointer::GyroPointerCore core;
    core.SetSettings(opt.sensitivity, opt.stability);

    // The cursor starts at the center of the screen.
    float cursorX = opt.screenW * 0.5f;
    float cursorY = opt.screenH * 0.5f;

    out << "t,phase,dt,yaw_raw,pitch_raw,yaw_filt,pitch_filt,gain,"
           "locked,precision,enter_thr,exit_thr,dx,dy,x,y\n";

    char buffer[512];
    auto emit = [&](double t, float dt, const Sample& s) {
        const float yaw   = opt.yawAxis.sign   * s.gyro[opt.yawAxis.index];
        const float pitch = opt.pitchAxis.sign * s.gyro[opt.pitchAxis.index];
        const gyro_pointer::PointerOutput r = core.Update(dt, yaw, pitch);

        cursorX = std::clamp(cursorX + r.deltaX, 0.0f, opt.screenW - 1.0f);
        cursorY = std::clamp(cursorY + r.deltaY, 0.0f, opt.screenH - 1.0f);

        std::snprintf(buffer, sizeof(buffer),
                      "%.5f,%s,%.5f,%.6f,%.6f,%.6f,%.6f,%.4f,%d,%d,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f\n",
                      t, s.phase.c_str(), r.dtSec, r.yawRate, r.pitchRate,
                      r.smoothYawRate, r.smoothPitchRate, r.gain,
                      r.isLocked ? 1 : 0, r.isPrecisionMode ? 1 : 0,
                      r.enterThreshold, r.exitThreshold,
                      r.deltaX, r.deltaY, cursorX, cursorY);
        out << buffer;
    };

    const double t0 = samples.front().t;
    if (opt.fps == 0.0)
    {
        // Every sample, with its own dt (the first one uses the second interval).
        for (size_t i = 0; i < samples.size(); ++i)
        {
            const double dt = (i == 0) ? samples[1].t - samples[0].t
                                       : samples[i].t - samples[i - 1].t;
            emit(samples[i].t - t0, static_cast<float>(dt), samples[i]);
        }
    }
    else
    {
        // Like the TV: one update per frame, using the latest sample available.
        const double frameDt = 1.0 / opt.fps;
        const double tEnd = samples.back().t;
        size_t latest = 0;
        for (long frame = 0;; ++frame)
        {
            const double t = t0 + frame * frameDt;
            if (t > tEnd + 1e-9) break;
            while (latest + 1 < samples.size() && samples[latest + 1].t <= t + 1e-9) ++latest;
            emit(t - t0, static_cast<float>(frameDt), samples[latest]);
        }
    }
    return 0;
}
