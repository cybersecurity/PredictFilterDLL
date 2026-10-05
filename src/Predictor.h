// Per-device predictor: owns a PoseFilter plus a ring buffer of past outputs (0x4C00 bytes).
#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "PoseFilter.h"
#include "PredictFilter.h"

// Prints the call rate every 3 seconds (64 bytes).
class FpsCounter {
public:
    FpsCounter();
    void Tick();  // 0x180013800
    void SetName(const std::string& name) { m_name = name; }

private:
    static int64_t Now();  // 100 ns ticks (_Xtime_get_ticks)

    int64_t m_start = 0;    // +0x00
    int64_t m_next = 0;     // +0x08
    int64_t m_count = 0;    // +0x10
    std::string m_name;     // +0x18
    bool m_enabled = true;  // +0x38
};

class Predictor {
public:
    static const int kHistorySize = 200;

    Predictor();   // 0x1800138B0
    ~Predictor();  // 0x180014730

    void Init(int deviceType);  // 0x180013A20
    // 0x180013FE0
    void Predict(const PosePredictState& in, float predictTime, PosePredictState* out);
    void ReleaseFilter() { m_poseFilter.reset(); }

private:
    // 0x180013ED0: run the filter once and append the result to the history.
    int Step(const PosePredictState& in, float predictTime, PosePredictState* out);
    void ResetHistory();

    std::unique_ptr<PoseFilter> m_poseFilter;       // +0x0000
    PosePredictState m_history[kHistorySize];       // +0x0008
    int m_count = 0;                                // +0x4B08
    int m_head = -1;                                // +0x4B0C
    double m_lastPreTime = 0.0;                     // +0x4B10
    int m_dofType = 0;                              // +0x4B18
    int m_capacity = kHistorySize;                  // +0x4B1C
    double m_maxPredictTime = 0.05f;                // +0x4B20
    int m_deviceType = -1;                          // +0x4B28
    std::mutex m_mutex;                             // +0x4B30
    FpsCounter m_hitFps;                            // +0x4B80
    FpsCounter m_filterFps;                         // +0x4BC0
};

// The four global predictors (0x18001CA10, 0x180021610, 0x180026210, 0x18002AE10).
struct PredictorManager {
    Predictor predictors[4];  // HMD, left controller, right controller, HMD (type 4)

    ~PredictorManager() { ReleaseAll(); }
    void ReleaseAll();  // 0x180014680
};

extern PredictorManager g_predictorManager;
