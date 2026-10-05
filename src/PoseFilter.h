// Per-device prediction + smoothing pipeline (0x5E0 bytes, allocated through Eigen's aligned new).
#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <fstream>
#include <memory>
#include <string>

#include "OneEuroFilter.h"
#include "PredictFilter.h"

class PoseFilter {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit PoseFilter(int deviceType);  // 0x180002990
    ~PoseFilter();                        // 0x180003300

    // 0x180004B00: extrapolate `in` by `predictTime` seconds and smooth the result.
    void Process(const PosePredictState& in, float predictTime, PosePredictState* out);

    // 0x180003FC0: dst.<derivatives> = (1 - t) * src.<derivatives> + t * dst.<derivatives>
    // (except linearAcceleration, which the binary blends with src.linearVelocity)
    void BlendDerivatives(float t, PosePredictState* dst, const PosePredictState* src) const;

private:
    // 0x180003580
    void Reset();
    // 0x1800036B0: constant-acceleration extrapolation.
    PosePredictState* Predict(PosePredictState* out, PosePredictState in, float dt);
    // 0x180004220: 1-Euro filter on position / rotation plus quaternion averaging.
    PosePredictState* Filter(PosePredictState* out, PosePredictState in, float predictTime);

    enum { kPosX, kPosY, kPosZ, kRotX, kRotY, kRotZ, kRotW, kNumFilters };

    int m_deviceType;                                       // +0x000
    std::unique_ptr<OneEuroFilter> m_filters[kNumFilters];  // +0x008

    // HMD parameters (position / rotation).
    float m_hmdPosFreq = 100.0f;       // +0x040
    float m_hmdRotFreq = 100.0f;       // +0x044
    float m_hmdPosMinCutoff = 0.5f;    // +0x048
    float m_hmdRotMinCutoff = 0.4f;    // +0x04C
    float m_hmdPosBeta = 400.0f;       // +0x050
    float m_hmdRotBeta = 600.0f;       // +0x054
    float m_hmdPosDCutoff = 30.0f;     // +0x058
    float m_hmdRotDCutoff = 30.0f;     // +0x05C
    int m_hmdWindowSize = 2;           // +0x060
    // Controller parameters (shared by all 7 channels).
    float m_ctrlFreq = 90.0f;          // +0x064
    float m_ctrlMinCutoff = 0.01f;     // +0x068
    float m_ctrlBeta = 500.0f;         // +0x06C
    float m_ctrlDCutoff = 90.0f;       // +0x070
    int m_ctrlWindowSize = 2;          // +0x074

    double m_lastFilterTime = -1.0;    // +0x078
    double m_lastPredictTime = -1.0;   // +0x080
    double m_currentTime = -1.0;       // +0x088
    double m_lastOutputTime = -1.0;    // +0x090
    bool m_quatInitialized = false;    // +0x098
    Eigen::Vector4f m_lastQuat;        // +0x0A0 (x, y, z, w)
    uint64_t m_lastOutputTimestamp = static_cast<uint64_t>(-1);  // +0x0B0
    PosePredictState m_lastOutput;     // +0x0B8

    std::string m_logDir;              // +0x118
    std::string m_predictedPosePath;   // +0x138
    std::string m_inputPosePath;       // +0x158
    std::string m_savePosePath;        // +0x178
    std::string m_filteredPosePath;    // +0x198
    // None of these streams is ever opened by the binary, so writes to them are no-ops.
    std::ofstream m_filteredLog;       // +0x1B8 receives the filtered pose
    std::ofstream m_unusedLog1;        // +0x2C0
    std::ofstream m_predictedLog;      // +0x3C8 receives the predicted pose
    std::ofstream m_unusedLog3;        // +0x4D0
};
