#include "Predictor.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <Eigen/Geometry>

#include "Logger.h"

PredictorManager g_predictorManager;  // 0x180001140

namespace {

double TimestampToSeconds(uint64_t ns) {
    return static_cast<double>(ns) / 1000000000.0;
}

uint64_t SecondsToNs(float seconds) {
    return static_cast<uint64_t>(seconds * 1000000000.0);
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// FpsCounter

int64_t FpsCounter::Now() {
    using Ticks = std::chrono::duration<int64_t, std::ratio<1, 10000000>>;
    return std::chrono::duration_cast<Ticks>(std::chrono::system_clock::now().time_since_epoch()).count();
}

FpsCounter::FpsCounter() {
    m_start = Now();
    m_next = m_start + 30000000;
    m_count = 0;
}

void FpsCounter::Tick() {
    if (!m_enabled)
        return;
    ++m_count;
    const int64_t now = Now();
    if (now >= m_next) {
        const double seconds = static_cast<double>(now - m_start) / 10000000.0;
        g_logger.Warn("%s: %f FPS\n", m_name.c_str(), static_cast<double>(m_count) / seconds);
        m_start = Now();
        m_next = m_start + 30000000;
        m_count = 0;
    }
}

// ---------------------------------------------------------------------------------------------
// Predictor

Predictor::Predictor() {
    for (auto& entry : m_history)
        entry.timestamp = 0;
    m_poseFilter.reset();
    m_count = 0;
    m_maxPredictTime = 0.05f;
    m_lastPreTime = 0.0;
    m_dofType = 0;
    m_head = -1;
    m_deviceType = -1;
    m_capacity = kHistorySize;
}

Predictor::~Predictor() = default;

void Predictor::ResetHistory() {
    m_count = 0;
    m_head = -1;
    m_lastPreTime = 0.0;
    m_dofType = -1;
    m_deviceType = -1;
}

void Predictor::Init(int deviceType) {
    m_count = 0;
    m_head = -1;
    m_lastPreTime = 0.0;
    m_dofType = -1;
    m_deviceType = deviceType;
    m_poseFilter.reset(new PoseFilter(deviceType));
    if (m_deviceType != 0)
        m_capacity = 100;

    switch (deviceType) {
    case 0:
        m_hitFps.SetName("Hmd hit");
        m_filterFps.SetName("Hmd filter");
        break;
    case 1:
        m_hitFps.SetName("Left hit");
        m_filterFps.SetName("Left filter");
        break;
    case 2:
        m_hitFps.SetName("Right hit");
        m_filterFps.SetName("Right filter");
        break;
    default:
        m_hitFps.SetName("other hit");
        m_filterFps.SetName("other filter");
        break;
    }
}

int Predictor::Step(const PosePredictState& in, float predictTime, PosePredictState* out) {
    m_filterFps.Tick();
    m_poseFilter->Process(in, predictTime, out);
    if (m_count < m_capacity)
        ++m_count;
    out->timestamp = in.timestamp + SecondsToNs(predictTime);
    m_head = (m_head + 1) % m_capacity;
    m_history[m_head] = *out;
    return m_head;
}

void Predictor::Predict(const PosePredictState& in, float predictTime, PosePredictState* out) {
    m_hitFps.Tick();
    std::lock_guard<std::mutex> lock(m_mutex);

    PosePredictState result = in;
    if (predictTime > m_maxPredictTime)
        predictTime = static_cast<float>(m_maxPredictTime);
    const double inTime = TimestampToSeconds(in.timestamp);
    const double absTime = predictTime + inTime;
    result.timestamp = in.timestamp + SecondsToNs(predictTime);

    if (!(in.poseQuality > 0.0f) || !m_poseFilter) {
        g_logger.Warn("pose_quality %f\n", in.poseQuality);
    } else {
        double lastPreTime;
        if (in.dofType == m_dofType) {
            lastPreTime = m_lastPreTime;
        } else {
            g_logger.Warn("dof type changed to %d, do reset!\n", in.dofType);
            ResetHistory();
            m_dofType = in.dofType;
            lastPreTime = 0.0;
        }

        if (lastPreTime > absTime && lastPreTime - absTime > 1.0) {
            ResetHistory();
            m_dofType = in.dofType;
            g_logger.Warn("absTime %f, m_lastPreTime %f time invalid reset!!!!!!!\n", absTime, 0.0);
            lastPreTime = m_lastPreTime;
        }

        if (lastPreTime > absTime) {
            // Requested time is already covered by the history: look it up or interpolate.
            int idx = m_head;
            if (idx >= 0) {
                const PosePredictState* newer = nullptr;
                for (int i = 0; i < m_count; ++i, --idx) {
                    if (idx < 0)
                        idx += m_capacity;
                    const PosePredictState& entry = m_history[idx];
                    const double entryTime = TimestampToSeconds(entry.timestamp);
                    if (std::fabs(absTime - entryTime) < 0.0005) {
                        if (out)
                            *out = entry;
                        return;
                    }
                    if (entryTime > absTime) {
                        newer = &entry;
                        continue;
                    }
                    if (!newer)
                        continue;

                    const float a = static_cast<float>((absTime - entryTime) /
                                                       (TimestampToSeconds(newer->timestamp) - entryTime));
                    PosePredictState interp = *newer;
                    interp.timestamp = result.timestamp;
                    for (int k = 0; k < 3; ++k)
                        interp.position[k] = entry.position[k] * (1.0f - a) + newer->position[k] * a;
                    m_poseFilter->BlendDerivatives(a, &interp, &entry);

                    Eigen::Quaternionf older;
                    older.coeffs() << entry.rotation[0], entry.rotation[1], entry.rotation[2], entry.rotation[3];
                    Eigen::Quaternionf newerQ;
                    newerQ.coeffs() << newer->rotation[0], newer->rotation[1], newer->rotation[2],
                        newer->rotation[3];
                    const Eigen::Quaternionf q = older.slerp(a, newerQ);
                    for (int k = 0; k < 4; ++k)
                        interp.rotation[k] = q.coeffs()[k];
                    result = interp;
                    break;
                }
            }
        } else {
            // Advance the filter in 5 ms steps up to the requested time.
            const double end = absTime - 0.005;
            double t = std::max(inTime, lastPreTime + 0.0001);
            if (predictTime > 0.005 && end > t) {
                do {
                    Step(in, static_cast<float>(t - inTime), &result);
                    t += 0.005;
                } while (end > t);
            }
            Step(in, predictTime, &result);
            m_lastPreTime = absTime;
        }
    }

    if (out)
        *out = result;
}

// ---------------------------------------------------------------------------------------------

void PredictorManager::ReleaseAll() {
    for (auto& predictor : predictors)
        predictor.ReleaseFilter();
}
