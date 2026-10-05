#include "PoseFilter.h"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <cmath>

#include "Logger.h"

namespace {

Eigen::Quaternionf GetRotation(const PosePredictState& s) {
    Eigen::Quaternionf q;
    q.coeffs() << s.rotation[0], s.rotation[1], s.rotation[2], s.rotation[3];
    return q;
}

void SetRotation(PosePredictState& s, const Eigen::Vector4f& coeffs) {
    for (int i = 0; i < 4; ++i)
        s.rotation[i] = coeffs[i];
}

double TimestampToSeconds(uint64_t ns) {
    return static_cast<double>(ns) / 1000000000.0;
}

// 0x180003D20: weighted quaternion average (Markley et al.). Each row of `Q` is (w, x, y, z).
// Returns the eigenvector of the largest eigenvalue in Eigen coefficient order (x, y, z, w).
Eigen::Vector4f AverageQuaternions(Eigen::MatrixXf Q, Eigen::VectorXf w) {
    Eigen::Matrix4f A = Eigen::Matrix4f::Zero();
    const int count = static_cast<int>(Q.rows());
    const float weightSum = w.sum();
    for (int i = 0; i < count; ++i) {
        Eigen::Quaternionf q(Q(i, 0), Q(i, 1), Q(i, 2), Q(i, 3));
        A += w(i) * q.coeffs() * q.coeffs().transpose();
    }
    A /= weightSum;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4f> solver(A);
    return solver.eigenvectors().col(3);
}

}  // namespace

PoseFilter::PoseFilter(int deviceType) : m_deviceType(deviceType), m_lastOutput() {
    m_logDir = "/home/vino/work/Project/controller/0803_controller/";
    m_predictedPosePath = m_logDir + "predicted_high_slam_pose.txt";
    m_inputPosePath = "DLL_input_real_time_pose.txt";
    m_savePosePath = "/data/save_real_time_pose.txt";
    m_filteredPosePath = m_logDir + "filtered_high_slam_pose.txt";

    if (deviceType == kPredictDeviceHmd || deviceType == kPredictDeviceHmd2) {
        g_logger.Info("Device type is HMD! %d\n", deviceType);
        g_logger.Warn("beta is  %f\n", m_hmdPosBeta);
        for (int i = kPosX; i <= kPosZ; ++i)
            m_filters[i].reset(new OneEuroFilter(m_hmdPosFreq, m_hmdPosMinCutoff, m_hmdPosBeta, m_hmdPosDCutoff,
                                                 m_hmdWindowSize));
        for (int i = kRotX; i <= kRotW; ++i)
            m_filters[i].reset(new OneEuroFilter(m_hmdRotFreq, m_hmdRotMinCutoff, m_hmdRotBeta, m_hmdRotDCutoff,
                                                 m_hmdWindowSize));
    }
    if (deviceType == kPredictDeviceLeftController || deviceType == kPredictDeviceRightController) {
        g_logger.Info("Device type is Controller! %d \n", deviceType);
        for (int i = 0; i < kNumFilters; ++i)
            m_filters[i].reset(
                new OneEuroFilter(m_ctrlFreq, m_ctrlMinCutoff, m_ctrlBeta, m_ctrlDCutoff, m_ctrlWindowSize));
    }
}

PoseFilter::~PoseFilter() = default;

void PoseFilter::Reset() {
    m_quatInitialized = false;
    m_lastFilterTime = -1.0;
    m_lastPredictTime = -1.0;
    for (auto& filter : m_filters)
        filter->Reset();
}

PosePredictState* PoseFilter::Predict(PosePredictState* out, PosePredictState in, float dt) {
    *out = in;

    const double timestampNs = static_cast<double>(in.timestamp);
    if (timestampNs == 0.0) {
        g_logger.Error("ERROR! TimeStamp is 0.0 or pose_quality is not good \n");
        return out;
    }
    if (in.dofType == 1 && in.position[0] == 0.0f && in.position[1] == 0.0f && in.position[2] == 0.0f) {
        g_logger.Error("ERROR! translation is 0.0 \n");
        return out;
    }

    const double t = timestampNs / 1000000000.0;
    if (m_lastPredictTime == -1.0) {
        m_lastPredictTime = t;
    } else if (t - m_lastPredictTime > 0.1f) {
        // The binary prints the *filter* timestamp here, not m_lastPredictTime.
        g_logger.Warn("Prediction time gap is too large! %f - %f = %f \n", t, m_lastFilterTime, t - m_lastFilterTime);
        m_lastPredictTime = t;
        return out;
    }

    Eigen::Vector3f angVel(in.angularVelocity[0], in.angularVelocity[1], in.angularVelocity[2]);
    Eigen::Vector3f angAcc(in.angularAcceleration[0], in.angularAcceleration[1], in.angularAcceleration[2]);
    Eigen::Vector3f omega = angVel + angAcc * dt;

    Eigen::Quaternionf q = GetRotation(in);
    Eigen::Vector3f pos(in.position[0], in.position[1], in.position[2]);

    const float speed = omega.norm();
    if (speed > 0.005f) {
        const float angle = speed * dt;
        Eigen::Vector3f axis = omega.normalized();
        Eigen::Quaternionf delta(Eigen::AngleAxisf(angle, axis));
        q = GetRotation(in).normalized() * delta;
        for (int i = 0; i < 3; ++i)
            pos[i] = in.linearAcceleration[i] * 0.5f * dt * dt + (in.linearVelocity[i] * dt + in.position[i]);
    }

    SetRotation(*out, q.coeffs());
    out->position[0] = pos.x();
    out->position[1] = pos.y();
    out->position[2] = pos.z();
    m_lastPredictTime = t;
    return out;
}

void PoseFilter::BlendDerivatives(float t, PosePredictState* dst, const PosePredictState* src) const {
    const float s = 1.0f - t;
    for (int i = 0; i < 3; ++i) {
        dst->angularVelocity[i] = s * src->angularVelocity[i] + dst->angularVelocity[i] * t;
        dst->linearVelocity[i] = dst->linearVelocity[i] * t + s * src->linearVelocity[i];
        dst->angularAcceleration[i] = s * src->angularAcceleration[i] + dst->angularAcceleration[i] * t;
        // Original bug preserved: linear acceleration is blended with src's *linear velocity*.
        dst->linearAcceleration[i] = dst->linearAcceleration[i] * t + s * src->linearVelocity[i];
    }
}

PosePredictState* PoseFilter::Filter(PosePredictState* out, PosePredictState in, float predictTime) {
    const double timestampNs = static_cast<double>(in.timestamp);
    if (in.poseQuality < 1.0f || timestampNs == 0.0) {
        g_logger.Error("ERROR! TimeStamp is 0.0 or pose_quality is not good \n");
        *out = in;
        return out;
    }
    if (in.dofType == 1 && in.position[0] == 0.0f && in.position[1] == 0.0f && in.position[2] == 0.0f) {
        g_logger.Error("ERROR! translation is 0.0 \n");
        *out = in;
        return out;
    }

    const double t = timestampNs / 1000000000.0 + predictTime;
    if (m_lastFilterTime == -1.0) {
        m_lastFilterTime = t;
    } else if (t - m_lastFilterTime > 0.1) {
        g_logger.Warn("filter time gap is too large! %f - %f = %f \n", t, m_lastFilterTime, t - m_lastFilterTime);
        m_lastFilterTime = t;
        *out = in;
        return out;
    }

    const float ft = static_cast<float>(t);
    const float px = m_filters[kPosX]->Filter(in.position[0], ft, false);
    const float py = m_filters[kPosY]->Filter(in.position[1], ft, false);
    const float pz = m_filters[kPosZ]->Filter(in.position[2], ft, false);
    const float qw = m_filters[kRotW]->Filter(in.rotation[3], ft, true);
    const float qx = m_filters[kRotX]->Filter(in.rotation[0], ft, true);
    const float qy = m_filters[kRotY]->Filter(in.rotation[1], ft, true);
    const float qz = m_filters[kRotZ]->Filter(in.rotation[2], ft, true);

    Eigen::Vector4f current(qx, qy, qz, qw);
    Eigen::MatrixXf quats;
    quats.resize(2, 4);
    Eigen::VectorXf weights(2);
    if (m_quatInitialized) {
        if (current.dot(m_lastQuat) < 0.0f)
            current = -current;
        quats << m_lastQuat.w(), m_lastQuat.x(), m_lastQuat.y(), m_lastQuat.z(),
                 current.w(), current.x(), current.y(), current.z();
        weights << 0.05f, 0.95f;
    } else {
        m_lastQuat = current;
        m_quatInitialized = true;
        quats << m_lastQuat.w(), m_lastQuat.x(), m_lastQuat.y(), m_lastQuat.z(),
                 current.w(), current.x(), current.y(), current.z();
        weights << 0.5f, 0.5f;
    }

    const Eigen::Vector4f averaged = AverageQuaternions(quats, weights);
    m_lastQuat = current;
    m_lastFilterTime = t;

    *out = in;
    SetRotation(*out, averaged);
    out->position[0] = px;
    out->position[1] = py;
    out->position[2] = pz;
    return out;
}

void PoseFilter::Process(const PosePredictState& in, float predictTime, PosePredictState* out) {
    PosePredictState predicted = in;
    m_currentTime = TimestampToSeconds(in.timestamp) + predictTime;
    if (predictTime > 0.0001) {
        PosePredictState tmp;
        predicted = *Predict(&tmp, in, predictTime);
    }

    if (m_predictedLog) {
        m_predictedLog << std::fixed << predicted.timestamp << " " << predicted.position[0] << " "
                       << predicted.position[1] << " " << predicted.position[2] << " " << predicted.rotation[0]
                       << " " << predicted.rotation[1] << " " << predicted.rotation[2] << " "
                       << predicted.rotation[3] << "\n";
    }

    const uint8_t lastDof = m_lastOutput.dofType;
    const bool isController =
        m_deviceType == kPredictDeviceLeftController || m_deviceType == kPredictDeviceRightController;
    if (predicted.dofType == lastDof && isController && m_lastOutput.timestamp != 0) {
        const Eigen::Vector3f angVel(in.angularVelocity[0], in.angularVelocity[1], in.angularVelocity[2]);
        const float speed = angVel.norm();

        if (speed < 0.02f) {
            // Controller is (almost) still: hold the previous output and zero all derivatives.
            m_lastOutput.timestamp = in.timestamp;
            for (int i = 0; i < 3; ++i) {
                m_lastOutput.angularVelocity[i] = 0.0f;
                m_lastOutput.linearVelocity[i] = 0.0f;
                m_lastOutput.angularAcceleration[i] = 0.0f;
                m_lastOutput.linearAcceleration[i] = 0.0f;
            }
            *out = m_lastOutput;
            m_lastOutputTime = m_currentTime;
            m_lastOutputTimestamp = m_lastOutput.timestamp;
            return;
        }

        if (speed < 0.2f) {
            // Slow motion: blend from the previous output towards the prediction.
            const float t = (speed - 0.02f) / 0.18f;
            const Eigen::Quaternionf q = GetRotation(m_lastOutput).slerp(t, GetRotation(predicted));
            q.normalized();  // result discarded in the binary as well
            Eigen::Vector3f pos;
            for (int i = 0; i < 3; ++i)
                pos[i] = t * predicted.position[i] + m_lastOutput.position[i] * (1.0f - t);
            BlendDerivatives(t, &predicted, &m_lastOutput);

            SetRotation(predicted, q.coeffs());
            predicted.position[0] = pos.x();
            predicted.position[1] = pos.y();
            predicted.position[2] = pos.z();
            *out = predicted;
            m_lastOutput = predicted;
            m_lastOutputTime = m_currentTime;
            m_lastOutputTimestamp = predicted.timestamp;
            return;
        }
    }

    if (m_lastOutputTime == -1.0) {
        m_lastOutputTime = m_currentTime;
    } else if (m_lastOutputTime > m_currentTime) {
        // Time went backwards: pass the prediction through unfiltered.
        *out = predicted;
        m_lastOutput = predicted;
        m_lastOutputTime = m_currentTime;
        m_lastOutputTimestamp = predicted.timestamp;
        return;
    }

    if (lastDof != predicted.dofType)
        Reset();

    PosePredictState filtered;
    Filter(&filtered, predicted, predictTime);
    *out = filtered;
    m_lastOutput = filtered;
    m_lastOutputTime = m_currentTime;
    m_lastOutputTimestamp = filtered.timestamp;

    if (m_filteredLog) {
        m_filteredLog << std::fixed << filtered.timestamp << " " << filtered.position[0] << " "
                      << filtered.position[1] << " " << filtered.position[2] << " " << filtered.rotation[0] << " "
                      << filtered.rotation[1] << " " << filtered.rotation[2] << " " << filtered.rotation[3]
                      << "\n";
    }
}
