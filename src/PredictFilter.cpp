// Exported entry points.
#include "PredictFilter.h"

#include "Logger.h"
#include "Predictor.h"

namespace {

Predictor* PredictorFor(int deviceType) {
    switch (deviceType) {
    case kPredictDeviceHmd:
        return &g_predictorManager.predictors[0];
    case kPredictDeviceLeftController:
        return &g_predictorManager.predictors[1];
    case kPredictDeviceRightController:
        return &g_predictorManager.predictors[2];
    case kPredictDeviceHmd2:
        return &g_predictorManager.predictors[3];
    default:
        return nullptr;
    }
}

}  // namespace

// 0x180014830
extern "C" int PredictInitialImpl(int deviceType) {
    Predictor* predictor = PredictorFor(deviceType);
    if (predictor)
        predictor->Init(deviceType);
    else
        g_logger.Error("device_typ %d not supported\n", deviceType);
    return 0;
}

// 0x1800148D0
extern "C" const char* GetPredictVersion(void) {
    return "POSE_PREDICT_24_08.23.0.2_3ecd640eacbf145856d151eac6eaa1c9f24a4803";
}

// 0x1800148E0
extern "C" int PosePredict(const PosePredictState* in, float predictTime, PosePredictState* out, int deviceType) {
    if (deviceType == kPredictDeviceHmd || deviceType == kPredictDeviceHmd2) {
        g_logger.Info(
            "++caputure input_state: device-type %d %lld %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f "
            "%f\n",
            deviceType, in->timestamp, in->position[0], in->position[1], in->position[2], in->rotation[0],
            in->rotation[1], in->rotation[2], in->rotation[3], in->linearAcceleration[0], in->linearAcceleration[1],
            in->linearAcceleration[2], in->angularVelocity[0], in->angularVelocity[1], in->angularVelocity[2],
            in->linearVelocity[0], in->linearVelocity[1], in->linearVelocity[2], in->angularAcceleration[0],
            in->angularAcceleration[1], in->angularAcceleration[2], static_cast<double>(predictTime));
    }

    Predictor* predictor = PredictorFor(deviceType);
    if (!predictor) {
        g_logger.Error("device_typ %d not supported\n", deviceType);
        return 0;
    }
    predictor->Predict(*in, predictTime, out);

    if (deviceType == kPredictDeviceHmd2) {
        g_logger.Warn(
            "caputure output_state: device-type %d %lld %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f "
            "%f\n",
            4, out->timestamp, out->position[0], out->position[1], out->position[2], out->rotation[0],
            out->rotation[1], out->rotation[2], out->rotation[3], out->linearAcceleration[0],
            out->linearAcceleration[1], out->linearAcceleration[2], out->angularVelocity[0], out->angularVelocity[1],
            out->angularVelocity[2], out->linearVelocity[0], out->linearVelocity[1], out->linearVelocity[2],
            out->angularAcceleration[0], out->angularAcceleration[1], out->angularAcceleration[2],
            static_cast<double>(predictTime));
    }
    return 0;
}

// 0x180014D60
extern "C" int PredictReleaseImpl(void) {
    g_predictorManager.ReleaseAll();
    return 0;
}

#ifdef _WIN32
#include <windows.h>

// 0x18000F920
BOOL APIENTRY DllMain(HMODULE, DWORD, LPVOID) {
    return TRUE;
}
#endif
