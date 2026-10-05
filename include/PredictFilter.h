// Public interface of PredictFilterDLL.dll (reconstructed).
//
// Version string of the original binary:
//   POSE_PREDICT_24_08.23.0.2_3ecd640eacbf145856d151eac6eaa1c9f24a4803
#pragma once

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#  if defined(PREDICTFILTER_BUILD)
#    define PREDICTFILTER_API __declspec(dllexport)
#  else
#    define PREDICTFILTER_API __declspec(dllimport)
#  endif
#else
#  define PREDICTFILTER_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Device types accepted by PredictInitialImpl / PosePredict.
enum PredictDeviceType {
    kPredictDeviceHmd = 0,
    kPredictDeviceLeftController = 1,
    kPredictDeviceRightController = 2,
    kPredictDeviceHmd2 = 4,  // treated as HMD by the filter
};

// 96-byte pose state exchanged with the caller. Layout must match the binary.
typedef struct PosePredictState {
    float rotation[4];             // +0x00 quaternion x, y, z, w (Eigen::Quaternionf layout)
    float position[3];             // +0x10
    float angularVelocity[3];      // +0x1C
    float linearVelocity[3];       // +0x28
    float angularAcceleration[3];  // +0x34
    float linearAcceleration[3];   // +0x40
    uint32_t reserved;             // +0x4C (padding)
    uint64_t timestamp;            // +0x50 nanoseconds
    float poseQuality;             // +0x58 must be > 0 to be processed
    uint8_t dofType;               // +0x5C 1 == 6dof (translation must be non-zero)
    uint8_t pad[3];                // +0x5D
} PosePredictState;

// Ordinal 1
PREDICTFILTER_API const char* GetPredictVersion(void);
// Ordinal 2. `predictTime` is in seconds (clamped to 0.05). `out` may be NULL.
PREDICTFILTER_API int PosePredict(const PosePredictState* in, float predictTime, PosePredictState* out,
                                  int deviceType);
// Ordinal 3
PREDICTFILTER_API int PredictInitialImpl(int deviceType);
// Ordinal 4
PREDICTFILTER_API int PredictReleaseImpl(void);

#ifdef __cplusplus
}  // extern "C"

static_assert(sizeof(PosePredictState) == 96, "PosePredictState must be 96 bytes");
static_assert(offsetof(PosePredictState, timestamp) == 0x50, "timestamp offset");
static_assert(offsetof(PosePredictState, poseQuality) == 0x58, "poseQuality offset");
static_assert(offsetof(PosePredictState, dofType) == 0x5C, "dofType offset");
#endif
