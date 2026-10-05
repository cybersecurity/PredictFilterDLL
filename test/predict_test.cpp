// Smoke test: feeds a rotating / translating HMD and controller pose through the library.
#include <cmath>
#include <cstdio>
#include <cstring>

#include "PredictFilter.h"

static PosePredictState MakePose(uint64_t tsNs, float t) {
    PosePredictState s;
    std::memset(&s, 0, sizeof(s));
    const float angle = 0.5f * t;  // rad, about Y
    s.rotation[1] = std::sin(angle / 2);
    s.rotation[3] = std::cos(angle / 2);
    s.position[0] = 0.1f * t;
    s.position[1] = 1.6f;
    s.position[2] = -0.2f;
    s.angularVelocity[1] = 0.5f;
    s.linearVelocity[0] = 0.1f;
    s.timestamp = tsNs;
    s.poseQuality = 1.0f;
    s.dofType = 1;
    return s;
}

int main() {
    std::printf("version: %s\n", GetPredictVersion());
    const int devices[] = {0, 1, 2, 4};
    for (int d : devices)
        PredictInitialImpl(d);
    PredictInitialImpl(3);  // unsupported, logs an error

    int failures = 0;
    const uint64_t start = 1000000000ull;
    for (int i = 0; i < 200; ++i) {
        const uint64_t ts = start + static_cast<uint64_t>(i) * 2000000ull;  // 500 Hz
        const float t = (ts - start) / 1e9f;
        PosePredictState in = MakePose(ts, t);
        for (int d : devices) {
            PosePredictState out;
            PosePredict(&in, 0.02f, &out, d);
            const float n = std::sqrt(out.rotation[0] * out.rotation[0] + out.rotation[1] * out.rotation[1] +
                                      out.rotation[2] * out.rotation[2] + out.rotation[3] * out.rotation[3]);
            if (!(std::fabs(n - 1.0f) < 1e-3f) || std::isnan(out.position[0])) {
                std::printf("bad output dev %d frame %d |q|=%f\n", d, i, n);
                ++failures;
            }
            if (i == 199)
                std::printf("dev %d: ts=%llu pos=(%.4f %.4f %.4f) q=(%.4f %.4f %.4f %.4f)\n", d,
                            static_cast<unsigned long long>(out.timestamp), out.position[0], out.position[1],
                            out.position[2], out.rotation[0], out.rotation[1], out.rotation[2], out.rotation[3]);
        }
    }
    PredictReleaseImpl();
    std::printf("%s (%d failures)\n", failures ? "FAIL" : "OK", failures);
    return failures ? 1 : 0;
}
