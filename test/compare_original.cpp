// Differential test (Windows / wine): loads the original PredictFilterDLL.dll and the rebuilt one
// side by side, drives both with the same pseudo-random pose stream and compares every output.
//
//   compare_original.exe <original.dll> <rebuilt.dll>
#include <windows.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>

#include "PredictFilter.h"

typedef const char* (*GetVersionFn)();
typedef int (*PosePredictFn)(const PosePredictState*, float, PosePredictState*, int);
typedef int (*InitFn)(int);
typedef int (*ReleaseFn)();

struct Api {
    GetVersionFn version;
    PosePredictFn predict;
    InitFn init;
    ReleaseFn release;
};

static bool Load(const char* path, Api* api) {
    HMODULE m = LoadLibraryA(path);
    if (!m) {
        std::printf("cannot load %s (%lu)\n", path, GetLastError());
        return false;
    }
    api->version = (GetVersionFn)GetProcAddress(m, "GetPredictVersion");
    api->predict = (PosePredictFn)GetProcAddress(m, "PosePredict");
    api->init = (InitFn)GetProcAddress(m, "PredictInitialImpl");
    api->release = (ReleaseFn)GetProcAddress(m, "PredictReleaseImpl");
    return api->version && api->predict && api->init && api->release;
}

static float Ulps(float a, float b) {
    if (a == b || (std::isnan(a) && std::isnan(b)))
        return 0;
    int32_t ia, ib;
    std::memcpy(&ia, &a, 4);
    std::memcpy(&ib, &b, 4);
    if (ia < 0) ia = 0x80000000 - ia;
    if (ib < 0) ib = 0x80000000 - ib;
    return static_cast<float>(std::abs(static_cast<int64_t>(ia) - ib));
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::printf("usage: %s original.dll rebuilt.dll\n", argv[0]);
        return 2;
    }
    Api orig, re;
    if (!Load(argv[1], &orig) || !Load(argv[2], &re))
        return 2;
    std::printf("orig: %s\nre:   %s\n", orig.version(), re.version());

    const int devices[] = {0, 1, 2, 4};
    for (int d : devices) {
        orig.init(d);
        re.init(d);
    }

    std::mt19937 rng(1234);
    std::uniform_real_distribution<float> uni(-1.0f, 1.0f);

    uint64_t ts[4] = {5000000000ull, 5000000000ull, 5000000000ull, 5000000000ull};
    float angle[4] = {0, 0, 0, 0};
    long long compared = 0, mismatches = 0;
    float worstUlps = 0;
    float worstAbs = 0;
    long long exact = 0;

    for (int frame = 0; frame < 20000; ++frame) {
        for (int di = 0; di < 4; ++di) {
            const int dev = devices[di];
            // Mostly regular 2 ms steps, with occasional jitter, backwards jumps and big gaps.
            const int r = static_cast<int>(rng() % 1000);
            if (r < 5)
                ts[di] -= 30000000ull;  // go back 30 ms (history lookup)
            else if (r < 7)
                ts[di] += 200000000ull;  // 200 ms gap
            else
                ts[di] += 2000000ull + (rng() % 400000);

            // Phases: moving, slow, still (controller special cases).
            const int phase = (frame / 700) % 3;
            const float speed = phase == 0 ? 1.5f : (phase == 1 ? 0.1f : 0.005f);
            angle[di] += speed * 0.002f;

            PosePredictState in;
            std::memset(&in, 0, sizeof(in));
            const float ax = 0.3f, ay = 0.9f, az = 0.3f;
            const float n = std::sqrt(ax * ax + ay * ay + az * az);
            const float s = std::sin(angle[di] / 2) / n;
            in.rotation[0] = ax * s + uni(rng) * 1e-4f;
            in.rotation[1] = ay * s;
            in.rotation[2] = az * s;
            in.rotation[3] = std::cos(angle[di] / 2);
            in.position[0] = 0.2f * std::sin(angle[di]) + uni(rng) * 1e-3f;
            in.position[1] = 1.6f + uni(rng) * 1e-3f;
            in.position[2] = -0.3f + 0.1f * std::cos(angle[di]);
            for (int k = 0; k < 3; ++k) {
                in.angularVelocity[k] = speed * (k == 1 ? ay : ax) / n + uni(rng) * 0.01f;
                in.linearVelocity[k] = uni(rng) * 0.2f;
                in.angularAcceleration[k] = uni(rng) * 0.5f;
                in.linearAcceleration[k] = uni(rng) * 0.5f;
            }
            in.timestamp = ts[di];
            in.poseQuality = (rng() % 200 == 0) ? 0.0f : ((rng() % 100 == 0) ? 0.5f : 1.0f);
            in.dofType = (frame % 5000 < 4990) ? 1 : 0;
            if (rng() % 3000 == 0)
                in.position[0] = in.position[1] = in.position[2] = 0.0f;
            const float predictTime = (rng() % 10 == 0) ? 0.0f : 0.001f + (rng() % 60) * 0.001f;

            PosePredictState a, b;
            std::memset(&a, 0xCD, sizeof(a));
            std::memset(&b, 0xCD, sizeof(b));
            orig.predict(&in, predictTime, &a, dev);
            re.predict(&in, predictTime, &b, dev);

            const float* fa = reinterpret_cast<const float*>(&a);
            const float* fb = reinterpret_cast<const float*>(&b);
            bool bad = a.timestamp != b.timestamp || a.dofType != b.dofType;
            float frameWorst = 0;
            for (int k = 0; k < 19; ++k) {
                const float u = Ulps(fa[k], fb[k]);
                frameWorst = u > frameWorst ? u : frameWorst;
                const float ad = std::fabs(fa[k] - fb[k]);
                worstAbs = ad > worstAbs ? ad : worstAbs;
                if (u > 64 && std::fabs(fa[k] - fb[k]) > 1e-5f)
                    bad = true;
            }
            worstUlps = frameWorst > worstUlps ? frameWorst : worstUlps;
            ++compared;
            if (std::memcmp(&a, &b, 93) == 0)
                ++exact;
            if (bad) {
                if (++mismatches <= 10) {
                    std::printf("MISMATCH frame %d dev %d ts %llu/%llu\n", frame, dev, (unsigned long long)a.timestamp,
                                (unsigned long long)b.timestamp);
                    for (int k = 0; k < 19; ++k)
                        std::printf("  [%2d] %.9g %.9g\n", k, fa[k], fb[k]);
                }
            }
        }
    }
    orig.release();
    re.release();
    std::printf("compared %lld outputs, %lld bit-exact, %lld mismatches, worst ULP %.0f, worst abs diff %g\n", compared, exact,
                mismatches, worstUlps, worstAbs);
    return mismatches ? 1 : 0;
}
