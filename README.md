# PredictFilterDLL – reconstructed source

Source reconstruction of `PredictFilterDLL.dll`
(`POSE_PREDICT_24_08.23.0.2_3ecd640eacbf145856d151eac6eaa1c9f24a4803`, x64, MSVC, sha256 `a166f274…7cdd3a`).
The DLL predicts and smooths HMD and controller poses. It uses constant-acceleration extrapolation,
per-axis 1-Euro filters and weighted quaternion averaging, and keeps a per-device history ring buffer.

## Build

Eigen 3.4.0 is a git submodule in `third_party/eigen`, pinned to tag `3.4.0` (commit `3147391d946b`)
of https://gitlab.com/libeigen/eigen.

```sh
git clone --recursive https://github.com/cybersecurity/PredictFilterDLL
# or, in an existing clone:
git submodule update --init --depth 1

# Windows (MSVC)
cmake -S . -B build -A x64 && cmake --build build --config Release
# Linux (for testing)
cmake -S . -B build-linux -DCMAKE_BUILD_TYPE=Release && cmake --build build-linux
# MinGW cross build
cmake -S . -B build-mingw -DCMAKE_TOOLCHAIN_FILE=<mingw toolchain> -DCMAKE_BUILD_TYPE=Release
```

`src/PredictFilterDLL.def` keeps the export ordinals identical to the original (1–4).

### Eigen version

Eigen doesn't embed a version string, so the version was dated from which upstream changes appear
in the compiled code:

| Upstream change | Seen in the binary | Implication |
|---|---|---|
| `90187a33e` (2021-03-25) new tridiagonal-QR deflation test `(e / eps)² <= \|d_i\| + \|d_i+1\|` | yes (`* 8388608.0` in 0x180006E90) | 3.4.0 or newer (not 3.3.x) |
| `c29a41011` (2024-01-12) null check before `free` in `aligned_free` | no (`free(0)` is still called in `PoseFilter::Filter`) | older than 5.0 / master 2024 |
| `d2dce3776` (2025-04-21) `slerp()` uses `sqrt(1-d²)` instead of `sin(acos d)` | no (`acosf` + 3× `sinf`) | older than 5.0 |

That leaves 3.4.0, 3.4.1, or a master snapshot from 2021-03 to 2024-01. The 3.4.1 changes that
touch the code in this binary don't change the generated code, so the binary can't tell 3.4.0 from
3.4.1. 3.4.0 is the likely choice: the version string (`POSE_PREDICT_24_08.23…`) points to an
August 2024 build, and 3.4.1 wasn't released until September 2025. Building against 3.4.0 instead
of an older snapshot (the 2018 PX4 fork) raises bit-exact outputs in the differential test from
39 692 to 43 168 of 80 000.

## Verification

`test/compare_original.cpp` loads the original DLL and the rebuilt DLL side by side and feeds both
80 000 pseudo-random inputs. The inputs cover all 4 device types, history lookups (backwards time),
>0.1 s gaps, bad pose quality, zero translation, dof changes, and the still and slow controller paths.
Result with the MinGW build under wine:

```
compared 80000 outputs, 43168 bit-exact, 0 mismatches, worst ULP 325400, worst abs diff 5.81145e-07
```

The remaining differences are float rounding only (GCC vs. MSVC code generation).

## Layout

| File | Contents |
|---|---|
| `include/PredictFilter.h` | exported C API + 96-byte `PosePredictState` |
| `src/PredictFilter.cpp` | exports `GetPredictVersion`, `PosePredict`, `PredictInitialImpl`, `PredictReleaseImpl` |
| `src/Predictor.*` | per-device predictor (history ring buffer, 5 ms stepping, mutex, FPS counters) |
| `src/PoseFilter.*` | prediction, 1-Euro filtering, quaternion averaging, controller still/slow handling |
| `src/OneEuroFilter.*` | `LowPassFilter` / `OneEuroFilter` with a 2-tap output window |
| `src/Logger.*` | `"%s %s %s"` stdout logger (level 3: warnings and errors only) |

## Address map

| Address | Function |
|---|---|
| 0x180001600 | `OneEuroFilter::OneEuroFilter` |
| 0x1800018F0 | `OneEuroFilter::Filter` |
| 0x180002660 / 2770 / 2880 | `Logger::Info` / `Warn` / `Error` |
| 0x180002990 | `PoseFilter::PoseFilter` |
| 0x180003300 | `PoseFilter::~PoseFilter` |
| 0x180003580 | `PoseFilter::Reset` |
| 0x1800036B0 | `PoseFilter::Predict` |
| 0x180003D20 | `AverageQuaternions` |
| 0x180003FC0 | `PoseFilter::BlendDerivatives` |
| 0x180004220 | `PoseFilter::Filter` |
| 0x180004B00 | `PoseFilter::Process` |
| 0x180006A40 … 0x1800134D0 | Eigen (`SelfAdjointEigenSolver<Matrix4f>`, GEMM helpers, cache queries) |
| 0x180013800 | `FpsCounter::Tick` |
| 0x1800138B0 | `Predictor::Predictor` |
| 0x180013A20 | `Predictor::Init` |
| 0x180013ED0 | `Predictor::Step` |
| 0x180013FE0 | `Predictor::Predict` |
| 0x180014680 | `PredictorManager::ReleaseAll` |
| 0x180014730 | `Predictor::~Predictor` |
| 0x180014830 … 0x180014D60 | exports |
| 0x1800053F0 … 0x180006A20 | inlined `std::basic_filebuf` / `std::ofstream` / `std::string` code |

## Quirks preserved from the original

- `BlendDerivatives` blends `linearAcceleration` with the source's `linearVelocity`.
- `Predict` checks the gap against `m_lastPredictTime` but logs `m_lastFilterTime`.
- The four debug `std::ofstream`s are never opened, so the pose dumps never write anything.
- The result of `q.normalized()` in the slow-controller branch is discarded.
- Timestamps are passed to the 1-Euro filters as `float` seconds.
