// 1-Euro filter (Casiez et al.) with an additional 2-tap output window.
#pragma once

#include <deque>
#include <memory>

// 16 bytes.
class LowPassFilter {
public:
    explicit LowPassFilter(float alpha, float initval = 0.0f);

    void SetAlpha(float alpha);
    float Filter(float value);
    float FilterWithAlpha(float value, float alpha);
    bool HasLastRawValue() const { return m_initialized; }
    float LastRawValue() const { return m_y; }
    void Reset() { m_initialized = false; }

private:
    float m_y;           // +0  last raw value
    float m_a;           // +4  alpha
    float m_s;           // +8  last filtered value
    bool m_initialized;  // +12
};

// 0x60 bytes.
class OneEuroFilter {
public:
    // 0x180001600
    OneEuroFilter(float freq, float mincutoff, float beta, float dcutoff, int windowSize);

    // 0x1800018F0. When `bypassWindow` is false the output goes through a 2-tap window
    // (0.05 * older + 0.95 * newer) once `windowSize` samples are available.
    float Filter(float value, float timestamp, bool bypassWindow);

    // Inlined in 0x180003580.
    void Reset();

private:
    float Alpha(float cutoff) const;
    void SetFrequency(float f);
    void SetMinCutoff(float mc);
    void SetBeta(float b) { m_beta = b; }
    void SetDerivateCutoff(float dc);

    float m_freq;                         // +0x00
    float m_mincutoff;                    // +0x04
    float m_beta;                         // +0x08
    float m_dcutoff;                      // +0x0C
    std::unique_ptr<LowPassFilter> m_x;   // +0x10
    std::unique_ptr<LowPassFilter> m_dx;  // +0x18
    float m_lasttime;                     // +0x20
    std::deque<float> m_window;           // +0x28
    int m_windowSize;                     // +0x50
};
