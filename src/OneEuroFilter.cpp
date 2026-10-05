#include "OneEuroFilter.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

static const double kTwoPi = 6.283185307179586;
static const float kUndefinedTime = -1.0f;

LowPassFilter::LowPassFilter(float alpha, float initval) : m_y(initval), m_a(0.0f), m_s(initval) {
    SetAlpha(alpha);
    m_initialized = false;
}

void LowPassFilter::SetAlpha(float alpha) {
    if (alpha <= 0.0f || alpha > 1.0f)
        throw std::range_error("alpha should be in (0.0., 1.0]");
    m_a = alpha;
}

float LowPassFilter::Filter(float value) {
    float result;
    if (m_initialized) {
        result = (1.0f - m_a) * m_s + m_a * value;
    } else {
        result = value;
        m_initialized = true;
    }
    m_y = value;
    m_s = result;
    return result;
}

float LowPassFilter::FilterWithAlpha(float value, float alpha) {
    SetAlpha(alpha);
    return Filter(value);
}

OneEuroFilter::OneEuroFilter(float freq, float mincutoff, float beta, float dcutoff, int windowSize)
    : m_lasttime(kUndefinedTime) {
    SetFrequency(freq);
    SetMinCutoff(mincutoff);
    SetBeta(beta);
    SetDerivateCutoff(dcutoff);
    if (windowSize <= 0)
        throw std::range_error("Window size should be >0");
    m_windowSize = windowSize;
    m_x.reset(new LowPassFilter(Alpha(mincutoff)));
    m_dx.reset(new LowPassFilter(Alpha(dcutoff)));
}

float OneEuroFilter::Alpha(float cutoff) const {
    float te = static_cast<float>(1.0 / m_freq);
    float tau = static_cast<float>(1.0 / (cutoff * kTwoPi));
    return static_cast<float>(1.0 / (tau / te + 1.0));
}

void OneEuroFilter::SetFrequency(float f) {
    if (f <= 0.0f)
        throw std::range_error("freq should be >0");
    m_freq = f;
}

void OneEuroFilter::SetMinCutoff(float mc) {
    if (mc <= 0.0f)
        throw std::range_error("mincutoff should be >0");
    m_mincutoff = mc;
}

void OneEuroFilter::SetDerivateCutoff(float dc) {
    if (dc <= 0.0f)
        throw std::range_error("dcutoff should be >0");
    m_dcutoff = dc;
}

void OneEuroFilter::Reset() {
    m_window.clear();
    m_x->Reset();
    m_dx->Reset();
    m_lasttime = kUndefinedTime;
}

float OneEuroFilter::Filter(float value, float timestamp, bool bypassWindow) {
    float lasttime = m_lasttime;
    if (lasttime == kUndefinedTime) {
        m_lasttime = timestamp;
        lasttime = timestamp;
    }
    if (timestamp != kUndefinedTime && timestamp > lasttime) {
        m_lasttime = timestamp;
        m_freq = 1.0f / std::max(0.0001f, timestamp - lasttime);
    }

    float dvalue = m_x->HasLastRawValue() ? (value - m_x->LastRawValue()) * m_freq : 0.0f;
    float edvalue = m_dx->FilterWithAlpha(dvalue, Alpha(m_dcutoff));
    float cutoff = std::fabs(edvalue) * m_beta + m_mincutoff;
    float filtered = m_x->FilterWithAlpha(value, Alpha(cutoff));
    if (bypassWindow)
        return filtered;

    m_window.push_back(filtered);
    const size_t windowSize = static_cast<size_t>(m_windowSize);
    if (m_window.size() < windowSize)
        return filtered;
    if (m_window.size() > windowSize)
        m_window.pop_front();

    float result = 0.0f;
    if (m_window.size() == windowSize)
        result = m_window[0] * 0.05f + m_window[1] * 0.95f;
    return result;
}
