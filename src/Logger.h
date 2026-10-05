// Minimal printf-style logger (global instance at 0x18001C038).
#pragma once

#include <string>

class Logger {
public:
    enum Level { kDebug = 1, kInfo = 2, kWarn = 3, kError = 4 };

    Logger(bool enabled, int level, const std::string& tag) : m_enabled(enabled), m_level(level), m_tag(tag) {}

    void Info(const char* fmt, ...);   // 0x180002660, printed when level <= 2
    void Warn(const char* fmt, ...);   // 0x180002770, printed when level <= 3
    void Error(const char* fmt, ...);  // 0x180002880, printed when level <= 4

private:
    bool m_enabled;     // +0
    int m_level;        // +4
    std::string m_tag;  // +8
};

// Enabled, level 3 (warnings and errors only), tag "Predict alg:".
extern Logger g_logger;
