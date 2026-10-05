#include "Logger.h"

#include <cstdarg>
#include <cstdio>

static const std::string kLogTag = "Predict alg:";  // 0x180001000
Logger g_logger(true, Logger::kWarn, kLogTag);       // 0x180001030

namespace {

void Emit(const std::string& tag, const char* levelName, const char* fmt, va_list args) {
    char message[256];
    char line[256];
    int n = vsnprintf(message, sizeof(message), fmt, args);
    int m = snprintf(line, 0xFF, "%s", message);
    if (n < 0)
        n = -1;
    if (n >= 0 && m >= 0) {
        printf("%s %s %s", tag.c_str(), levelName, line);
        fflush(stdout);
    }
}

}  // namespace

void Logger::Info(const char* fmt, ...) {
    if (!m_enabled || m_level > kInfo)
        return;
    va_list args;
    va_start(args, fmt);
    Emit(m_tag, "[info]", fmt, args);
    va_end(args);
}

void Logger::Warn(const char* fmt, ...) {
    if (!m_enabled || m_level > kWarn)
        return;
    va_list args;
    va_start(args, fmt);
    Emit(m_tag, "[warn]", fmt, args);
    va_end(args);
}

void Logger::Error(const char* fmt, ...) {
    if (!m_enabled || m_level > kError)
        return;
    va_list args;
    va_start(args, fmt);
    Emit(m_tag, "[error]", fmt, args);
    va_end(args);
}
