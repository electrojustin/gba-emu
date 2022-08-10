#include <string>

#ifndef CORE_LOGGING_H
#define CORE_LOGGING_H

namespace Core {

enum LogLevel {
  info = 0,
  warning,
  error,
  fatal,
};

void log(LogLevel level, const char* format, ...);

void set_log_level(LogLevel level);

} // namespace Core

#endif
