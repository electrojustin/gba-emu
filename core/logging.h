#include <string>

#ifndef CORE_LOGGING_H
#define CORE_LOGGING_H

enum LogLevel {
  info = 0,
  warning,
  error,
  fatal,
};

void log(LogLevel level, const char* format, ...);

void set_log_level(LogLevel level);

#endif
