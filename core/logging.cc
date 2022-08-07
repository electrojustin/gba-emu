#include "logging.h"

#include <stdarg.h>
#include <stdio.h>

LogLevel curr_level = LogLevel::error;

void log(LogLevel level, const char* format, ...) {
  if (level >= curr_level) {
    va_list args;
    va_start(args, format);

    vprintf(format, args);

    va_end(args);
  }
}

void set_log_level(LogLevel level) {
  curr_level = level;
}
