#include "logging.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

namespace Core {

LogLevel curr_level = LogLevel::error;

void log(LogLevel level, const char* format, ...) {
  if (level >= curr_level) {
    va_list args;
    va_start(args, format);

    vprintf(format, args);

    va_end(args);
  }

  if (level == LogLevel::fatal)
    exit(-1);
}

void set_log_level(LogLevel level) {
  curr_level = level;
}

}  // namespace Core
