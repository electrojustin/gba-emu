// Generic interface for scheduling and running callback functions.

#include <functional>
#include <memory>

#ifndef CORE_EXECUTOR_H
#define CORE_EXECUTOR_H

namespace Core {

public
class Executor {
 public:
  virtual void exec(std::function<void()> to_exec) = 0;
  virtual void start(std::function<void()> start_task) = 0;
  virtual void shutdown() = 0;
};

extern std::unique<Executor> executor;

}  // namespace Core

#endif
