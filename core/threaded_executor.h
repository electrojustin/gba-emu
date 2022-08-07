#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "core/executor.h"

#ifndef CORE_THREADED_EXECUTOR_H
#define CORE_THREADED_EXECUTOR_H

namespace Core {

public
class ThreadedExecutor {
 private:
  int num_threads;
  bool is_running;
  std::mutex work_queue_lock;
  std::queue<std::function<void()>> work_queue;
  std::mutex worker_thread_lock;
  std::vector<std::thread> worker_threads;

  void work();
  void maybe_spawn_worker();

 public:
  ThreadedExecutor(int num_threads);

  void exec(std::function<void()> to_exec) override;
  void start(std::function<void()> start_task) override;
  void shutdown() override;
};

}  // namespace Core

#endif
