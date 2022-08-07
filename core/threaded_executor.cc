#include "core/threaded_executor.h"

namespace Core {

ThreadedExecutor::ThreadedExecutor(int num_threads) {
  this->num_threads = num_threads;
  this->is_running = false;
}

void ThreadedExecutor::work() {
  std::function<void()> task = nullptr;
  do {
    work_queue_lock.lock();
    if (work_queue.empty()) {
      task = nullptr;
    } else {
      task = work_queue.pop_front();
    }
    work_queue_lock.unlock();

    if (task)
      task();
  } while (task);
}

void ThreadedExecutor::maybe_spawn_worker() {
  worker_thread_lock.lock();

  if (worker_threads.size() < num_threads - 1) {
    std::thread worker(work);
    worker_threads.emplace_back(std::move(worker));
  }

  worker_thread_lock.unlock();
}

void ThreadedExecutor::start(std::function<void()> start_task) {
  is_running = true;

  work_queue_lock.lock();
  work_queue.emplace_back(std::move(start_task));
  work_queue_lock.unlock();

  work();
}

void ThreadedExecutor::exec(std::function<void()> to_exec) {
  if (is_running) {
    work_queue_lock.lock();
    work_queue.emplace_back(std::move(to_exec));
    work_queue_lock.unlock();
  }
}

void ThreadedExecutor::shutdown() {
  is_running = false;
}

}  // namespace Core
