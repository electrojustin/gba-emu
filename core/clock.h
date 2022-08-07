#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>

#include "future.h"

#ifndef CORE_CLOCK_H
#define CORE_CLOCK_H

namespace Core {

public
class Clock {
 private:
  std::mutex lock;
  std::queue<std::function<std::shared_ptr<Future>()>> rising_edge_listeners;
  std::queue<std::function<std::shared_ptr<Future>()>> falling_edge_listeners;

  std::shared_ptr<Future> trigger_edge(
      std::queue<std::function<Future()>>& listeners);

 public:
  // Note that because of the locking mechanics, listeners that register new
  // listeners will be doing so for the next edge. So for example, if a falling
  // edge listener calls register_falling_edge_listener, that callback will be
  // execute at the *next* falling edge, not the current one.
  void register_rising_edge_listener(
      std::function<std::shared_ptr<Future>()> listener);
  void register_falling_edge_listener(
      std::function<std::shared_ptr<Future>()> listener);

  // Helper functions for introducing a delay of N cycles
  void register_rising_edge_listener(
      std::function<std::shared_ptr<Future>()> listener,
      int delay);
  void register_falling_edge_listener(
      std::function<std::shared_ptr<Future>()> listener,
      int delay);

  // rising_edge's Future will become available once all activity has settled
  std::shared_ptr<Future> rising_edge();

  // falling_edge's Future will become available once all activity has settled
  std::shared_ptr<Future> falling_edge();
};

extern std::unique_ptr<Clock> clock;

}  // namespace Core

#endif
