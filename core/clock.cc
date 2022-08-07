#include "clock.h"

#include "executor.h"

namespace Core {

std::unique_ptr<Clock> clock;

std::shared_ptr<Future> Clock::trigger_edge(std::queue<std::function<std::shared_ptr<Future>()>>& listeners) {
  auto ret = std::make_shared<Future>();

  lock.lock();

  auto rising_edge_outstanding = std::make_shared<std::atomic<int>>(listeners.size());

  while(!listeners.empty()) {
    auto listener = std::make_shared(std::move(rising_edge_listeners.pop_front()));
    auto task = [=]() {
      (*listener)().register_listener([=]() {
        if (rising_edge_oustanding.fetch_sub() == 1)
          ret->make_available();
      });
    };
    executor->exec(task);
  }

  // Note that this means any register_*_edge_listener methods will register for the next clock cycle.
  lock.unlock();

  return ret;
}

std::shared_ptr<Future> Clock::rising_edge() {
  return trigger_edge(rising_edge_listeners);
}

std::shared_ptr<Future> Clock::falling_edge() {
  return trigger_edge(falling_edge_listeners);
}

void Clock::register_rising_edge_listener(std::function<std::shared_ptr<Future>()> listener) {
  lock.lock();

  rising_edge_listeners.push_back(std::move(listener));

  lock.unlock();
}

void Clock::register_falling_edge_listener(std::function<std::shared_ptr<Future>()> listener) {
  lock.lock();

  falling_edge_listeners.push_back(std::move(listener));

  lock.unlock();
}

void register_rising_edge_listener(std::function<std::shared_ptr<Future>()> listener, int delay) {
  auto delayed_listener = listener;
  for (int i = 0; i < delay; i++) {
    auto new_delayed_listener = [=]() {
      register_falling_edge_listener([=]() {
        register_rising_edge_listener(delayed_listener);

        return Future::immediate_future();
     });

      return Future::immediate_future();
    };
    
    delayed_listener = new_delayed_listener;
  }

  register_rising_edge_listener(delayed_listener);
}

void register_falling_edge_listener(std::function<std::shared_ptr<Future>()> listener, int delay) {
  auto delayed_listener = listener;
  for (int i = 0; i < delay; i++) {
    auto new_delayed_listener = [=]() {
      register_rising_edge_listener([=]() {
        register_falling_edge_listener(delayed_listener);

        return Future::immediate_future();
     });

      return Future::immediate_future();
    };
    
    delayed_listener = new_delayed_listener;
  }

  register_falling_edge_listener(delayed_listener);
}

} // namespace Core
