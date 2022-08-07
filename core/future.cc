#include "future.h"

#include "executor.h"

namespace Core {

Future::Future() {
  is_available = false;
}

Future::Future(Future&& future) {
  lock = std::move(future.lock);
  listeners = std::move(future.listeners);
  is_available = future.is_available;
}

void Future::make_available() {
  lock.lock();

  is_available = true;
  while(!listeners.empty())
    executor->exec(std::move(listeners.pop_front()));

  lock.unlock();
}

void Future::register_listener(std::function<void()> listener) {
  lock.lock();

  if (is_available) {
    executor->exec(std::move(listener));
  } else {
    listeners.push_back(std::move(listener));
  }

  lock.unlock();
}

static std::shared_ptr<Future> Future::immediate_future() {
  if (!immediate) {
    immediate = std::make_shared<Future>();
    immediate->make_available();
  }

  return immediate;
}

} // namespace Core
