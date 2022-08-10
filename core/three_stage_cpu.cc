#include "core/three_stage_cpu.h"

#include "core/logging.h"

namespace Core {

void ThreeStageCPU::cycle() {
  wait_future_lock.lock();
  wait_future->add_listener([=, &this]() {
    auto bus_activity_future = std::make_shared<Future>();

    auto fetch_future = fetch(bus_activity_future);
    fetch_future->add_listener([=, &this]() {
      this->decode();

      auto exec_future = this->exec();
      exec_future->add_listener(std::bind(ThreeStageCPU::cycle, this));

      bus_activity_future->make_available();
    });
  });
  wait_future_lock.unlock();
}

}  // namespace Core
