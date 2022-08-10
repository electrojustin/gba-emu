#include "core/three_stage_cpu.h"

#include "core/logging.h"

namespace Core {

void ThreeStageCPU::cycle() {
  auto bus_activity_future = std::make_shared<Future>();

  auto fetch_future = fetch(bus_activity_future);
  fetch_future->add_listener((=, &this)[] {
    this->decode();

    auto exec_future = this->exec();
    exec_future->add_listener(std::bind(ThreeStageCPU::cycle, this));

    bus_activity_future->make_available();
  });
}

void ThreeStageCPU::boot(uint64_t init_program_counter) {
  if (!is_running) {
    program_counter = init_program_counter;
    is_running = true;
    cycle();
  } else {
    log(LogLevel::fatal, "Error! Attempted to boot already running CPU!\n");
  }
}

}  // namespace Core
