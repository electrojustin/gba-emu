#include <atomic>
#include <memory>
#include <mutex>

#include "core/clock.h"
#include "core/future.h"

#ifndef CORE_THREE_STAGE_CPU_H
#define CORE_THREE_STAGE_CPU_H

namespace Core {

// Framework for a generic three stage CPU pipeline.
public
class ThreeStageCPU {
protected:
  void cycle();

  virtual void process_interrupt() = 0;

  // This future helps us make sure we don't start the next clock cycle until
  // after we start the next fetch.
  std::shared_ptr<Future> exec_bus_activity_future = nullptr;

  // Fetch is expected to return a future that is made available when the
  // instruction fetch is complete. The bus completion future will be made
  // available by the cycle() method after exec() is called, although not
  // necessarily when the exec() future returns.
  // Note: execution of an instruction should finish on a falling edge.
  virtual std::shared_ptr<Future> fetch(
      std::shared_ptr<Future> bus_activity_future) = 0;
  virtual void decode() = 0;
  virtual std::shared_ptr<Future> exec() = 0;

 public:
   atomic_bool wait;

   atomic_bool is_interrupting;

   virtual void reset() = 0;
}

}  // namespace Core

#endif
