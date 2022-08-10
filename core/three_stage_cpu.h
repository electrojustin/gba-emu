#include <memory>
#include <mutex>

#include "core/future.h"

#ifndef CORE_THREE_STAGE_CPU_H
#define CORE_THREE_STAGE_CPU_H

namespace Core {

// Framework for a generic three stage CPU pipeline.
public
class ThreeStageCPU {
 private:
  void cycle();

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
  // This provides us a way to temporarily halt the processor, whether for an
  // interrupt or a DMA transfer.
  std::mutex wait_future_lock;
  std::shared_ptr<Future> wait_future = Future::immediate_future();

  virtual void reset() = 0;
}

}  // namespace Core

#endif
