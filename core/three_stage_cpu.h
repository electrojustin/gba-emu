#include <memory>

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
  virtual std::shared_ptr<Future> fetch(
      std::shared_ptr<Future> bus_activity_future) = 0;
  virtual void decode() = 0;
  virtual std::shared_ptr<Future> exec() = 0;

 public:
  virtual void reset() = 0;
}

}  // namespace Core

#endif
