#include <stdint.h>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>

#include "future.h"

#ifndef CORE_BUS_H
#define CORE_BUS_H

namespace Core {

enum ReadWrite {
  read,
  write,
};

enum DataSize {
  Byte,
  Word,
  DoubleWord,
  QuadWord,
};

struct BusRequest {
  uint64_t addr;
  ReadWrite dir;
  DataSize size;
  std::shared_ptr<uint64_t> data;
  std::shared_ptr<Future> request_completion_future;
  std::shared_ptr<Future> bus_activity_future;
};

// BusListeners should operate on the rising edge.
public
class BusListener {
 public:
  uint64_t start_addr;
  uint64_t end_addr;

  virtual std::shared_ptr<Future> read_write(
      uint64_t addr,
      ReadWrite dir,
      DataSize size,
      std::shared_ptr<uint64_t> data) = 0;
};

// Buses should operate on the falling edge.
public
class Bus {
 private:
  uint64_t last_data = 0;
  std::mutex lock;
  std::vector<std::shared_ptr<BusListener>> listeners;
  std::queue<BusRequest> bus_requests;

  std::shared_ptr<Future> execute_next_request();

 public:
  void register_listener(std::shared_ptr<BusListener> listener);

  // Note that we pass in a bus activity future. This is because we need to know when the requester is done with the bus information so we can move on to the next clock cycle.
  std::shared_ptr<Future> request(uint64_t addr,
                                  ReadWrite dir,
                                  DataSize size,
                                  std::shared_ptr<uint64_t> data,
                                  std::shared_ptr<Future> bus_activity_future);
};

} // namespace Core

#endif
