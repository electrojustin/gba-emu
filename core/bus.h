#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <stdint.h>

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
};

public class BusListener {
public:
  uint64_t start_addr;
  uint64_t end_addr;

  virtual std::shared_ptr<Future> read_write(uint64_t addr, ReadWrite dir, DataSize size, std::shared_ptr<uint64_t> data) = 0;
};

public class Bus {
private:
  std::mutex lock;
  std::vector<std::shared_ptr<BusListener>> listeners;
  std::queue<BusRequest> bus_requests;

  std::shared_ptr<Future> execute_next_request();

public:
  void register_listener(std::shared_ptr<BusListener> listener);

  std::shared_ptr<Future> request(uint64_t addr, ReadWrite dir, DataSize size, std::shared_ptr<uint64_t> data);
}

#endif
