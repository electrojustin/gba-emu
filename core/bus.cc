#include "core/bus.h"

#include "core/clock.h"
#include "core/future.h"
#include "core/log.h"

namespace Core {

std::shared_ptr<Future> Bus::execute_next_request() {
  lock.lock();
  auto request = bus_requests.pop_front();
  lock.unlock();

  // TODO: replace with binary search or page tables?
  std::shared_ptr<BusListener> listener = nullptr;
  for (auto curr_listener : listeners) {
    if (curr_listener.start_addr <= request.addr &&
        curr_listener.end_addr > request.addr) {
      listener = curr_listener;
      break;
    }
  }

  if (!listener) {
    log(LogLevel::error, "Error! Invalid address %x\n", request.addr);

    // The GBA doesn't have an MMU, so invalid address reads won't fault,
    // they'll read the open bus, which will likely contain the last value on
    // the data line. Some games actually rely on this behavior, which is why
    // this is a non-fatal error.
    *request.data = last_data;
    request.request_completion_future->make_available();
  } else {
    auto read_write_future =
        listener->read_write(request.addr, request.size, request.data);
    read_write_future->add_listener([=, &clock, &this]() {
      last_data = *request.data;

      // Make the data available on the next falling edge and then block the
      // rising edge until the requester processes the data.
      clock->register_falling_edge_trigger([=]() {
        request.request_completion_future->make_available();
        return request.bus_activity_future;
      });

      // Note that we immediately start the next request to support burst mode
      // transfers.
      this->lock.lock();
      int num_queued_requests = bus_requests.size();
      this->lock.unlock();
      if (num_queued_requests)
        clock->register_falling_edge_trigger(
            std::bind(Bus::execute_next_request, this));
    });
  }

  return Future::immediate_future();
}

void Bus::register_listener(std::shared_ptr<BusListener> listener) {
  listeners.emplace_back(listener);
}

std::shared_ptr<Future> Bus::request(
    uint64_t addr,
    ReadWrite dir,
    DataSize size,
    std::shared_ptr<uint64_t> data,
    std::shared_ptr<Future> bus_activity_future) {
  BusRequest req;

  req.addr = addr;
  req.dir = dir;
  req.size = size;
  req.data = data;
  req.bus_activity_future = bus_activity_future;

  lock.lock();
  bus_requests.push_back(req);
  bool start_requests = bus_requests.size() == 1;
  lock.unlock();

  if (start_requests)
    clock->register_falling_edge_trigger(
        std::bind(Bus::execute_next_request, this));

  return req.request_completion_future;
}

}  // namespace Core
