#include <memory>
#include <stdint.h>

#include "core/bus.h"
#include "core/clock.h"
#include "core/logging.h"

#ifndef CORE_MEMORY_H
#define CORE_MEMORY_H

namespace Core {

public class MemoryModule : public BusListener {
private:
  uint8_t* backing_memory;

  uint64_t last_addr = 0;
  DataSize last_size = DataSize::Byte;

  // Whether or not this memory module is ROM. Will warn and discard writes.
  bool is_read_only;

  // Maximum data access width. Will warn if we attempt to access more.
  DataSize max_data_width;

  // Access timings
  int byte_cycles;
  int word_cycles;
  int dword_cycles;
  int qword_cycles;
  int sequential_byte_cycles;
  int sequential_word_cycles;
  int sequential_dword_cycles;
  int sequential_qword_cycles;

  uint64_t read(uint64_t addr, DataSize size);
  void write(uint64_t addr, uint64_t val, DataSize size);

public:
  MemoryModule(uint64_t start_addr,
               uint64_t end_addr,
               bool is_read_only,
               DataSize max_data_width,
               int byte_cycles,
               int word_cycles,
               int dword_cycles,
               int qword_cycles,
               int sequential_byte_cycles,
               int sequential_word_cycles,
               int sequential_dword_cycles,
               int sequential_qword_cycles);

  ~MemoryModule();

  std::shared_ptr<Future> read_write(uint64_t addr,
                                     ReadWrite dir,
                                     DataSize size,
                                     std::shared_ptr<uint64_t> data) override;
}

} // namespace Core

#endif
