#include "core/clock.h"
#include "core/logging.h"
#include "core/memory.h"

namespace Core {

uint64_t MemoryModule::read(uint64_t addr, DataSize size) {
  switch (size) {
    case DataSize::Byte:
      return backing_memory[addr - start_addr];
    case DataSize::Word:
      return *(uint16_t*)(backing_memory + (addr - start_addr));
    case DataSize::DoubleWord:
      return *(uint32_t*)(backing_memory + (addr - start_addr));
    case DataSize::QuadWord:
      return *(uint64_t*)(backing_memory + (addr - start_addr));
    default:
      log(LogLevel::fatal, "Invalid DataSize %d\n", size);
      return -1;
  }
}

void MemoryModule::write(uint64_t addr, uint64_t val, DataSize size) {
  if (is_read_only) {
    log(LogLevel::error, "Writing to read only address %x\n", addr);
    return;
  }

  switch (size) {
    case DataSize::Byte:
      backing_memory[addr - start_addr] = val & 0xFF;
      break;
    case DataSize::Word:
      *(uint16_t*)(backing_memory + (addr - start_addr)) = val & 0xFFFF;
      break;
    case DataSize::DoubleWord:
      *(uint32_t*)(backing_memory + (addr - start_addr)) = val & 0xFFFFFFFF;
      break;
    case DataSize::QuadWord:
      *(uint64_t*)(backing_memory + (addr - start_addr)) = val;
      break;
    default:
      log(LogLevel::fatal, "Invalid DataSize %d\n", size);
      break;
  }
}

MemoryModule::MemoryModule(uint64_t start_addr,
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
               int sequential_qword_cycles) {
  this->start_addr = start_addr;
  this->end_addr = end_addr;
  backing_memory = (uint8_t*)malloc(end_addr - start_addr);

  this->is_read_only = is_read_only;
  this->max_data_width = max_data_width;
  this->byte_cycles = byte_cycles;
  this->word_cycles = word_cycles;
  this->dword_cycles = dword_cycles;
  this->qword_cycles = qword_cycles;
  this->sequential_byte_cycles = sequential_byte_cycles;
  this->sequential_word_cycles = sequential_word_cycles;
  this->sequential_dword_cycles = sequential_dword_cycles;
  this->sequential_qword_cycles = sequential_qword_cycles;
}

MemoryModule::~MemoryModule() {
  free(backing_memory);
}

std::shared_ptr<Future> MemoryModule::read_write(uint64_t addr,
                                                 ReadWrite dir,
                                                 DataSize size,
                                                 std::shared_ptr<uint64_t> data) {
  auto ret = std::make_shared<Future>();
  int cycles = 0;

  if (size > max_data_width)
    size = max_data_width;

  if (addr == last_addr + last_size) {
    switch (size) {
      case DataSize::Byte:
        cycles = sequential_byte_cycles;
        break;
      case DataSize::Word:
        cycles = sequential_word_cycles;
        break;
      case DataSize::DoubleWord:
        cycles = sequential_dword_cycles;
        break;
      case DataSize::QuadWord:
        cycles = sequential_qword_cycles;
        break;
      default:
        log(LogLevel::fatal, "Invalid DataSize %d\n", size);
        break;
    }
  } else {
    switch (size) {
      case DataSize::Byte:
        cycles = byte_cycles;
        break;
      case DataSize::Word:
        cycles = word_cycles;
        break;
      case DataSize::DoubleWord:
        cycles = dword_cycles;
        break;
      case DataSize::QuadWord:
        cycles = qword_cycles;
        break;
      default:
        log(LogLevel::fatal, "Invalid DataSize %d\n", size);
        break;
    }
  }

  clock->register_rising_edge_listener([=]() {
    if (dir == ReadWrite::read) {
      *data = read(addr, size);
    } else {
      write(addr, *data, size);
    }

    last_addr = addr;
    size = size;

    ret->make_available();
    return Futures::immediate_future();
  }, cycles - 1);

  return ret;
}

} // namespace Core
