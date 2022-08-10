#include "arm/arm7_tdmi.h"

#include <string.h>

#include "core/logging.h"

namespace ARM {

namespace {

int get_trailing_zeros(uint32_t x) {
  int ret = 0;

  for (int i = 0; i < 32; i++) {
    if (x & 0x01)
      break;

    ret++;
    x >>= 1;
  }

  return ret;
}

}  // namespace

#define BOOLEAN_CPSR_FLAG(name, bit_num)                      \
  bool ARM7TDMI::get_name() { return cpsr & (1 << bit_num); } \
                                                              \
  void ARM7TDMI::set_name(bool val) {                         \
    cpsr &= ~(1 << bit_num);                                  \
    cpsr |= val << bit_num;                                   \
  }

int ARM7TDMI::get_ring() {
  return cpsr & 0x1F;
}

void ARM7TDMI::set_ring(int ring) {
  cpsr &= ~0x1F;
  cpsr |= (ring & 0x1F);
}

BOOLEAN_CPSR_FLAG(thumb, 5)
BOOLEAN_CPSR_FLAG(fast_interrupt_disable, 6)
BOOLEAN_CPSR_FLAG(irq_disable, 7)
BOOLEAN_CPSR_FLAG(imprecise_data_abort_disable, 8)
BOOLEAN_CPSR_FLAG(little_endian, 9)

ARMCondition ARM7TDMI::get_it_condition() {
  int ret = (cpsr >> 25) & 0x03;
  ret <<= 1;
  ret |= (cpsr >> 15) & 0x01;

  return ret;
}

void ARM7TDMI::set_it_condition(ARMCondition cond) {
  cpsr &= ~(0x03 << 25);
  cpsr |= ((cond & 0x07) >> 1) << 25;
  cpsr &= ~(0x01 << 15);
  cpsr |= (cond & 0x01) << 15;
}

int get_it_length() {
  int it_block = (cpsr >> 10) & 0x1F;
  return 4 - get_trailing_zeros(it_block);
}

void set_it_length(int len) {
  if (len > 4 || len < 0)
    Core::log(LogLevel::fatal, "Invalid Thumb if-then length %d\n", len);

  cpsr &= ~(0x1F << 10);
  cpsr |= 1 << (10 + (4 - len));
}

bool get_it_inverted(int index) {
  if (index > get_it_length() - 1)
    Core::log(LogLevel::fatal, "Thumb if-then index out of bounds\n");

  return cpsr & (1 << (11 + (4 - index)));
}

void set_it_inverted(int index, bool inv) {
  if (index > get_it_length() - 1)
    Core::log(LogLevel::fatal, "Thumb if-then index out of bounds\n");

  cpsr &= ~(1 << (11 + (4 - index)));
  cpsr |= inv << (11 + (4 - index));
}

int get_greater_or_equal() {
  return (cpsr >> 16) & 0x0F;
}

void set_greater_or_equal(int val) {
  cpsr &= ~(0x0F << 16);
  cpsr |= ((val & 0x0F) << 16);
}

BOOLEAN_CPSR_FLAG(java_state, 24)
BOOLEAN_CPSR_FLAG(sticky_overflow, 27)
BOOLEAN_CPSR_FLAG(overflow, 28)
BOOLEAN_CPSR_FLAG(carry, 29)
BOOLEAN_CPSR_FLAG(zero, 30)
BOOLEAN_CPSR_FLAG(negative, 31)

std::shared_ptr<Core::Future> ARM7TDMI::fetch(
    std::shared_ptr<Core::Future> bus_activity_future) {
  auto data = std::make_shared<uint64_t> data;
  Core::DataSize data_size =
      get_thumb() ? Core::DataSize::Word : Core::DataSize::DoubleWord;
  auto ret = instruction_bus->request(program_counter, Core::ReadWrite::read,
                                      data_size, data, bus_activity_future);

  ret->add_listener((=, &this)[] {
    if (data_size == Core::DataSize::Word) {
      encoded_insn_buf = *data & 0xFFFF;
    } else {
      encoded_insn_buf = *data & 0xFFFFFFFF;
    });

    program_counter += data_size == Core::DataSize::Word ? 2 : 4;

    return ret;
}

void ARM7TDMI::decode() {
    if (encoded_insn)
      decoded_insn_buf = get_decoded_insn(*this, encoded_insn);

    encoded_insn = encoded_insn_buf;
}

std::shared_ptr<Core::Future> ARM7TDMI::exec() {
    std::shared_ptr<Core::Future> ret = nullptr;
    if (decoded_insn) {
      ret = decoded_insn->exec();
    } else {
      ret = Core::Future::immediate_future();
    }

    decoded_insn = decoded_insn_buf;

    return ret;
}

void ARM7TDMI::reset() {
  memset(registers, 0, sizeof(registers);
  cpsr = 0;
  set_little_endian(true);
  vtor = 0;
}

}  // namespace ARM
