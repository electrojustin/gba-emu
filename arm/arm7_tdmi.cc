#include "arm/arm7_tdmi.h"

#include <stmode.h>

#include "core/clock.h"
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

uint32_t& ARM7TDMI::access_reg(int index) {
  if (index < 8) {
    return r0_r7[index];
  } else if (index < 13) {
    if (get_mode() == ARMMode::FIQ) {
      return r8_r12_fiq[index - 8];
    } else {
      return r8_r12_fiq[index - 8];
    }
  } else if (index == 13) {
    switch (get_mode()) {
      case ARMMode::SVC:
        return r13_svc;
      case ARMMode::Abort:
        return r13_abt;
      case ARMMode::UndefinedInsn:
        return r13_und;
      case ARMMode::IRQ:
        return r13_irq;
      case ARMMode::FIQ:
        return r13_fiq;
      default:
        return r13;
    }
  } else if (index == 14) {
    switch (get_mode()) {
      case ARMMode::SVC:
        return r14_svc;
      case ARMMode::Abort:
        return r14_abt;
      case ARMMode::UndefinedInsn:
        return r14_und;
      case ARMMode::IRQ:
        return r14_irq;
      case ARMMode::FIQ:
        return r14_fiq;
      default:
        return r14;
    }
  } else if (index == 15) {
    return r15;
  } else {
    log(LogLevel::fatal, "Invalid register R%d\n", index);
  }
}

uint32_t& ARM7TDMI::access_spsr() {
  switch (get_mode()) {
    case ARMMode::SVC:
      return spsr_svc;
    case ARMMode::Abort:
      return spsr_abt;
    case ARMMode::UndefinedInsn:
      return spsr_und;
    case ARMMode::IRQ:
      return spsr_irq;
    case ARMMode::FIQ:
      return spsr_fiq;
    default:
      log(LogLevel::fatal, "Cannot access SPSR in User/Sys mode\n");
      return 0;
  }
}

#define BOOLEAN_CPSR_FLAG(name, bit_num)                      \
  bool ARM7TDMI::get_name() { return cpsr & (1 << bit_num); } \
                                                              \
  void ARM7TDMI::set_name(bool val) {                         \
    cpsr &= ~(1 << bit_num);                                  \
    cpsr |= val << bit_num;                                   \
  }

ARMMode ARM7TDMI::get_mode() {
  return cpsr & 0x1F;
}

void ARM7TDMI::set_mode(ARMMode mode) {
  cpsr &= ~0x1F;
  cpsr |= (mode & 0x1F);
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

void ARM7TDMI::flush_pipeline() {
    encoded_insn = 0;
    encoded_insn_buf = 0;
    decoded_insn = nullptr;
    decoded_insn_buf = nullptr;
}

void ARM7TDMI::interrupt(ARMInterruptType type) {
    // Halt processor until we load up the new program counter
    auto interrupt_wait_future = std::make_shared<Core::Future>();
    wait_future_lock.lock();
    wait_future = interrupt_wait_future;
    wait_future_lock.unlock();

    is_interrupting = true;

    Core::clock->register_rising_edge_listener([=, &this]() {
      // Change mode appropriately
      switch (type) {
        case ARMInterruptType::Reset:
        case ARMInterruptType::Software this->set_mode(ARMMode::SVC); break;
            case ARMInterruptType::UndefinedInsn:
          this->set_mode(ARMMode::Undef);
          break;
        case ARMInterruptType::PrefetchAbort:
        case ARMInterruptType::DataAbort:
          this->set_mode(ARMMode::Abort);
          break;
        case ARMInterruptType::Slow:
          this->set_mode(ARMMode::IQR);
          break;
        case ARMInterruptType::Fast:
          this->set_mode(ARMMode::FIQ);
          break;
        default:
          log(LogLevel::fatal, "Invalid interrupt type!\n");
          break;
      }

      // Save return register
      this->access_link() = this->access_pc();

      // Save CPSR
      this->access_spsr() = this->cpsr;

      // Flush execution pipeline
      this->flush_pipeline();

      auto new_program_counter = std::make_shared<uint64_t>();
      auto read_future = this->instruction_bus->request(
          vtor + type * 4, Core::ReadWrite::read, Core::DataSize::DoubleWord,
          new_program_counter, interrupt_wait_future);
      read_future->add_listener([=, &this]() {
        this->access_pc() = *new_program_counter;
        this->is_interrupting = false;
        interrupt_wait_future->make_available();

        return Core::Future::immediate_future();
      });

      return Core::Future::immediate_future();
    });
}

void ARM7TDMI::reset() {
  memset(registers, 0, sizeof(registers);
  cpsr = 0;
  set_little_endian(true);
  vtor = 0;
  wait_future = Core::Future::immediate_future();
}

}  // namespace ARM
