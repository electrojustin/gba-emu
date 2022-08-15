#include <stdint.h>
#include <memory>
#include <mutex>

#include "arm/decoded_insn.h"
#include "core/bus.h"
#include "core/three_stage_cpu.h"

#ifndef ARM_ARM7_TDMI_H
#define ARM_ARM7_TDMI_H

namespace ARM {

enum ARMCondition {
  Equal = 0,
  NotEqual,
  CarrySet,
  CarryClear,
  Negative,
  Positive,
  OverflowSet,
  OverflowClear,
  UnsignedGreater,
  UnsignedLessOrEqual,
  SignedGreaterOrEqual,
  SignedLess,
  SignedGreater,
  SignedLessOrEqual,
  Always,
  Never,
};

enum ARMMode {
  User = 0b10000,
  FIQ = 0b10001,
  IRQ = 0b10010,
  SVC = 0b10011,
  Abort = 0b10111,
  Undef = 0b11011,
  System = 0b11111,
};

enum ARMInterruptType {
  Reset = 0,
  UndefinedInsn,
  Software,
  PrefetchAbort,
  DataAbort,
  Slow,
  Fast,
};

public
class ARM7TDMI : public Core::ThreeStageCPU {
 private:
  // Register backings
  uint32_t r0_r7[8];
  uint32_t r8_r12[5];
  uint32_t r8_r12_fiq[5];
  uint32_t r13, r13_svc, r13_abt, r13_und, r13_irq, r13_fiq;
  uint32_t r14, r14_svc, r14_abt, r14_und, r14_irq, r14_fiq;
  uint32_t r15;
  uint32_t cpsr = 0;
  uint32_t spsr_svc, spsr_abt, spsr_und, spsr_irq, spsr_fiq = 0;
  uint32_t vtor = 0;

  // Helper functions to get the right register for the right ring.
  uint32_t& access_reg(int index);
  uint32_t& access_spsr();
  uint32_t& access_pc() { return access_reg(15); }
  uint32_t& access_link() { return access_reg(14); }
  uint32_t access_sp() { return access_reg(13); }

  // Helper functions for getting and setting bits in CPSR, the ARM equivalent.
  // of "flags".
  ARMMode get_mode();
  void set_mode(ARMMode mode);
  bool get_thumb();
  void set_thumb(bool val);
  bool get_fast_interrupt_disable();
  void set_fast_interrupt_disable(bool val);
  bool get_irq_disable();
  void set_irq_disable(bool val);
  bool
  get_imprecise_data_abort_disable();  // Never used, GBA doesn't have an MMU.
  void set_imprecise_data_abort_disable(bool val);
  bool get_little_endian();  // Should always be true for GBA.
  void set_little_endian(bool val);
  ARMCondition get_it_condition();
  void set_it_condition(ARMCondition cond);
  int get_it_length();
  // Note that this will clear the ITSTATE bits corresponding to the block
  // state.
  void set_it_length(int len);
  bool get_it_inverted(int index);
  void set_it_inverted(int index, bool inv);
  // Technically this is 4 bits, to support SIMD greater or equal to
  // comparisons, but the GBA doesn't have an SIMD unit, so all four bits will
  // be the same value.
  int get_greater_or_equal();
  void set_greater_or_equal(int val);
  bool get_java_state();  // Unused in GBA
  void set_java_state(bool val);
  bool get_sticky_overflow();
  void set_sticky_overflow(bool val);
  bool get_overflow();
  void set_overflow(bool val);
  bool get_carry();
  void set_carry(bool val);
  bool get_zero();
  void set_zero(bool val);
  bool get_negative();
  void set_negative(bool val);

  // Note that these will point to the same bus on GBA.
  std::shared_ptr<Core::Bus> instruction_bus;
  std::shared_ptr<Core::Bus> data_bus;

  // State of the instruction pipeline.
  uint32_t encoded_insn = 0;
  uint32_t encoded_insn_buf = 0;
  std::shared_ptr<DecodedInsn> decoded_insn = nullptr;
  std::shared_ptr<DecodedInsn> decoded_insn_buf = nullptr;

  std::atomic_bool is_interrupting = false;

  friend class DecodedInsn;

  std::shared_ptr<Core::Future> fetch(
      std::shared_ptr<Core::Future> bus_activity_future) override;
  void decode() override;
  std::shared_ptr<Core::Future> exec() override;

  void flush_pipeline();

 public:
  void interrupt(ARMInterruptType type);

  void reset() override;
}

}  // namespace ARM

#endif
