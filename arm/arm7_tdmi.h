#include <stdint.h>
#include <memory>

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
  SignedLessOrEqual
};

public
class ARM7TDMI : public Core::ThreeStageCPU {
 private:
  // Register definitions
  uint32_t registers[16] = {0};
  uint32_t& stack_pointer = registers[13];
  uint32_t& link_register = registers[14];
  uint32_t& program_counter = registers[15];
  uint32_t cpsr = 0;
  uint32_t vtor = 0;

  // Helper functions for getting and setting bits in CPSR, the ARM equivalent
  // of "flags".
  int get_ring();  // Should always be 0 for GBA
  void set_ring(int ring);
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

  friend class DecodedInsn;

  std::shared_ptr<Core::Future> fetch(
      std::shared_ptr<Core::Future> bus_activity_future) override;
  void decode() override;
  std::shared_ptr<Core::Future> exec() override;

 public:
  void reset() override;
}

}  // namespace ARM

#endif
