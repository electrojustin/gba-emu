#include <stdint.h>

#include "arm/arm7_tdmi.h"
#include "arm/decoded_insn.h"

#ifndef ARM_ARM_INSN_H
#define ARM_ARM_INSN_H

namespace ARM {

public
class ARMInsn : public DecodedInsn {
protected:
  virtual std::shared_ptr<Core::Future> internal_exec() = 0;

 public:
  virtual const static bool is_parseable(uint32_t encoded_insn) = 0;
  ARMCondition cond = ARMCondition::Always;
  std::shared_ptr<Core::Future> exec();
};

std::shared_ptr<ARMInsn> get_arm_insn(ARM7TDMI& processor,
                                      uint32_t encoded_insn);

}  // namespace ARM

#endif
