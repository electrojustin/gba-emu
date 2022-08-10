#include "core/future.h"

#ifndef ARM_DECODED_INSN_H
#define ARM_DECODED_INSN_H

namespace ARM {

public
class DecodedInsn {
 public:
  ARM7TDMI& processor;

  virtual std::shared_ptr<Core::Future> exec() = 0;
};

std::shared_ptr<DecodedInsn> get_decoded_insn(ARM7TDMI& processor,
                                              uint32_t encoded_insn);

}  // namespace ARM

#endif
