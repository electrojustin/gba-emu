#include "arm/arm_insn.h"

#include "core/bitwise.h"
#include "core/clock.h"
#include "core/logging.h"

namespace ARM {

std::shared_ptr<Core::Future> ARMInsn::exec() {
  bool should_execute = true;

  switch (cond) {
    case ARMCondition::Equal:
      should_execute = processor.get_zero();
      break;
    case ARMCondition::NotEqual:
      should_execute = !processor.get_zero();
      break;
    case ARMCondition::CarrySet:
      should_execute = processor.get_carry();
      break;
    case ARMCondition::CarryClear:
      should_execute = !processor.get_carry();
      break;
    case ARMCondition::Negative:
      should_execute = processor.get_negative();
      break;
    case ARMCondition::Positive:
      should_execute = !processor.get_negative();
      break;
    case ARMCondition::OverflowSet:
      should_execute = processor.get_overflow();
      break;
    case ARMCondition::OverflowClear:
      should_execute = !processor.get_overflow();
      break;
    case ARMCondition::UnsignedGreater:
      should_execute = processor.get_carry() && !processor.get_zero();
      break;
    case ARMCondition::UnsignedLessOrEqual:
      should_execute = !processor.get_carry() && processor.get_zero();
      break;
    case ARMCondition::SignedGreaterOrEqual:
      should_execute = processor.get_negative() == processor.get_overflow();
      break;
    case ARMCondition::SignedLess:
      should_execute = processor.get_negative() != processor.get_overflow();
      break;
    case ARMCondition::SignedGreater:
      should_execute = !processor.get_zero() &&
                       (processor.get_negative() == processor.get_overflow());
      break;
    case ARMCondition::SignedLessOrEqual:
      should_execute = processor.get_zero() ||
                       (processor.get_negative() != processor.get_overflow());
      break;
    case ARMCondition::Always:
      should_execute = true;
      break;
    default:
      Core::log(Core::LogLevel::warning, "Invalid conditional %x\n", cond);
      should_execute = false;
      break;
  }

  if (should_execute) {
    return internal_exec();
  } else {
    return Future::immediate_future();
  }
}

/////////////////////////
// Branch Instructions //
/////////////////////////

public
class BranchInsn : public ARMInsn {
 private:
  int32_t offset;
  bool should_link;

 public:
  const static bool is_parseable(uint32_t encoded_insn) override {
    return ((encoded_insn >> 25) & 0x07) == 0b101;
  }

  // Note that the documentation says there's an additional delay of 1
  // sequential and 1 non-sequential memory reads. We don't simulate that here
  // directly because the delay is actually due to the pipeline flush.
  std::shared_ptr<Core::Future> internal_exec() {
    processor.flush_pipeline();
    if (should_link)
      processor.access_link() = processor.access_pc() - 4;
    processor.access_pc() = processor.access_pc() + offset;
    return Core::Future::immediate_future();
  }

  Branch(uint32_t encoded_insn) {
    offset = encoded_insn & 0x00FFFFFF;
    if (offset & 0x00800000)
      offset |= 0xFF000000;
    offset <<= 2;

    should_link = encoded_insn & 0x01000000;
  }
};

// TODO: Add branch exchange once we implement thumb instructions

////////////////
// Interrupts //
////////////////

public
class SoftwareInterruptInsn : public ARMInsn {
 public:
  const static bool is_parseable(uint32_t encoded_insn) {
    return ((encoded_insn >> 24) & 0x0F) == 0b1111;
  }

  // SWI has a similar instruction timing thing going on as B/BL. It's the
  // pipeline flush that causes the delay.
  std::shared_ptr<Core::Future> internal_exec() {
    processor.interrupt(ARMInterruptType::Software);
    return Core::Future::immediate_future();
  }

  SoftwareInterrupt(uint32_t encoded_insn) {}
};

////////////////////////////
// Basic ALU Instructions //
////////////////////////////

public
class ALUInsn : public ARMInsn {
 private:
  uint32_t encoded_insn;

  bool handle_flags() { return encoded_insn & 0x00100000; }

  void handle_arithmetic_flags(uint32_t op1, uint32_t op2, uint64_t result) {
    if (handle_flags()) {
      processor.set_zero(!result);
      processor.set_zero(result >> 31);
      processor.set_carry(result >> 32);
      if (op1 >> 31 == op2 >> 31) {
        bool overflow = ((result >> 31) & 0x01) == op1 >> 31;
        processor.set_overflow(overflow);
        if (overflow)
          processor.set_sticky_overflow(true);
      } else {
        processor.set_overflow(false);
      }
    }
  }

  void _and(uint64_t op1, uint64_t op2, uint32_t& dst) {
    uint64_t result = op1 & op2;
    handle_arithmetic_flags(op1, op2, result);
    dst = result;
  }

  void _eor(uint64_t op1, uint64_t op2, uint32_t& dst) {
    uint64_t result = op1 ^ op2;
    handle_arithmetic_flags(op1, op2, result);
    dst = result;
  }

  void _sub(uint64_t op1, uint64_t op2, uint32_t& dst) {
    uint64_t result = op1 - op2;
    handle_arithmetic_flags(op1, op2, result);
    dst = result;
  }

  void _rsb(uint64_t op1, uint64_t op2, uint32_t& dst) {
    uint64_t result = op2 - op1;
    handle_arithmetic_flags(op1, op2, result);
    dst = result;
  }

  void _add(uint64_t op1, uint64_t op2, uint32_t& dst) {
    uint64_t result = op1 + op2;
    handle_arithmetic_flags(op1, op2, result);
    dst = result;
  }

  void _adc(uint64_t op1, uint64_t op2, uint32_t& dst) {
    uint64_t result = op1 + op2 + processor.get_carry();
    handle_arithmetic_flags(op1, op2, result);
    dst = result;
  }

  void _sbc(uint64_t op1, uint64_t op2, uint32_t& dst) {
    uint64_t result = op1 - op2 + processor.get_carry() - 1;
    handle_arithmetic_flags(op1, op2, result);
    dst = result;
  }

  void _rsc(uint64_t op1, uint64_t op2, uint32_t& dst) {
    uint64_t result = op2 - op1 + processor.get_carry() - 1;
    handle_arithmetic_flags(op1, op2, result);
    dst = result;
  }

  void _tst(uint64_t op1, uint64_t op2, uint32_t& dst) {
    uint64_t result = op1 & op2;
    handle_arithmetic_flags(op1, op2, result);
  }

  void _tst(uint64_t op1, uint64_t op2, uint32_t& dst) {
    uint64_t result = op1 ^ op2;
    handle_arithmetic_flags(op1, op2, result);
  }

  void _cmp(uint64_t op1, uint64_t op2, uint32_t& dst) {
    uint64_t result = op1 - op2;
    handle_arithmetic_flags(op1, op2, result);
  }

  void _cmn(uint64_t op1, uint64_t op2, uint32_t& dst) {
    uint64_t result = op1 + op2;
    handle_arithmetic_flags(op1, op2, result);
  }

  void _orr(uint64_t op1, uint64_t op2, uint32_t& dst) {
    uint64_t result = op1 | op2;
    handle_arithmetic_flags(op1, op2, result);
    dst = result;
  }

  void _mov(uint64_t op1, uint64_t op2, uint32_t& dst) {
    uint64_t result = op2;
    handle_arithmetic_flags(op1, op2, result);
    dst = result;
  }

  void _bic(uint64_t op1, uint64_t op2, uint32_t& dst) {
    uint64_t result = op1 & (~op2);
    handle_arithmetic_flags(op1, op2, result);
    dst = result;
  }

  void _mvn(uint64_t op1, uint64_t op2, uint32_t& dst) {
    uint64_t result = ~op2;
    handle_arithmetic_flags(op1, op2, result);
    dst = result;
  }

  uint32_t& access_dst() {
    return processor.access_reg((encoded_insn >> 12) & 0x0F);
  }

  uint32_t get_op1() {
    return processor.access_reg((encoded_insn >> 16) & 0x0F);
  }

  uint32_t get_op2() {
    if (encoded_insn & 0x01000000) {
      uint32_t ret = encoded_insn & 0xFF;
      int rotate = ((encoded_insn >> 8) & 0x0F) * 2;
      ret = Core::rotate_right<uint32_t>(ret, rotate);
      return ret;
    } else {
      uint32_t ret = processor.access_reg(encoded_insn & 0x0F);
      int shift_val = 0;
      if (encoded_insn & 0x00000010) {
        shift_val = processor.access_reg((encoded_insn >> 8) & 0x0F);
      } else {
        shift_val = (encoded_insn >> 7) & 0x1F;
      }

      switch ((encoded_insn >> 5) & 0x03) {
        case 0:
          ret <<= shift_val;
          break;
        case 1:
          ret >>= shift_val;
          break;
        case 2:
          ret = Core::arithmetic_right_shift<uint32_t>(ret, shift_val);
          break;
        case 3:
          ret = Core::rotate_right<uint32_t>(ret, shift_val);
          break;
        default:
          log(LogLevel::fatal, "Error! Invalid shift value\n");
          break;
      }

      return ret;
    }
  }

  std::shared_ptr<Core::Future> internal_exec() {
    int opcode = (encoded_insn >> 21) & 0x0F;

    switch (opcode) {
      case 0:
        _and(get_op1(), get_op2(), access_dst());
        break;
      case 1:
        _eor(get_op1(), get_op2(), access_dst());
        break;
      case 2:
        _sub(get_op1(), get_op2(), access_dst());
        break;
      case 3:
        _rsb(get_op1(), get_op2(), access_dst());
        break;
      case 4:
        _add(get_op1(), get_op2(), access_dst());
        break;
      case 5:
        _adc(get_op1(), get_op2(), access_dst());
        break;
      case 6:
        _sbc(get_op1(), get_op2(), access_dst());
        break;
      case 7:
        _rsc(get_op1(), get_op2(), access_dst());
        break;
      case 8:
        _tst(get_op1(), get_op2(), access_dst());
        break;
      case 9:
        _teq(get_op1(), get_op2(), access_dst());
        break;
      case 10:
        _cmp(get_op1(), get_op2(), access_dst());
        break;
      case 11:
        _cmn(get_op1(), get_op2(), access_dst());
        break;
      case 12:
        _orr(get_op1(), get_op2(), access_dst());
        break;
      case 13:
        _mov(get_op1(), get_op2(), access_dst());
        break;
      case 14:
        _bic(get_op1(), get_op2(), access_dst());
        break;
      case 15:
        _mvn(get_op1(), get_op2(), access_dst());
        break;
      default:
        log(LogLevel::fatal, "Invalid ALU opcode!\n");
        break;
    }

    return Core::Future::immediate_future();
  }

 public:
  const static bool is_parseable(uint32_t encoded_insn) override {
    if (((encoded_insn >> 26) & 0x03) == 0b00) {
      if (((encoded_insn >> 25) & 0x01) == 1) {
        return true;
      } else if (((encoded_insn >> 4) & 0x01) == 0) {
        return true;
      } else if (((encoded_insn >> 7) & 0x01) == 0) {
        return true;
      }
    }

    return false;
  }

  ALUInsn(uint32_t encoded_insn) { this->encoded_insn = encoded_insn; }
};

//////////////////////
// MUL Instructions //
//////////////////////

public
class MulInsn : ARMInsn {
 private:
  void _mul(uint64_t op1, uint64_t op2, uint32_t& dst) {
    uint64_t result = op1 * op2;
    if (handle_flags())
      set_negative_zero_flags(result);
    dst = result;
  }

  void _mla(uint64_t op1, uint64_t op2, uint64_t op3, uint32_t& dst) {
    uint64_t result = op1 * op2 + op3;
    if (handle_flags())
      set_negative_zero_flags(result);
    dst = result;
  }

  void _umull(uint64_t op1, uint64_t op2, uint32_t& dst_hi, uint32_t& dst_lo) {
    uint64_t result = op1 * op2;
    if (handle_flags()) {
      processor.set_negative(result >> 63);
      processor.set_zero(!result);
    }
    dst_hi = result >> 32;
    dst_lo = result;
  }

  void _umlal(uint64_t op1, uint64_t op2, uint32_t& dst_hi, uint32_t& dst_lo) {
    uint64_t result = dst_hi;
    result <<= 32;
    result |= dst_lo;
    result += op1 * op2;
    if (handle_flags()) {
      processor.set_negative(result >> 63);
      processor.set_zero(!result);
    }
    dst_hi = result >> 32;
    dst_lo = result;
  }

  void _smull(int64_t op1, int64_t op2, uint32_t& dst_hi, uint32_t& dst_lo) {
    int64_t result = op1 * op2;
    if (handle_flags()) {
      processor.set_negative(result >> 63);
      processor.set_zero(!result);
    }
    dst_hi = result >> 32;
    dst_lo = result;
  }

  void _smlal(int64_t op1, int64_t op2, uint32_t& dst_hi, uint32_t& dst_lo) {
    int64_t result = dst_hi;
    result <<= 32;
    result |= dst_lo;
    result += op1 * op2;
    if (handle_flags()) {
      processor.set_negative(result >> 63);
      processor.set_zero(!result);
    }
    dst_hi = result >> 32;
    dst_lo = result;
  }

  std::shared_ptr<Core::Future> internal_exec() {
    uint32_t& dst_hi = processor.access_reg((encoded_insn >> 16) & 0x0F);
    uint32_t& dst_lo = processor.access_reg((encoded_insn >> 12) & 0x0F);
    uint32_t op1 = processor.access_reg(encoded_insn & 0x0F);
    uint32_t op2 = processor.access_reg((encoded_insn >> 8) & 0x0F);
    int32_t signed_op1 = (int32_t)op1;
    int32_t signed_op2 = (int32_t)op2;
    int opcode = (encoded_insn >> 21) & 0x07;

    int delay = 0;

    if (signed_op1 >= -256 && signed_op1 < 256) {
      delay = 1;
    } else if (signed_op1 >= -65536 && signed_op1 < 65536) {
      delay = 2;
    } else if (signed_op1 >= -16777216 && signed_op1 < 16777216) {
      delay = 3;
    } else {
      delay = 4;
    }

    switch (opcode) {
      case 1:
        delay += 1;
        break;
      case 4:
      case 6:
        delay += 1;
        break;
      case 5:
      case 7:
        delay += 2;
        break;
      default:
        break;
    }

    std::shared_ptr<Core::Future> ret = std::make_shared<Core::Future>();

    Core::clock->register_falling_edge_listener(
        [=, &this]() {
          switch (opcode) {
            case 0:
              this->_mul(op1, op2, dst_hi);
              break;
            case 1:
              this->_mla(op1, op2, dst_lo, dst_hi);
              break;
            case 4:
              this->_umull(op1, op2, dst_hi, dst_lo);
              break;
            case 5:
              this->_umlal(op1, op2, dst_hi, dst_lo);
              break;
            case 6:
              this->_smull(signed_op1, signed_op2, dst_hi, dst_lo);
              break;
            case 7:
              this->_smlal(signed_op1, signed_op2, dst_hi, dst_lo);
              break;
            default:
              log(LogLevel::fatal, "Invalid MUL opcode %d\n", opcode);
              break;
          }

          ret->make_available();
          return Core::Future::immediate_future();
        },
        delay - 1);

    return ret;
  }

 public:
  const static bool is_parseable(uint32_t encoded_insn) override {
    return (((encoded_insn >> 24) & 0x0F) == 0b0000) &&
           (((encoded_insn >> 4) & 0x0F) == 0b1001);
  }

  MulInsn(uint32_t encoded_insn) { this->encoded_insn = encoded_insn; }
};

}  // namespace ARM
