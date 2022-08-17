#include "arm/arm_insn.h"

#include "core/bitwise.h"
#include "core/bus.h"
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

protected:
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

public:
  const static bool is_parseable(uint32_t encoded_insn) override {
    return ((encoded_insn >> 25) & 0x07) == 0b101;
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
protected:
  // SWI has a similar instruction timing thing going on as B/BL. It's the
  // pipeline flush that causes the delay.
  std::shared_ptr<Core::Future> internal_exec() {
    processor.interrupt(ARMInterruptType::Software);
    return Core::Future::immediate_future();
  }

public:
  const static bool is_parseable(uint32_t encoded_insn) {
    return ((encoded_insn >> 24) & 0x0F) == 0b1111;
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

protected:
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
   bool handle_flags() { return encoded_insn & 0x00100000; }

   void _mul(uint64_t op1, uint64_t op2, uint32_t &dst) {
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

protected:
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

          exec_bus_activity_future = std::make_shared<Core::Future>();
          ret->make_available();
          return exec_bus_activity_future;
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

/////////////////////////////
// Load/Store Instructions //
/////////////////////////////

public
class LoadStoreInsn : public ARMInsn {
private:
  std::queue<int> regs;
  bool multiple_transfers;
  int base_reg_idx;
  uint32_t base_addr;
  bool uses_offset_reg;
  uint32_t offset_immediate;
  int offset_reg_idx;
  int shift_type;
  int shift_val;

  Core::ReadWrite dir;
  bool write_back;
  Core::DataSize size;
  bool index_up;
  bool post_index;
  bool restore_spr;
  bool is_signed;

  std::shared_ptr<Core::Future> exec_completion_future =
      std::make_shared<Core::Future>();

  void compute_new_base_addr() {
    if (index_up) {
      base_addr += size;
    } else {
      base_addr -= size;
    }
  }

  uint32_t compute_addr() {
    uint32_t ret = 0;

    if (post_index || !multiple_transfers) {
      ret = base_addr;
      compute_new_base_addr();
    } else {
      compute_new_base_addr();
      ret = base_addr;
    }

    if (!multiple_transfers && !post_index) {
      uint32_t offset = 0;
      if (uses_offset_reg) {
        offset = processor.access_reg(offset_reg_idx);
        switch (shift_type) {
        case 0:
          offset <<= shift_val;
          break;
        case 1:
          offset >>= shift_val;
          break;
        case 2:
          offset = Core::arithmetic_right_shift<uint32_t>(ret, shift_val);
          break;
        case 3:
          offset = Core::rotate_right<uint32_t>(ret, shift_val);
          break;
        default:
          log(LogLevel::fatal, "Error! Invalid shift value\n");
          break;
        }
      } else {
        offset = offset_immediate;
      }

      ret += offset;
      base_addr += offset;
    }

    return ret;
  }

  uint32_t handle_sign(uint32_t x) {
    if (!is_signed) {
      return x;
    } else {
      if (size == Core::DataSize::Byte) {
        return (int32_t)((int8_t)x);
      } else {
        return (int32_t)((int16_t)x);
      }
    }
  }

  void do_transfer() {
    if (regs.empty()) {
      if (write_back) {
        processor.access_reg(base_reg_idx) = base_addr;
      }
      exec_completion_future->make_available();
      return;
    }

    int reg_idx = regs.pop();
    std::shared_ptr<uint64_t> data;
    if (dir == Core::ReadWrite::write)
      *data = handle_sign(processor.access_reg(reg_idx));
    uint32_t addr = compute_addr();

    std::shared_ptr<Core::Future> bus_activity_future;
    if (regs.empty()) {
      exec_bus_activity_future = std::make_shared<Core::Future>();
      bus_activity_future = exec_bus_activity_future;
    } else {
      bus_activity_future = std::make_shared<Core::Future>();
    }

    auto bus_activity_future = std::make_shared<Core::Future>();
    auto bus_req_future =
        processor.data_bus->request(addr, dir, size, data, bus_activity_future);
    bus_req_future->add_listener([=, &this]() {
      if (dir == Core::ReadWrite::read) {
        processor.access_reg(reg_idx) = handle_sign(*data);

        if (reg_idx == 15 && multiple_transfers && restore_spr)
          processor.cpsr = processor.access_spsr();
      }
      do_transfer();

      if (!regs.empty()) {
        bus_activity_future->make_available();
      }
    });
  }

protected:
  std::shared_ptr<Core::Future> internal_exec() {
    base_addr = processor.access_reg(base_reg_idx);
    Core::clock->register_falling_edge_listener(
        [=, &this]() { do_transfer(); });

    return exec_completion_future;
  }

public:
  const static bool is_parseable(uint32_t encoded_insn) override {
    return // Full word or unsigned byte
        (((encoded_insn >> 25) & 0x03) == 0b010) ||
        (((encoded_insn >> 25) & 0x03) == 0b011 &&
         (((encoded_insn >> 4) & 0x01) == 0)) ||
        // Half word or signed byte
        (((encoded_insn >> 25) & 0x03) == 0b000 &&
         ((encoded_insn >> 22) & 0x01) == 1) ||
        (((encoded_insn >> 25) & 0x03) == 0b000 &&
         ((encoded_insn >> 22) & 0x01) == 0 &&
         ((encoded_insn >> 8) & 0x0F) == 0x0F) ||
        // Multi load/store
        (((encoded_insn >> 25) & 0x03) == 0b100);
  }

  LoadStoreInsn(uint32_t encoded_insn) {
    post_index = !((encoded_insn >> 24) & 0x01);
    index_up = ((encoded_insn >> 23) & 0x01);
    write_back = ((encoded_insn >> 21) & 0x01);

    if ((encoded_insn >> 20) & 0x01) {
      dir = Core::ReadWrite::read;
    } else {
      dir = Core::ReadWrite::write;
    }

    // Full word or unsigned byte
    if (((encoded_insn >> 25) & 0x03) == 0b01) {
      multiple_transfers = false;
      restore_spr = false;
      is_signed = false;

      if ((encoded_insn >> 22) & 0x01) {
        size = Core::DataSize::Byte;
      } else {
        size = Core::DataSize::DoubleWord;
      }

      if ((encoded_insn >> 25) & 0x01) {
        uses_offset_reg = false;
        offset_immediate = encoded_insn & 0x0FFF;
      } else {
        uses_offset_reg = true;
        shift_val = ((encoded_insn >> 7) & 0x1F;
        shift_type = ((encoded_insn >> 5) & 0x03;
        offset_reg_idx = encoded_insn & 0x0F;
      }

      base_reg_idx = ((encoded_insn >> 16) & 0x0F);

      regs.push((encoded_insn >> 12) & 0x0F);
    } else if {
      multiple_transfers = false;
      restore_spr = false;

      is_signed = ((encoded_insn >> 6) & 0x01);

      if ((encoded_insn >> 5) & 0x01) {
        size = Core::DataSize::Word;
      } else {
        size = Core::DataSize::Byte;
      }

      if ((encoded_insn >> 22) & 0x01) {
        uses_offset_reg = false;
        offset_immediate = ((encoded_insn >> 8) & 0x0F);
        offset_immediate <<= 4;
        offset_immediate |= encoded_insn & 0x0F;
      } else {
        uses_offset_reg = true;
        offset_reg_idx = encoded_insn & 0x0F;
        shift_type = 0;
        shift_val = 0;
      }

      base_reg_idx = ((encoded_insn >> 16) & 0x0F);

      regs.push((encoded_insn >> 12) & 0x0F);
    } else {
      multiple_transfers = true;
      uses_offset_reg = false;
      offset_immediate = 0;
      size = Core::DataSize::DoubleWord;
      is_signed = false;

      restore_spr = ((encoded_insn >> 22) & 0x01);

      base_reg_idx = ((encoded_insn >> 16) & 0x0F);

      for (int i = 0; i < 16; i++) {
        if (encoded_insn & 0x01)
          regs.push(i);

        encoded_insn >>= 1;
      }
    }
  }
};

//////////////////////
// Swap Instruction //
//////////////////////

public
class SwapInsn {
private:
  int source_reg_idx;
  int dest_reg_idx;
  int base_reg_idx;
  Core::DataSize size;

protected:
  std::shared_ptr<Core::Future> internal_exec() {
    Core::BusRequest store_req;
    Core::BusRequest load_req;

    load_req.addr = processor.access_reg(base_reg_idx);
    load_req.dir = Core::ReadWrite::read;
    load_req.size = size;
    load_req.data = std::make_shared<uint64_t>();
    load_req.request_completion_future = std::make_shared<Core::Future>();
    load_req.bus_activity_future = std::make_shared<Core::Future>();
    load_req.request_completeion_future->register_listener([=, &this]() {
      processor.access_reg(dest_reg_idx) = *data;
      load_req.bus_activity_future->make_available();
    });

    store_req.addr = processor.access_reg(base_reg_idx);
    store_req.dir = Core::ReadWrite::write;
    store_req.size = size;
    store_req.data = std::make_shared<uint64_t>();
    *data = processor.access_reg(
  }

} // namespace ARM
