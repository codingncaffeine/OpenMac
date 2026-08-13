#include "openmac/r65c02.hpp"

#include <stdexcept>

namespace openmac {

u8 R65C02::rb(u16 address) const {
    return read ? read(address) : 0xFF;
}

void R65C02::wb(u16 address, u8 value) const {
    if (write) write(address, value);
}

u8 R65C02::fetch8() {
    const u8 value = rb(pc);
    ++pc;
    return value;
}

u16 R65C02::fetch16() {
    const u8 low = fetch8();
    return static_cast<u16>(low | (static_cast<u16>(fetch8()) << 8));
}

u16 R65C02::read16(u16 address) const {
    const u8 low = rb(address);
    return static_cast<u16>(low |
        (static_cast<u16>(rb(static_cast<u16>(address + 1))) << 8));
}

void R65C02::reset() {
    a = x = y = 0;
    s = 0xFF;
    p = Reserved | IrqDisable;
    waiting = stopped = false;
    irq_ = false;
    nmiLine_ = nmiPending_ = false;
    pc = read16(0xFFFC);
    instructions = cycles = brkCount = 0;
    lastOpcode = 0;
    recentPc_.fill(pc);
    preBrkPc_.fill(pc);
    recentPcHead_ = 0;
}

void R65C02::setNmi(bool level) {
    if (level && !nmiLine_) nmiPending_ = true;
    nmiLine_ = level;
}

void R65C02::push(u8 value) {
    wb(static_cast<u16>(0x0100u | s), value);
    --s;
}

u8 R65C02::pop() {
    ++s;
    return rb(static_cast<u16>(0x0100u | s));
}

void R65C02::nz(u8 value) {
    p = static_cast<u8>((p & ~(Negative | Zero)) |
                        (value == 0 ? Zero : 0) | (value & Negative));
}

void R65C02::compare(u8 lhs, u8 rhs) {
    const u8 result = static_cast<u8>(lhs - rhs);
    p = static_cast<u8>((p & ~Carry) | (lhs >= rhs ? Carry : 0));
    nz(result);
}

void R65C02::adc(u8 value) {
    const unsigned carry = (p & Carry) ? 1u : 0u;
    const unsigned binary = static_cast<unsigned>(a) + value + carry;
    const bool overflow = ((~(a ^ value) & (a ^ binary)) & 0x80u) != 0;
    if (p & Decimal) {
        unsigned low = (a & 0x0Fu) + (value & 0x0Fu) + carry;
        if (low > 9u) low += 6u;
        unsigned result = (a & 0xF0u) + (value & 0xF0u) + low;
        if (result > 0x99u) result += 0x60u;
        p = static_cast<u8>((p & ~(Carry | Overflow)) |
                            (result > 0xFFu ? Carry : 0) |
                            (overflow ? Overflow : 0));
        a = static_cast<u8>(result);
    } else {
        p = static_cast<u8>((p & ~(Carry | Overflow)) |
                            (binary > 0xFFu ? Carry : 0) |
                            (overflow ? Overflow : 0));
        a = static_cast<u8>(binary);
    }
    nz(a);
}

void R65C02::sbc(u8 value) {
    const int borrow = (p & Carry) ? 0 : 1;
    const int binary = static_cast<int>(a) - static_cast<int>(value) - borrow;
    const bool overflow = (((a ^ value) & (a ^ binary)) & 0x80) != 0;
    if (p & Decimal) {
        int low = static_cast<int>(a & 0x0F) -
                  static_cast<int>(value & 0x0F) - borrow;
        int high = static_cast<int>(a >> 4) - static_cast<int>(value >> 4);
        if (low < 0) {
            low -= 6;
            --high;
        }
        if (high < 0) high -= 6;
        a = static_cast<u8>(((high << 4) & 0xF0) | (low & 0x0F));
    } else {
        a = static_cast<u8>(binary);
    }
    p = static_cast<u8>((p & ~(Carry | Overflow)) |
                        (binary >= 0 ? Carry : 0) |
                        (overflow ? Overflow : 0));
    nz(a);
}

u16 R65C02::effective(Mode mode, bool& pageCrossed) {
    pageCrossed = false;
    switch (mode) {
    case Mode::Immediate:
        return pc++;
    case Mode::Zero:
        return fetch8();
    case Mode::ZeroX:
        return static_cast<u8>(fetch8() + x);
    case Mode::ZeroY:
        return static_cast<u8>(fetch8() + y);
    case Mode::Absolute:
        return fetch16();
    case Mode::AbsoluteX: {
        const u16 base = fetch16();
        const u16 address = static_cast<u16>(base + x);
        pageCrossed = (base & 0xFF00u) != (address & 0xFF00u);
        return address;
    }
    case Mode::AbsoluteY: {
        const u16 base = fetch16();
        const u16 address = static_cast<u16>(base + y);
        pageCrossed = (base & 0xFF00u) != (address & 0xFF00u);
        return address;
    }
    case Mode::IndirectX: {
        const u8 pointer = static_cast<u8>(fetch8() + x);
        return static_cast<u16>(rb(pointer) |
            (static_cast<u16>(rb(static_cast<u8>(pointer + 1))) << 8));
    }
    case Mode::IndirectY: {
        const u8 pointer = fetch8();
        const u16 base = static_cast<u16>(rb(pointer) |
            (static_cast<u16>(rb(static_cast<u8>(pointer + 1))) << 8));
        const u16 address = static_cast<u16>(base + y);
        pageCrossed = (base & 0xFF00u) != (address & 0xFF00u);
        return address;
    }
    case Mode::ZeroIndirect: {
        const u8 pointer = fetch8();
        return static_cast<u16>(rb(pointer) |
            (static_cast<u16>(rb(static_cast<u8>(pointer + 1))) << 8));
    }
    }
    return 0;
}

u8 R65C02::operand(Mode mode, bool& pageCrossed) {
    return rb(effective(mode, pageCrossed));
}

int R65C02::interrupt(u16 vector, bool software) {
    push(static_cast<u8>(pc >> 8));
    push(static_cast<u8>(pc));
    push(static_cast<u8>((p & ~Break) | Reserved |
                         (software ? Break : 0)));
    p = static_cast<u8>((p | IrqDisable | Reserved) & ~Decimal);
    pc = read16(vector);
    waiting = false;
    return 7;
}

int R65C02::branch(bool take) {
    const auto displacement = static_cast<s8>(fetch8());
    if (!take) return 2;
    const u16 oldPc = pc;
    pc = static_cast<u16>(pc + displacement);
    return 3 + (((oldPc ^ pc) & 0xFF00u) != 0 ? 1 : 0);
}

int R65C02::bitBranch(u8 opcode) {
    const u8 zeroPage = fetch8();
    const auto displacement = static_cast<s8>(fetch8());
    const u8 mask = static_cast<u8>(1u << ((opcode >> 4) & 7u));
    const bool setForm = (opcode & 0x80u) != 0;
    const bool take = ((rb(zeroPage) & mask) != 0) == setForm;
    if (!take) return 5;
    const u16 oldPc = pc;
    pc = static_cast<u16>(pc + displacement);
    return 7 + (((oldPc ^ pc) & 0xFF00u) != 0 ? 1 : 0);
}

int R65C02::invalidNop(u8 opcode) {
    // W65C02S reserved-opcode lengths/times.  The NCR cell executes the same
    // harmless NOP classes; defining them avoids accidental NMOS "illegal"
    // behavior if downloaded diagnostics probe the empty matrix entries.
    switch (opcode) {
    case 0x02: case 0x22: case 0x42: case 0x62:
    case 0x82: case 0xC2: case 0xE2:
        (void)fetch8(); return 2;
    case 0x44:
        (void)fetch8(); return 3;
    case 0x54: case 0xD4: case 0xF4:
        (void)fetch8(); return 4;
    case 0x5C:
        (void)fetch16(); return 8;
    case 0xDC: case 0xFC:
        (void)fetch16(); return 4;
    default:
        return 1;
    }
}

int R65C02::step() {
    recentPc_[recentPcHead_++ & 31u] = pc;
    if (stopped) return 1;
    if (waiting && !nmiPending_ && !irq_) return 1;
    if (waiting && irq_ && (p & IrqDisable)) {
        waiting = false;
        ++cycles;
        return 1;
    }
    if (nmiPending_) {
        nmiPending_ = false;
        const int used = interrupt(0xFFFA, false);
        cycles += static_cast<u64>(used);
        return used;
    }
    if (irq_ && !(p & IrqDisable)) {
        const int used = interrupt(0xFFFE, false);
        cycles += static_cast<u64>(used);
        return used;
    }

    const u8 opcode = fetch8();
    lastOpcode = opcode;
    if (opcode == 0) {
        ++brkCount;
        for (int back = 0; back < 32; ++back)
            preBrkPc_[static_cast<unsigned>(back)] = recentPc(back);
    }
    ++instructions;
    bool crossed = false;
    int used = 0;

    auto readOp = [&](Mode mode, int base) {
        const u8 value = operand(mode, crossed);
        return std::pair<u8, int>{value, base + (crossed ? 1 : 0)};
    };
    auto logic = [&](Mode mode, int base, int operation) {
        const auto [value, time] = readOp(mode, base);
        if (operation == 0) a = static_cast<u8>(a | value);
        else if (operation == 1) a = static_cast<u8>(a & value);
        else a = static_cast<u8>(a ^ value);
        nz(a);
        return time;
    };
    auto load = [&](u8& reg, Mode mode, int base) {
        const auto [value, time] = readOp(mode, base);
        reg = value;
        nz(reg);
        return time;
    };
    auto arithmetic = [&](Mode mode, int base, bool subtract) {
        const auto [value, time] = readOp(mode, base);
        if (subtract) sbc(value); else adc(value);
        return time + ((p & Decimal) ? 1 : 0);
    };
    auto comparison = [&](u8 reg, Mode mode, int base) {
        const auto [value, time] = readOp(mode, base);
        compare(reg, value);
        return time;
    };
    auto store = [&](u8 value, Mode mode, int time) {
        wb(effective(mode, crossed), value);
        return time;
    };
    auto rmw = [&](Mode mode, int time, int operation) {
        const u16 address = effective(mode, crossed);
        u8 value = rb(address);
        if (operation == 0) { // ASL
            p = static_cast<u8>((p & ~Carry) | ((value & 0x80u) ? Carry : 0));
            value = static_cast<u8>(value << 1);
        } else if (operation == 1) { // LSR
            p = static_cast<u8>((p & ~Carry) | ((value & 1u) ? Carry : 0));
            value = static_cast<u8>(value >> 1);
        } else if (operation == 2) { // ROL
            const u8 carry = (p & Carry) ? 1u : 0u;
            p = static_cast<u8>((p & ~Carry) | ((value & 0x80u) ? Carry : 0));
            value = static_cast<u8>((value << 1) | carry);
        } else if (operation == 3) { // ROR
            const u8 carry = (p & Carry) ? 0x80u : 0u;
            p = static_cast<u8>((p & ~Carry) | ((value & 1u) ? Carry : 0));
            value = static_cast<u8>((value >> 1) | carry);
        } else if (operation == 4) {
            --value;
        } else {
            ++value;
        }
        wb(address, value);
        nz(value);
        return time;
    };
    auto accShift = [&](int operation) {
        if (operation == 0) {
            p = static_cast<u8>((p & ~Carry) | ((a & 0x80u) ? Carry : 0));
            a = static_cast<u8>(a << 1);
        } else if (operation == 1) {
            p = static_cast<u8>((p & ~Carry) | ((a & 1u) ? Carry : 0));
            a = static_cast<u8>(a >> 1);
        } else if (operation == 2) {
            const u8 carry = (p & Carry) ? 1u : 0u;
            p = static_cast<u8>((p & ~Carry) | ((a & 0x80u) ? Carry : 0));
            a = static_cast<u8>((a << 1) | carry);
        } else {
            const u8 carry = (p & Carry) ? 0x80u : 0u;
            p = static_cast<u8>((p & ~Carry) | ((a & 1u) ? Carry : 0));
            a = static_cast<u8>((a >> 1) | carry);
        }
        nz(a);
        return 2;
    };

    switch (opcode) {
    // ORA
    case 0x09: used = logic(Mode::Immediate, 2, 0); break;
    case 0x05: used = logic(Mode::Zero, 3, 0); break;
    case 0x15: used = logic(Mode::ZeroX, 4, 0); break;
    case 0x0D: used = logic(Mode::Absolute, 4, 0); break;
    case 0x1D: used = logic(Mode::AbsoluteX, 4, 0); break;
    case 0x19: used = logic(Mode::AbsoluteY, 4, 0); break;
    case 0x01: used = logic(Mode::IndirectX, 6, 0); break;
    case 0x11: used = logic(Mode::IndirectY, 5, 0); break;
    case 0x12: used = logic(Mode::ZeroIndirect, 5, 0); break;
    // AND
    case 0x29: used = logic(Mode::Immediate, 2, 1); break;
    case 0x25: used = logic(Mode::Zero, 3, 1); break;
    case 0x35: used = logic(Mode::ZeroX, 4, 1); break;
    case 0x2D: used = logic(Mode::Absolute, 4, 1); break;
    case 0x3D: used = logic(Mode::AbsoluteX, 4, 1); break;
    case 0x39: used = logic(Mode::AbsoluteY, 4, 1); break;
    case 0x21: used = logic(Mode::IndirectX, 6, 1); break;
    case 0x31: used = logic(Mode::IndirectY, 5, 1); break;
    case 0x32: used = logic(Mode::ZeroIndirect, 5, 1); break;
    // EOR
    case 0x49: used = logic(Mode::Immediate, 2, 2); break;
    case 0x45: used = logic(Mode::Zero, 3, 2); break;
    case 0x55: used = logic(Mode::ZeroX, 4, 2); break;
    case 0x4D: used = logic(Mode::Absolute, 4, 2); break;
    case 0x5D: used = logic(Mode::AbsoluteX, 4, 2); break;
    case 0x59: used = logic(Mode::AbsoluteY, 4, 2); break;
    case 0x41: used = logic(Mode::IndirectX, 6, 2); break;
    case 0x51: used = logic(Mode::IndirectY, 5, 2); break;
    case 0x52: used = logic(Mode::ZeroIndirect, 5, 2); break;
    // ADC
    case 0x69: used = arithmetic(Mode::Immediate, 2, false); break;
    case 0x65: used = arithmetic(Mode::Zero, 3, false); break;
    case 0x75: used = arithmetic(Mode::ZeroX, 4, false); break;
    case 0x6D: used = arithmetic(Mode::Absolute, 4, false); break;
    case 0x7D: used = arithmetic(Mode::AbsoluteX, 4, false); break;
    case 0x79: used = arithmetic(Mode::AbsoluteY, 4, false); break;
    case 0x61: used = arithmetic(Mode::IndirectX, 6, false); break;
    case 0x71: used = arithmetic(Mode::IndirectY, 5, false); break;
    case 0x72: used = arithmetic(Mode::ZeroIndirect, 5, false); break;
    // SBC
    case 0xE9: used = arithmetic(Mode::Immediate, 2, true); break;
    case 0xE5: used = arithmetic(Mode::Zero, 3, true); break;
    case 0xF5: used = arithmetic(Mode::ZeroX, 4, true); break;
    case 0xED: used = arithmetic(Mode::Absolute, 4, true); break;
    case 0xFD: used = arithmetic(Mode::AbsoluteX, 4, true); break;
    case 0xF9: used = arithmetic(Mode::AbsoluteY, 4, true); break;
    case 0xE1: used = arithmetic(Mode::IndirectX, 6, true); break;
    case 0xF1: used = arithmetic(Mode::IndirectY, 5, true); break;
    case 0xF2: used = arithmetic(Mode::ZeroIndirect, 5, true); break;
    // LDA
    case 0xA9: used = load(a, Mode::Immediate, 2); break;
    case 0xA5: used = load(a, Mode::Zero, 3); break;
    case 0xB5: used = load(a, Mode::ZeroX, 4); break;
    case 0xAD: used = load(a, Mode::Absolute, 4); break;
    case 0xBD: used = load(a, Mode::AbsoluteX, 4); break;
    case 0xB9: used = load(a, Mode::AbsoluteY, 4); break;
    case 0xA1: used = load(a, Mode::IndirectX, 6); break;
    case 0xB1: used = load(a, Mode::IndirectY, 5); break;
    case 0xB2: used = load(a, Mode::ZeroIndirect, 5); break;
    // CMP
    case 0xC9: used = comparison(a, Mode::Immediate, 2); break;
    case 0xC5: used = comparison(a, Mode::Zero, 3); break;
    case 0xD5: used = comparison(a, Mode::ZeroX, 4); break;
    case 0xCD: used = comparison(a, Mode::Absolute, 4); break;
    case 0xDD: used = comparison(a, Mode::AbsoluteX, 4); break;
    case 0xD9: used = comparison(a, Mode::AbsoluteY, 4); break;
    case 0xC1: used = comparison(a, Mode::IndirectX, 6); break;
    case 0xD1: used = comparison(a, Mode::IndirectY, 5); break;
    case 0xD2: used = comparison(a, Mode::ZeroIndirect, 5); break;

    // Loads/stores of index registers.
    case 0xA2: used = load(x, Mode::Immediate, 2); break;
    case 0xA6: used = load(x, Mode::Zero, 3); break;
    case 0xB6: used = load(x, Mode::ZeroY, 4); break;
    case 0xAE: used = load(x, Mode::Absolute, 4); break;
    case 0xBE: used = load(x, Mode::AbsoluteY, 4); break;
    case 0xA0: used = load(y, Mode::Immediate, 2); break;
    case 0xA4: used = load(y, Mode::Zero, 3); break;
    case 0xB4: used = load(y, Mode::ZeroX, 4); break;
    case 0xAC: used = load(y, Mode::Absolute, 4); break;
    case 0xBC: used = load(y, Mode::AbsoluteX, 4); break;
    case 0x85: used = store(a, Mode::Zero, 3); break;
    case 0x95: used = store(a, Mode::ZeroX, 4); break;
    case 0x8D: used = store(a, Mode::Absolute, 4); break;
    case 0x9D: used = store(a, Mode::AbsoluteX, 5); break;
    case 0x99: used = store(a, Mode::AbsoluteY, 5); break;
    case 0x81: used = store(a, Mode::IndirectX, 6); break;
    case 0x91: used = store(a, Mode::IndirectY, 6); break;
    case 0x92: used = store(a, Mode::ZeroIndirect, 5); break;
    case 0x86: used = store(x, Mode::Zero, 3); break;
    case 0x96: used = store(x, Mode::ZeroY, 4); break;
    case 0x8E: used = store(x, Mode::Absolute, 4); break;
    case 0x84: used = store(y, Mode::Zero, 3); break;
    case 0x94: used = store(y, Mode::ZeroX, 4); break;
    case 0x8C: used = store(y, Mode::Absolute, 4); break;
    case 0x64: used = store(0, Mode::Zero, 3); break;
    case 0x74: used = store(0, Mode::ZeroX, 4); break;
    case 0x9C: used = store(0, Mode::Absolute, 4); break;
    case 0x9E: used = store(0, Mode::AbsoluteX, 5); break;

    case 0xE0: used = comparison(x, Mode::Immediate, 2); break;
    case 0xE4: used = comparison(x, Mode::Zero, 3); break;
    case 0xEC: used = comparison(x, Mode::Absolute, 4); break;
    case 0xC0: used = comparison(y, Mode::Immediate, 2); break;
    case 0xC4: used = comparison(y, Mode::Zero, 3); break;
    case 0xCC: used = comparison(y, Mode::Absolute, 4); break;

    // Shifts, rotates, increment and decrement.
    case 0x0A: used = accShift(0); break;
    case 0x06: used = rmw(Mode::Zero, 5, 0); break;
    case 0x16: used = rmw(Mode::ZeroX, 6, 0); break;
    case 0x0E: used = rmw(Mode::Absolute, 6, 0); break;
    case 0x1E: used = rmw(Mode::AbsoluteX, 6, 0); break;
    case 0x4A: used = accShift(1); break;
    case 0x46: used = rmw(Mode::Zero, 5, 1); break;
    case 0x56: used = rmw(Mode::ZeroX, 6, 1); break;
    case 0x4E: used = rmw(Mode::Absolute, 6, 1); break;
    case 0x5E: used = rmw(Mode::AbsoluteX, 6, 1); break;
    case 0x2A: used = accShift(2); break;
    case 0x26: used = rmw(Mode::Zero, 5, 2); break;
    case 0x36: used = rmw(Mode::ZeroX, 6, 2); break;
    case 0x2E: used = rmw(Mode::Absolute, 6, 2); break;
    case 0x3E: used = rmw(Mode::AbsoluteX, 6, 2); break;
    case 0x6A: used = accShift(3); break;
    case 0x66: used = rmw(Mode::Zero, 5, 3); break;
    case 0x76: used = rmw(Mode::ZeroX, 6, 3); break;
    case 0x6E: used = rmw(Mode::Absolute, 6, 3); break;
    case 0x7E: used = rmw(Mode::AbsoluteX, 6, 3); break;
    case 0x3A: --a; nz(a); used = 2; break;
    case 0xC6: used = rmw(Mode::Zero, 5, 4); break;
    case 0xD6: used = rmw(Mode::ZeroX, 6, 4); break;
    case 0xCE: used = rmw(Mode::Absolute, 6, 4); break;
    case 0xDE: used = rmw(Mode::AbsoluteX, 6, 4); break;
    case 0x1A: ++a; nz(a); used = 2; break;
    case 0xE6: used = rmw(Mode::Zero, 5, 5); break;
    case 0xF6: used = rmw(Mode::ZeroX, 6, 5); break;
    case 0xEE: used = rmw(Mode::Absolute, 6, 5); break;
    case 0xFE: used = rmw(Mode::AbsoluteX, 6, 5); break;

    // BIT and CMOS test/set/reset memory.
    case 0x89: {
        const u8 value = fetch8();
        p = static_cast<u8>((p & ~Zero) | ((a & value) == 0 ? Zero : 0));
        used = 2; break;
    }
    case 0x24: case 0x34: case 0x2C: case 0x3C: {
        Mode mode = Mode::Zero; int base = 3;
        if (opcode == 0x34) { mode = Mode::ZeroX; base = 4; }
        else if (opcode == 0x2C) { mode = Mode::Absolute; base = 4; }
        else if (opcode == 0x3C) { mode = Mode::AbsoluteX; base = 4; }
        const auto [value, time] = readOp(mode, base);
        p = static_cast<u8>((p & ~(Negative | Overflow | Zero)) |
                            (value & (Negative | Overflow)) |
                            ((a & value) == 0 ? Zero : 0));
        used = time; break;
    }
    case 0x04: case 0x0C: case 0x14: case 0x1C: {
        const bool absolute = (opcode & 0x08u) != 0;
        const u16 address = effective(absolute ? Mode::Absolute : Mode::Zero,
                                      crossed);
        const u8 value = rb(address);
        p = static_cast<u8>((p & ~Zero) | ((a & value) == 0 ? Zero : 0));
        wb(address, (opcode & 0x10u) ? static_cast<u8>(value & ~a)
                                     : static_cast<u8>(value | a));
        used = absolute ? 6 : 5;
        break;
    }

    // Rockwell bit manipulation families.
    case 0x07: case 0x17: case 0x27: case 0x37:
    case 0x47: case 0x57: case 0x67: case 0x77:
    case 0x87: case 0x97: case 0xA7: case 0xB7:
    case 0xC7: case 0xD7: case 0xE7: case 0xF7: {
        const u8 address = fetch8();
        const u8 mask = static_cast<u8>(1u << ((opcode >> 4) & 7u));
        const u8 value = rb(address);
        wb(address, (opcode & 0x80u) ? static_cast<u8>(value | mask)
                                     : static_cast<u8>(value & ~mask));
        used = 5; break;
    }
    case 0x0F: case 0x1F: case 0x2F: case 0x3F:
    case 0x4F: case 0x5F: case 0x6F: case 0x7F:
    case 0x8F: case 0x9F: case 0xAF: case 0xBF:
    case 0xCF: case 0xDF: case 0xEF: case 0xFF:
        used = bitBranch(opcode); break;

    // Conditional branches and BRA.
    case 0x10: used = branch(!(p & Negative)); break;
    case 0x30: used = branch((p & Negative) != 0); break;
    case 0x50: used = branch(!(p & Overflow)); break;
    case 0x70: used = branch((p & Overflow) != 0); break;
    case 0x90: used = branch(!(p & Carry)); break;
    case 0xB0: used = branch((p & Carry) != 0); break;
    case 0xD0: used = branch(!(p & Zero)); break;
    case 0xF0: used = branch((p & Zero) != 0); break;
    case 0x80: used = branch(true); break;

    // Control flow and interrupt instructions.
    case 0x00:
        (void)fetch8();
        used = interrupt(0xFFFE, true);
        break;
    case 0x20: {
        const u16 target = fetch16();
        const u16 ret = static_cast<u16>(pc - 1);
        push(static_cast<u8>(ret >> 8));
        push(static_cast<u8>(ret));
        pc = target;
        used = 6; break;
    }
    case 0x4C: pc = fetch16(); used = 3; break;
    case 0x6C: pc = read16(fetch16()); used = 6; break;
    case 0x7C: {
        const u16 base = fetch16();
        pc = read16(static_cast<u16>(base + x));
        used = 6; break;
    }
    case 0x40: {
        p = static_cast<u8>((pop() & ~Break) | Reserved);
        const u8 low = pop();
        pc = static_cast<u16>(low | (static_cast<u16>(pop()) << 8));
        used = 6; break;
    }
    case 0x60: {
        const u8 low = pop();
        pc = static_cast<u16>((low | (static_cast<u16>(pop()) << 8)) + 1);
        used = 6; break;
    }

    // Stack operations.
    case 0x08: push(static_cast<u8>(p | Break | Reserved)); used = 3; break;
    case 0x28: p = static_cast<u8>((pop() & ~Break) | Reserved); used = 4; break;
    case 0x48: push(a); used = 3; break;
    case 0x68: a = pop(); nz(a); used = 4; break;
    case 0x5A: push(y); used = 3; break;
    case 0x7A: y = pop(); nz(y); used = 4; break;
    case 0xDA: push(x); used = 3; break;
    case 0xFA: x = pop(); nz(x); used = 4; break;

    // Register transfers, increments and flag instructions.
    case 0x88: --y; nz(y); used = 2; break;
    case 0xC8: ++y; nz(y); used = 2; break;
    case 0xCA: --x; nz(x); used = 2; break;
    case 0xE8: ++x; nz(x); used = 2; break;
    case 0x8A: a = x; nz(a); used = 2; break;
    case 0x98: a = y; nz(a); used = 2; break;
    case 0xAA: x = a; nz(x); used = 2; break;
    case 0xA8: y = a; nz(y); used = 2; break;
    case 0xBA: x = s; nz(x); used = 2; break;
    case 0x9A: s = x; used = 2; break;
    case 0x18: p &= static_cast<u8>(~Carry); used = 2; break;
    case 0x38: p |= Carry; used = 2; break;
    case 0x58: p &= static_cast<u8>(~IrqDisable); used = 2; break;
    case 0x78: p |= IrqDisable; used = 2; break;
    case 0xB8: p &= static_cast<u8>(~Overflow); used = 2; break;
    case 0xD8: p &= static_cast<u8>(~Decimal); used = 2; break;
    case 0xF8: p |= Decimal; used = 2; break;
    case 0xEA: used = 2; break;
    case 0xCB: waiting = true; used = 3; break;
    case 0xDB: stopped = true; used = 3; break;

    default:
        used = invalidNop(opcode);
        break;
    }

    p |= Reserved;
    cycles += static_cast<u64>(used);
    return used;
}

} // namespace openmac
