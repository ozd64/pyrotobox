use byteorder::{ByteOrder, LittleEndian};

use crate::mapper::CpuMemMap;

const CPU_STACK_START_OFFSET: u16 = 0x0100;

#[derive(Debug, Clone, Copy)]
enum AddressingMode {
    Implied,
    Accumulator,
    Immediate,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
    Relative,
    Absolute,
    AbsoluteX,
    AbsoluteY,
    Indirect,
    IndirectX,
    IndirectY,
}

#[derive(Debug, Clone, Copy)]
enum CpuFlag {
    Carry = 1 << 0,
    Zero = 1 << 1,
    InterruptDisable = 1 << 2,
    Decimal = 1 << 3,
    Break = 1 << 4,
    Overflow = 1 << 6,
    Negative = 1 << 7,
}

#[derive(Debug, Clone)]
struct CpuInstruction(String, AddressingMode, u8);

pub struct Cpu {
    mem_map: CpuMemMap,
    r_pc: u16,
    r_x: u8,
    r_y: u8,
    r_a: u8,
    r_sp: u8,
    r_flags: u8,
    instructions: Vec<CpuInstruction>,
}

pub struct OperandDetails {
    address: u16,
    value: u8,
    cycles: u8,
    addressing_mode: AddressingMode,
}

impl OperandDetails {
    fn new(address: u16, value: u8, cycles: u8, addressing_mode: AddressingMode) -> Self {
        Self {
            address,
            value,
            cycles,
            addressing_mode,
        }
    }
}

impl Cpu {
    pub fn new(cpu_mem_map: CpuMemMap) -> Self {
        let r_pc = Cpu::reset_vector(&cpu_mem_map);

        let mut instructions =
            vec![CpuInstruction("Unknown".to_string(), AddressingMode::Implied, 0x00); 0xFF];

        Cpu::generate_inst_defs(&mut instructions);

        Self {
            mem_map: cpu_mem_map,
            r_pc,
            r_x: 0,
            r_y: 0,
            r_a: 0,
            r_sp: 0xFD,
            r_flags: 0x04,
            instructions,
        }
    }

    pub fn exec_instruction(&mut self) -> usize {
        let r_pc = self.r_pc;
        let opcode = self.read_u8(self.r_pc);

        let operand_details = self.get_operand_details(opcode);

        self.disassemble_instruction(r_pc, &self.instructions[opcode as usize], &operand_details);
        let mut branch_cycles = 0;

        match opcode {
            0x69 | 0x65 | 0x75 | 0x6D | 0x7D | 0x79 | 0x61 | 0x71 => {
                self.adc(operand_details.value)
            }
            0x29 | 0x25 | 0x35 | 0x2D | 0x3D | 0x39 | 0x21 | 0x31 => {
                self.and(operand_details.value)
            }
            0x0A | 0x06 | 0x16 | 0x0E | 0x1E => self.asl(&operand_details),
            0x90 => self.bcc(operand_details.address, &mut branch_cycles),
            0xB0 => self.bcs(operand_details.address, &mut branch_cycles),
            0xF0 => self.beq(operand_details.address, &mut branch_cycles),
            0x24 | 0x2C => self.bit(operand_details.value),
            0x30 => self.bmi(operand_details.address, &mut branch_cycles),
            0xD0 => self.bne(operand_details.address, &mut branch_cycles),
            0x10 => self.bpl(operand_details.address, &mut branch_cycles),
            0x00 => self.brk(),
            0x50 => self.bvc(operand_details.address, &mut branch_cycles),
            0x70 => self.bvs(operand_details.address, &mut branch_cycles),
            0x18 => self.clc(),
            0xD8 => self.cld(),
            0x58 => self.cli(),
            0xB8 => self.clv(),
            0xC9 | 0xC5 | 0xD5 | 0xCD | 0xDD | 0xD9 | 0xC1 | 0xD1 => {
                self.cmp(operand_details.value)
            }
            0xE0 | 0xE4 | 0xEC => self.cpx(operand_details.value),
            0xC0 | 0xC4 | 0xCC => self.cpy(operand_details.value),
            0xCE | 0xC6 | 0xD6 | 0xDE => self.dec(operand_details.address, operand_details.value),
            0xCA => self.dex(),
            0x88 => self.dey(),
            0x49 | 0x45 | 0x55 | 0x4D | 0x5D | 0x59 | 0x41 | 0x51 => {
                self.eor(operand_details.value)
            }
            0xE6 | 0xF6 | 0xEE | 0xFE => self.inc(operand_details.address, operand_details.value),
            0xE8 => self.inx(),
            0xC8 => self.iny(),
            0x4C | 0x6C => self.jmp(operand_details.address),
            0x20 => self.jsr(operand_details.address),
            0xA9 | 0xA5 | 0xB5 | 0xAD | 0xBD | 0xB9 | 0xA1 | 0xB1 => {
                self.lda(operand_details.value)
            }
            0xA2 | 0xA6 | 0xB6 | 0xAE | 0xBE => self.ldx(operand_details.value),
            0xA0 | 0xA4 | 0xB4 | 0xAC | 0xBC => self.ldy(operand_details.value),
            0x4A | 0x46 | 0x56 | 0x4E | 0x5E => self.lsr(&operand_details),
            0xEA => self.nop(),
            0x09 | 0x05 | 0x15 | 0x0D | 0x1D | 0x19 | 0x01 | 0x11 => {
                self.ora(operand_details.value)
            }
            0x48 => self.pha(),
            0x08 => self.php(),
            0x68 => self.pla(),
            0x28 => self.plp(),
            0x2A | 0x26 | 0x36 | 0x2E | 0x3E => self.rol(&operand_details),
            0x6A | 0x66 | 0x76 | 0x6E | 0x7E => self.ror(&operand_details),
            0x40 => self.rti(),
            0x60 => self.rts(),
            0xE9 | 0xE5 | 0xF5 | 0xED | 0xFD | 0xF9 | 0xE1 | 0xF1 => {
                self.sbc(operand_details.value)
            }
            0x38 => self.sec(),
            0xF8 => self.sed(),
            0x78 => self.sei(),
            0x85 | 0x95 | 0x8D | 0x9D | 0x99 | 0x81 | 0x91 => self.sta(operand_details.address),
            0x86 | 0x96 | 0x8E => self.stx(operand_details.address),
            0x84 | 0x94 | 0x8C => self.sty(operand_details.address),
            0xAA => self.tax(),
            0xA8 => self.tay(),
            0xBA => self.tsx(),
            0x8A => self.txa(),
            0x9A => self.txs(),
            0x98 => self.tya(),
            // If the instruction is unknown then just increase pc by 1 to switch to next one
            _ => self.r_pc += 1,
        };

        (operand_details.cycles + branch_cycles) as usize
    }

    fn adc(&mut self, value: u8) {
        let mut new_acc_value = (self.r_a as u16).wrapping_add(value as u16);

        if self.get_flag(CpuFlag::Carry) {
            new_acc_value = new_acc_value.wrapping_add(1);
        }

        let overflow =
            (!((self.r_a as u16) ^ (value as u16)) & ((self.r_a as u16) ^ new_acc_value)) & 0x0080;

        self.set_flag(CpuFlag::Carry, new_acc_value > 255);
        self.set_flag(CpuFlag::Zero, new_acc_value & 0xFF == 0x00);
        self.set_flag(CpuFlag::Negative, new_acc_value & 0x80 > 0);
        self.set_flag(CpuFlag::Overflow, overflow > 0);

        self.r_a = (new_acc_value & 0x00FF) as u8;
    }

    fn and(&mut self, value: u8) {
        let result = self.r_a & value;

        self.set_flag(CpuFlag::Zero, result == 0x00);
        self.set_flag(CpuFlag::Negative, result & 0x80 > 0);

        self.r_a = result;
    }

    fn asl(&mut self, operand_details: &OperandDetails) {
        if let AddressingMode::Accumulator = operand_details.addressing_mode {
            let old_a = self.r_a;
            let carry_flag = (old_a >> 7) > 0;

            self.r_a = old_a << 1;

            self.set_flag(CpuFlag::Zero, self.r_a == 0);
            self.set_flag(CpuFlag::Carry, carry_flag);
            self.set_flag(CpuFlag::Negative, (self.r_a & 0x80) > 0);
        } else {
            let carry_flag = (operand_details.value >> 7) > 0;
            let new_value = operand_details.value << 1;

            self.write_u8(operand_details.address, new_value);

            self.set_flag(CpuFlag::Zero, new_value == 0);
            self.set_flag(CpuFlag::Carry, carry_flag);
            self.set_flag(CpuFlag::Negative, (new_value & 0x80) > 0);
        }
    }

    fn bcc(&mut self, branch_address: u16, branch_cycles: &mut u8) {
        if !self.get_flag(CpuFlag::Carry) {
            self.r_pc = branch_address;
            *branch_cycles += 1;
        }
    }

    fn bcs(&mut self, branch_address: u16, branch_cycles: &mut u8) {
        if self.get_flag(CpuFlag::Carry) {
            self.r_pc = branch_address;
            *branch_cycles += 1;
        }
    }

    fn beq(&mut self, branch_address: u16, branch_cycles: &mut u8) {
        if self.get_flag(CpuFlag::Zero) {
            self.r_pc = branch_address;
            *branch_cycles += 1;
        }
    }

    fn bit(&mut self, value: u8) {
        let and = self.r_a & value;

        self.set_flag(CpuFlag::Zero, and == 0);
        self.set_flag(CpuFlag::Overflow, (value & 0x40) > 0);
        self.set_flag(CpuFlag::Negative, (value & 0x80) > 0);
    }

    fn bmi(&mut self, branch_address: u16, branch_cycles: &mut u8) {
        if self.get_flag(CpuFlag::Negative) {
            self.r_pc = branch_address;
            *branch_cycles += 1;
        }
    }

    fn bne(&mut self, branch_address: u16, branch_cycles: &mut u8) {
        if !self.get_flag(CpuFlag::Zero) {
            self.r_pc = branch_address;
            *branch_cycles += 1;
        }
    }

    fn bpl(&mut self, branch_address: u16, branch_cycles: &mut u8) {
        if !self.get_flag(CpuFlag::Negative) {
            self.r_pc = branch_address;
            *branch_cycles += 1;
        }
    }

    fn brk(&mut self) {
        let pc_hl = (self.r_pc >> 8) as u8;
        let pc_ll = (self.r_pc & 0x00FF) as u8;

        self.push_stack(pc_hl);
        self.push_stack(pc_ll);

        self.push_stack(self.r_flags);

        self.r_pc = Cpu::irq_vector(&self.mem_map);

        self.set_flag(CpuFlag::Break, true);
    }

    fn bvc(&mut self, branch_address: u16, branch_cycles: &mut u8) {
        if !self.get_flag(CpuFlag::Overflow) {
            self.r_pc = branch_address;
            *branch_cycles += 1;
        }
    }

    fn bvs(&mut self, branch_address: u16, branch_cycles: &mut u8) {
        if self.get_flag(CpuFlag::Overflow) {
            self.r_pc = branch_address;
            *branch_cycles += 1;
        }
    }

    fn clc(&mut self) {
        self.set_flag(CpuFlag::Carry, false)
    }

    fn cld(&mut self) {
        self.set_flag(CpuFlag::Decimal, false)
    }

    fn cli(&mut self) {
        self.set_flag(CpuFlag::InterruptDisable, false)
    }

    fn clv(&mut self) {
        self.set_flag(CpuFlag::Overflow, false)
    }

    fn cmp(&mut self, value: u8) {
        let result = self.r_a.wrapping_sub(value);

        self.set_flag(CpuFlag::Carry, self.r_a >= value);
        self.set_flag(CpuFlag::Zero, result == 0);
        self.set_flag(CpuFlag::Negative, result & 0x80 > 0);
    }

    fn cpx(&mut self, value: u8) {
        let result = self.r_x.wrapping_sub(value);

        self.set_flag(CpuFlag::Carry, self.r_x >= value);
        self.set_flag(CpuFlag::Zero, result == 0);
        self.set_flag(CpuFlag::Negative, result & 0x80 > 0);
    }

    fn cpy(&mut self, value: u8) {
        let result = self.r_y.wrapping_sub(value);

        self.set_flag(CpuFlag::Carry, self.r_y >= value);
        self.set_flag(CpuFlag::Zero, result == 0);
        self.set_flag(CpuFlag::Negative, result & 0x80 > 0);
    }

    fn dec(&mut self, address: u16, value: u8) {
        let result = value.wrapping_sub(1);

        self.write_u8(address, result);

        self.set_flag(CpuFlag::Zero, result == 0);
        self.set_flag(CpuFlag::Negative, result & 0x80 > 0);
    }

    fn dex(&mut self) {
        let result = self.r_x.wrapping_sub(1);

        self.r_x = result;

        self.set_flag(CpuFlag::Zero, result == 0);
        self.set_flag(CpuFlag::Negative, result & 0x80 > 0);
    }

    fn dey(&mut self) {
        let result = self.r_y.wrapping_sub(1);

        self.r_y = result;

        self.set_flag(CpuFlag::Zero, result == 0);
        self.set_flag(CpuFlag::Negative, result & 0x80 > 0);
    }

    fn eor(&mut self, value: u8) {
        let result = self.r_a ^ value;

        self.r_a = result;

        self.set_flag(CpuFlag::Zero, result == 0);
        self.set_flag(CpuFlag::Negative, result & 0x80 > 0);
    }

    fn inc(&mut self, address: u16, value: u8) {
        let result = value.wrapping_add(1);

        self.write_u8(address, result);

        self.set_flag(CpuFlag::Zero, result == 0);
        self.set_flag(CpuFlag::Negative, result & 0x80 > 0);
    }

    fn inx(&mut self) {
        let result = self.r_x.wrapping_add(1);

        self.r_x = result;

        self.set_flag(CpuFlag::Zero, result == 0);
        self.set_flag(CpuFlag::Negative, result & 0x80 > 0);
    }

    fn iny(&mut self) {
        let result = self.r_y.wrapping_add(1);

        self.r_y = result;

        self.set_flag(CpuFlag::Zero, result == 0);
        self.set_flag(CpuFlag::Negative, result & 0x80 > 0);
    }

    fn jsr(&mut self, address: u16) {
        self.r_pc -= 1;

        let hl = (self.r_pc >> 8 & 0x00FF) as u8;
        let ll = (self.r_pc & 0x00FF) as u8;

        self.push_stack(hl);
        self.push_stack(ll);

        self.r_pc = address;
    }

    #[inline]
    fn jmp(&mut self, address: u16) {
        self.r_pc = address;
    }

    fn lda(&mut self, value: u8) {
        self.r_a = value;

        self.set_flag(CpuFlag::Zero, value == 0x00);
        self.set_flag(CpuFlag::Negative, value & 0x80 > 0);
    }

    fn ldx(&mut self, value: u8) {
        self.r_x = value;

        self.set_flag(CpuFlag::Zero, value == 0x00);
        self.set_flag(CpuFlag::Negative, value & 0x80 > 0);
    }

    fn ldy(&mut self, value: u8) {
        self.r_y = value;

        self.set_flag(CpuFlag::Zero, value == 0x00);
        self.set_flag(CpuFlag::Negative, value & 0x80 > 0);
    }

    fn lsr(&mut self, operand_details: &OperandDetails) {
        if let AddressingMode::Accumulator = operand_details.addressing_mode {
            let old_a = self.r_a;
            let carry_flag = (old_a & 0x01) > 0;

            self.r_a = old_a >> 1;

            self.set_flag(CpuFlag::Zero, self.r_a == 0);
            self.set_flag(CpuFlag::Carry, carry_flag);
            self.set_flag(CpuFlag::Negative, (self.r_a & 0x80) > 0);
        } else {
            let carry_flag = (operand_details.value & 0x01) > 0;
            let new_value = operand_details.value >> 1;

            self.write_u8(operand_details.address, new_value);

            self.set_flag(CpuFlag::Zero, new_value == 0);
            self.set_flag(CpuFlag::Carry, carry_flag);
            self.set_flag(CpuFlag::Negative, (new_value & 0x80) > 0);
        }
    }

    #[inline]
    fn nop(&self) {
        ()
    }

    fn ora(&mut self, value: u8) {
        let result = self.r_a | value;

        self.set_flag(CpuFlag::Zero, result == 0x00);
        self.set_flag(CpuFlag::Negative, result & 0x80 > 0);

        self.r_a = result;
    }

    #[inline]
    fn pha(&mut self) {
        self.push_stack(self.r_a);
    }

    #[inline]
    fn php(&mut self) {
        self.push_stack(self.r_flags);
    }

    fn pla(&mut self) {
        let value = self.pop_stack();

        self.r_a = value;

        self.set_flag(CpuFlag::Zero, value == 0x00);
        self.set_flag(CpuFlag::Negative, value & 0x80 > 0x00);
    }

    fn plp(&mut self) {
        let value = self.pop_stack();
        self.r_flags = value;
    }

    fn rol(&mut self, operand_details: &OperandDetails) {
        if let AddressingMode::Accumulator = operand_details.addressing_mode {
            let old_a = self.r_a;
            let carry_flag = (old_a >> 7) > 0;
            let bit0 = self.r_flags & 0x01;

            self.r_a = old_a << 1;
            self.r_a |= bit0;

            self.set_flag(CpuFlag::Zero, self.r_a == 0);
            self.set_flag(CpuFlag::Carry, carry_flag);
            self.set_flag(CpuFlag::Negative, (self.r_a & 0x80) > 0);
        } else {
            let carry_flag = (operand_details.value >> 7) > 0;

            let bit0 = self.r_flags & 0x01;
            let new_value = (operand_details.value << 1) | bit0;

            self.write_u8(operand_details.address, new_value);

            self.set_flag(CpuFlag::Zero, new_value == 0);
            self.set_flag(CpuFlag::Carry, carry_flag);
            self.set_flag(CpuFlag::Negative, (new_value & 0x80) > 0);
        }
    }

    fn ror(&mut self, operand_details: &OperandDetails) {
        if let AddressingMode::Accumulator = operand_details.addressing_mode {
            let old_a = self.r_a;
            let carry_flag = (old_a & 0x01) > 0;
            let bit7 = if self.get_flag(CpuFlag::Carry) {
                0x80
            } else {
                0
            };

            self.r_a = old_a >> 1;
            self.r_a |= bit7;

            self.set_flag(CpuFlag::Zero, self.r_a == 0);
            self.set_flag(CpuFlag::Carry, carry_flag);
            self.set_flag(CpuFlag::Negative, (self.r_a & 0x80) > 0);
        } else {
            let carry_flag = (operand_details.value & 0x01) > 0;

            let bit7 = if self.get_flag(CpuFlag::Carry) {
                0x80
            } else {
                0
            };
            let new_value = (operand_details.value >> 1) | bit7;

            self.write_u8(operand_details.address, new_value);

            self.set_flag(CpuFlag::Zero, new_value == 0);
            self.set_flag(CpuFlag::Carry, carry_flag);
            self.set_flag(CpuFlag::Negative, (new_value & 0x80) > 0);
        }
    }

    fn rti(&mut self) {
        let flags = self.pop_stack();
        let r_pc_ll = self.pop_stack();
        let r_pc_hl = self.pop_stack();

        self.r_pc = LittleEndian::read_u16(&vec![r_pc_ll, r_pc_hl]);
        self.r_flags = flags;
    }

    fn rts(&mut self) {
        let ll = self.pop_stack();
        let hl = self.pop_stack();
        let r_pc = LittleEndian::read_u16(&vec![ll, hl]);

        self.r_pc = r_pc + 1;
    }

    fn sbc(&mut self, value: u8) {
        let mut new_acc_value = (self.r_a as u16).wrapping_sub(value as u16);

        if self.get_flag(CpuFlag::Carry) {
            new_acc_value = new_acc_value.wrapping_sub(1);
        }

        let overflow =
            (!((self.r_a as u16) ^ (value as u16)) & ((self.r_a as u16) ^ new_acc_value)) & 0x0080;

        self.set_flag(CpuFlag::Carry, new_acc_value > 255);
        self.set_flag(CpuFlag::Zero, new_acc_value & 0xFF == 0x00);
        self.set_flag(CpuFlag::Negative, new_acc_value & 0x80 > 0);
        self.set_flag(CpuFlag::Overflow, overflow > 0);

        self.r_a = (new_acc_value & 0x00FF) as u8;
    }

    #[inline]
    fn sec(&mut self) {
        self.set_flag(CpuFlag::Carry, true);
    }

    #[inline]
    fn sed(&mut self) {
        self.set_flag(CpuFlag::Decimal, true);
    }

    #[inline]
    fn sei(&mut self) {
        self.set_flag(CpuFlag::InterruptDisable, true);
    }

    #[inline]
    fn sta(&mut self, address: u16) {
        self.write_u8(address, self.r_a);
    }

    #[inline]
    fn stx(&mut self, address: u16) {
        self.write_u8(address, self.r_x);
    }

    #[inline]
    fn sty(&mut self, address: u16) {
        self.write_u8(address, self.r_y);
    }

    fn tax(&mut self) {
        self.r_x = self.r_a;

        self.set_flag(CpuFlag::Zero, self.r_x == 0);
        self.set_flag(CpuFlag::Negative, self.r_x & 0x80 > 0);
    }

    fn tay(&mut self) {
        self.r_y = self.r_a;

        self.set_flag(CpuFlag::Zero, self.r_y == 0);
        self.set_flag(CpuFlag::Negative, self.r_y & 0x80 > 0);
    }

    fn tsx(&mut self) {
        self.r_x = self.r_sp;

        self.set_flag(CpuFlag::Zero, self.r_x == 0);
        self.set_flag(CpuFlag::Negative, self.r_x & 0x80 > 0);
    }

    fn txa(&mut self) {
        self.r_a = self.r_x;

        self.set_flag(CpuFlag::Zero, self.r_a == 0);
        self.set_flag(CpuFlag::Negative, self.r_a & 0x80 > 0);
    }

    fn txs(&mut self) {
        self.r_sp = self.r_x;

        self.set_flag(CpuFlag::Zero, self.r_sp == 0);
        self.set_flag(CpuFlag::Negative, self.r_sp & 0x80 > 0);
    }

    fn tya(&mut self) {
        self.r_a = self.r_y;

        self.set_flag(CpuFlag::Zero, self.r_a == 0);
        self.set_flag(CpuFlag::Negative, self.r_a & 0x80 > 0);
    }

    fn get_operand_details(&mut self, opcode: u8) -> OperandDetails {
        let instruction = &self.instructions[opcode as usize];

        match instruction.1 {
            AddressingMode::Implied => {
                self.r_pc += 1;
                OperandDetails::new(self.r_pc, opcode, instruction.2, instruction.1)
            }
            AddressingMode::Accumulator => {
                self.r_pc += 1;
                OperandDetails::new(self.r_pc, self.r_a, instruction.2, instruction.1)
            }
            AddressingMode::Immediate => {
                let addr = self.r_pc + 1;
                let value = self.read_u8(addr);
                self.r_pc += 2;

                OperandDetails::new(addr, value, instruction.2, instruction.1)
            }
            AddressingMode::ZeroPage => {
                let zero_page_addr = self.read_u8(self.r_pc + 1) as u16;
                let value = self.read_u8(zero_page_addr);

                self.r_pc += 2;
                OperandDetails::new(zero_page_addr, value, instruction.2, instruction.1)
            }
            AddressingMode::ZeroPageX => {
                let zero_page_addr = self.read_u8(self.r_pc + 1).wrapping_add(self.r_x) as u16;
                let value = self.read_u8(zero_page_addr);

                self.r_pc += 2;

                OperandDetails::new(zero_page_addr, value, instruction.2, instruction.1)
            }
            AddressingMode::ZeroPageY => {
                let zero_page_addr = self.read_u8(self.r_pc + 1).wrapping_add(self.r_y) as u16;
                let value = self.read_u8(zero_page_addr);

                self.r_pc += 2;

                OperandDetails::new(zero_page_addr, value, instruction.2, instruction.1)
            }
            AddressingMode::Relative => {
                let val = self.read_u8(self.r_pc + 1) as i8;
                let hl = (self.r_pc >> 8) as u8;

                self.r_pc += 2;
                let relative_address = ((self.r_pc as i32) + (val as i32)) as u16;
                //
                //If memory page changes then we need extra cycles
                let extra_cycles = if (relative_address & 0xFF00) != ((hl as u16) << 8) {
                    1
                } else {
                    0
                };

                OperandDetails::new(
                    relative_address,
                    0,
                    instruction.2 + extra_cycles,
                    instruction.1,
                )
            }
            AddressingMode::Absolute => {
                let le_addr_arr = vec![self.read_u8(self.r_pc + 1), self.read_u8(self.r_pc + 2)];
                let addr = LittleEndian::read_u16(&le_addr_arr);

                let value = self.read_u8(addr);

                self.r_pc += 3;

                OperandDetails::new(addr, value, instruction.2, instruction.1)
            }
            AddressingMode::AbsoluteX => {
                let low_bit = self.read_u8(self.r_pc + 1);
                let high_bit = self.read_u8(self.r_pc + 2);

                let le_addr_arr = vec![low_bit, high_bit];
                let addr = LittleEndian::read_u16(&le_addr_arr).wrapping_add(self.r_x as u16);

                let value = self.read_u8(addr);
                //
                //If memory page changes then we need extra cycles
                let extra_cycles = if (addr & 0xFF00) != ((high_bit as u16) << 8) {
                    1
                } else {
                    0
                };
                self.r_pc += 3;

                OperandDetails::new(addr, value, instruction.2 + extra_cycles, instruction.1)
            }
            AddressingMode::AbsoluteY => {
                let low_bit = self.read_u8(self.r_pc + 1);
                let high_bit = self.read_u8(self.r_pc + 2);

                let le_addr_arr = vec![low_bit, high_bit];
                let addr = LittleEndian::read_u16(&le_addr_arr).wrapping_add(self.r_y as u16);

                let value = self.read_u8(addr);
                //
                //If memory page changes then we need extra cycles
                let extra_cycles = if (addr & 0xFF00) != ((high_bit as u16) << 8) {
                    1
                } else {
                    0
                };
                self.r_pc += 3;

                OperandDetails::new(addr, value, instruction.2 + extra_cycles, instruction.1)
            }
            AddressingMode::Indirect => {
                let lsb = self.read_u8(self.r_pc + 1);
                let msb = self.read_u8(self.r_pc + 2);

                let lsb_addr = LittleEndian::read_u16(&vec![lsb, msb]);
                let msb_addr = lsb_addr.wrapping_add(1);

                let lsb = self.read_u8(lsb_addr);
                let msb = self.read_u8(msb_addr);

                let addr = LittleEndian::read_u16(&vec![lsb, msb]);

                self.r_pc += 3;

                OperandDetails::new(addr, opcode, 0, instruction.1)
            }
            AddressingMode::IndirectX => {
                let base = self.read_u8(self.r_pc + 1);
                let ptr = base.wrapping_add(self.r_x);

                let lo = self.read_u8(ptr as u16);
                let hi = self.read_u8(ptr.wrapping_add(1) as u16);

                let addr = ((hi as u16) << 8) | lo as u16;

                let value = self.read_u8(addr);

                self.r_pc += 2;

                OperandDetails::new(addr, value, instruction.2, instruction.1)
            }
            AddressingMode::IndirectY => {
                let base = self.read_u8(self.r_pc + 1);
                let lo = self.read_u8(base as u16);
                let hi = self.read_u8((base.wrapping_add(1) & 0x00FF) as u16);

                let mut addr = ((hi as u16) << 8) | lo as u16;
                addr = addr.wrapping_add(self.r_y as u16);

                let value = self.read_u8(addr);
                //If memory page changes then we need extra cycles
                let extra_cycles = if (addr & 0xFF00) != ((hi as u16) << 8) {
                    1
                } else {
                    0
                };

                self.r_pc += 2;

                OperandDetails::new(addr, value, instruction.2 + extra_cycles, instruction.1)
            }
        }
    }

    fn get_flag(&self, flag: CpuFlag) -> bool {
        (self.r_flags & flag as u8) > 0
    }

    fn set_flag(&mut self, flag: CpuFlag, value: bool) {
        let f = flag as u8;

        if value {
            self.r_flags |= f;
        } else {
            self.r_flags &= !f;
        }
    }

    fn read_u8(&self, address: u16) -> u8 {
        match address {
            //RAM space
            0x0000..=0x1FFF => {
                // RAM is mirrored towards the address 0x1FFF. No need to deal with mirrors.
                let pure_ram_address = (address & 0x07FF) as usize;
                self.mem_map[pure_ram_address]
            }
            //PPU registers
            0x2000..=0x3FFF => {
                // PPU registers are mirrored towards the address 0x3FFF. No need to deal with mirrors.
                let pure_ppu_address = (address & 0x2007) as usize;

                //TODO: Deal with PPU registers once we start working on them.
                self.mem_map[pure_ppu_address]
            }
            _ => self.mem_map[address as usize],
        }
    }

    fn write_u8(&mut self, address: u16, value: u8) {
        match address {
            //RAM space
            0x0000..=0x1FFF => {
                // RAM is mirrored towards the address 0x1FFF. No need to deal with mirrors.
                let pure_ram_address = (address & 0x07FF) as usize;
                self.mem_map[pure_ram_address] = value;
            }
            //PPU registers
            0x2000..=0x3FFF => {
                // PPU registers are mirrored towards the address 0x3FFF. No need to deal with mirrors.
                let pure_ppu_address = (address & 0x2007) as usize;

                //TODO: Deal with PPU registers once we start working on them.
                self.mem_map[pure_ppu_address] = value;
            }
            _ => self.mem_map[address as usize] = value,
        }
    }

    fn push_stack(&mut self, value: u8) {
        let addr = CPU_STACK_START_OFFSET | (self.r_sp as u16);

        self.write_u8(addr, value);
        self.r_sp -= 1;
    }

    fn pop_stack(&mut self) -> u8 {
        self.r_sp += 1;
        let addr = CPU_STACK_START_OFFSET | (self.r_sp as u16);
        let stack_value = self.read_u8(addr);

        // Clean up stack value after popping
        self.write_u8(addr, 0x00);

        stack_value
    }

    #[inline]
    fn reset_vector(cpu_mem_map: &CpuMemMap) -> u16 {
        LittleEndian::read_u16(&cpu_mem_map[0xFFFC..0xFFFE])
    }

    #[inline]
    fn irq_vector(cpu_mem_map: &CpuMemMap) -> u16 {
        LittleEndian::read_u16(&cpu_mem_map[0xFFFE..0x10000])
    }

    fn disassemble_instruction(
        &self,
        r_pc: u16,
        instruction: &CpuInstruction,
        operand_details: &OperandDetails,
    ) {
        print!("${:4X}:\t{}  ", r_pc, instruction.0);

        let instruction_output = match instruction.1 {
            AddressingMode::Implied => "".to_string(),
            AddressingMode::Accumulator => format!("A"),
            AddressingMode::Immediate => format!("#{}", operand_details.value),
            AddressingMode::ZeroPage => format!("${:X}", operand_details.address),
            AddressingMode::ZeroPageX => format!("${:X}, X", operand_details.address),
            AddressingMode::ZeroPageY => format!("${:X}, Y", operand_details.address),
            AddressingMode::Relative => {
                let r_pc_i32 = r_pc as i32;
                let new_address = operand_details.address as i32;
                let result = new_address - r_pc_i32;

                format!("*{}", result)
            }
            AddressingMode::Absolute => format!("${:X}", operand_details.address),
            AddressingMode::AbsoluteX => {
                let addr = operand_details.address.wrapping_sub(self.r_x as u16);
                format!("${:X}, X", addr)
            }
            AddressingMode::AbsoluteY => {
                let addr = operand_details.address.wrapping_sub(self.r_y as u16);
                format!("${:X}, Y", addr)
            }
            AddressingMode::Indirect => {
                let msb = self.read_u8(r_pc.wrapping_add(2));
                let lsb = self.read_u8(r_pc.wrapping_add(1));
                format!("(${:X}{:X})", lsb, msb)
            }
            AddressingMode::IndirectX => {
                let byte = self.read_u8(r_pc.wrapping_add(1));
                format!("(${:X}, X)", byte)
            }
            AddressingMode::IndirectY => {
                let byte = self.read_u8(r_pc.wrapping_add(1));
                format!("(${:X}), Y", byte)
            }
        };

        println!("{}", instruction_output);
    }

    fn generate_inst_defs(instructions: &mut Vec<CpuInstruction>) {
        instructions[0x69] = CpuInstruction("ADC".to_string(), AddressingMode::Immediate, 0x02);
        instructions[0x65] = CpuInstruction("ADC".to_string(), AddressingMode::ZeroPage, 0x03);
        instructions[0x75] = CpuInstruction("ADC".to_string(), AddressingMode::ZeroPageX, 0x04);
        instructions[0x6D] = CpuInstruction("ADC".to_string(), AddressingMode::Absolute, 0x04);
        instructions[0x7D] = CpuInstruction("ADC".to_string(), AddressingMode::AbsoluteX, 0x04);
        instructions[0x79] = CpuInstruction("ADC".to_string(), AddressingMode::AbsoluteY, 0x04);
        instructions[0x61] = CpuInstruction("ADC".to_string(), AddressingMode::IndirectX, 0x06);
        instructions[0x71] = CpuInstruction("ADC".to_string(), AddressingMode::IndirectY, 0x05);
        instructions[0x29] = CpuInstruction("AND".to_string(), AddressingMode::Immediate, 0x02);
        instructions[0x25] = CpuInstruction("AND".to_string(), AddressingMode::ZeroPage, 0x03);
        instructions[0x35] = CpuInstruction("AND".to_string(), AddressingMode::ZeroPageX, 0x04);
        instructions[0x2D] = CpuInstruction("AND".to_string(), AddressingMode::Absolute, 0x04);
        instructions[0x3D] = CpuInstruction("AND".to_string(), AddressingMode::AbsoluteX, 0x04);
        instructions[0x39] = CpuInstruction("AND".to_string(), AddressingMode::AbsoluteY, 0x04);
        instructions[0x21] = CpuInstruction("AND".to_string(), AddressingMode::IndirectX, 0x06);
        instructions[0x31] = CpuInstruction("AND".to_string(), AddressingMode::IndirectY, 0x05);
        instructions[0x0A] = CpuInstruction("ASL".to_string(), AddressingMode::Accumulator, 0x02);
        instructions[0x06] = CpuInstruction("ASL".to_string(), AddressingMode::ZeroPage, 0x05);
        instructions[0x16] = CpuInstruction("ASL".to_string(), AddressingMode::ZeroPageX, 0x06);
        instructions[0x0E] = CpuInstruction("ASL".to_string(), AddressingMode::Absolute, 0x06);
        instructions[0x1E] = CpuInstruction("ASL".to_string(), AddressingMode::AbsoluteX, 0x07);
        instructions[0x90] = CpuInstruction("BCC".to_string(), AddressingMode::Relative, 0x02);
        instructions[0xB0] = CpuInstruction("BCS".to_string(), AddressingMode::Relative, 0x02);
        instructions[0xF0] = CpuInstruction("BEQ".to_string(), AddressingMode::Relative, 0x02);
        instructions[0x24] = CpuInstruction("BIT".to_string(), AddressingMode::ZeroPage, 0x03);
        instructions[0x2C] = CpuInstruction("BIT".to_string(), AddressingMode::Absolute, 0x02);
        instructions[0x30] = CpuInstruction("BMI".to_string(), AddressingMode::Relative, 0x02);
        instructions[0xD0] = CpuInstruction("BNE".to_string(), AddressingMode::Relative, 0x02);
        instructions[0x10] = CpuInstruction("BPL".to_string(), AddressingMode::Relative, 0x02);
        instructions[0x00] = CpuInstruction("BRK".to_string(), AddressingMode::Implied, 0x07);
        instructions[0x50] = CpuInstruction("BVC".to_string(), AddressingMode::Relative, 0x02);
        instructions[0x70] = CpuInstruction("BVS".to_string(), AddressingMode::Relative, 0x02);
        instructions[0x18] = CpuInstruction("CLC".to_string(), AddressingMode::Implied, 0x02);
        instructions[0xD8] = CpuInstruction("CLD".to_string(), AddressingMode::Implied, 0x02);
        instructions[0x58] = CpuInstruction("CLI".to_string(), AddressingMode::Implied, 0x02);
        instructions[0xB8] = CpuInstruction("CLV".to_string(), AddressingMode::Implied, 0x02);
        instructions[0xC9] = CpuInstruction("CMP".to_string(), AddressingMode::Immediate, 0x02);
        instructions[0xC5] = CpuInstruction("CMP".to_string(), AddressingMode::ZeroPage, 0x03);
        instructions[0xD5] = CpuInstruction("CMP".to_string(), AddressingMode::ZeroPageX, 0x04);
        instructions[0xCD] = CpuInstruction("CMP".to_string(), AddressingMode::Absolute, 0x04);
        instructions[0xDD] = CpuInstruction("CMP".to_string(), AddressingMode::AbsoluteX, 0x04);
        instructions[0xD9] = CpuInstruction("CMP".to_string(), AddressingMode::AbsoluteY, 0x04);
        instructions[0xC1] = CpuInstruction("CMP".to_string(), AddressingMode::IndirectX, 0x06);
        instructions[0xD1] = CpuInstruction("CMP".to_string(), AddressingMode::IndirectY, 0x05);
        instructions[0xE0] = CpuInstruction("CPX".to_string(), AddressingMode::Immediate, 0x02);
        instructions[0xE4] = CpuInstruction("CPX".to_string(), AddressingMode::ZeroPage, 0x03);
        instructions[0xEC] = CpuInstruction("CPX".to_string(), AddressingMode::Absolute, 0x04);
        instructions[0xC0] = CpuInstruction("CPY".to_string(), AddressingMode::Immediate, 0x02);
        instructions[0xC4] = CpuInstruction("CPY".to_string(), AddressingMode::ZeroPage, 0x03);
        instructions[0xCC] = CpuInstruction("CPY".to_string(), AddressingMode::Absolute, 0x04);
        instructions[0xC6] = CpuInstruction("DEC".to_string(), AddressingMode::ZeroPage, 0x05);
        instructions[0xD6] = CpuInstruction("DEC".to_string(), AddressingMode::ZeroPageX, 0x06);
        instructions[0xCE] = CpuInstruction("DEC".to_string(), AddressingMode::Absolute, 0x06);
        instructions[0xDE] = CpuInstruction("DEC".to_string(), AddressingMode::AbsoluteX, 0x07);
        instructions[0xCA] = CpuInstruction("DEX".to_string(), AddressingMode::Implied, 0x02);
        instructions[0x88] = CpuInstruction("DEY".to_string(), AddressingMode::Implied, 0x02);
        instructions[0x49] = CpuInstruction("EOR".to_string(), AddressingMode::Immediate, 0x02);
        instructions[0x45] = CpuInstruction("EOR".to_string(), AddressingMode::ZeroPage, 0x03);
        instructions[0x55] = CpuInstruction("EOR".to_string(), AddressingMode::ZeroPageX, 0x04);
        instructions[0x4D] = CpuInstruction("EOR".to_string(), AddressingMode::Absolute, 0x04);
        instructions[0x5D] = CpuInstruction("EOR".to_string(), AddressingMode::AbsoluteX, 0x04);
        instructions[0x59] = CpuInstruction("EOR".to_string(), AddressingMode::AbsoluteY, 0x04);
        instructions[0x41] = CpuInstruction("EOR".to_string(), AddressingMode::IndirectX, 0x06);
        instructions[0x51] = CpuInstruction("EOR".to_string(), AddressingMode::IndirectY, 0x05);
        instructions[0xE6] = CpuInstruction("INC".to_string(), AddressingMode::ZeroPage, 0x05);
        instructions[0xF6] = CpuInstruction("INC".to_string(), AddressingMode::ZeroPageX, 0x06);
        instructions[0xEE] = CpuInstruction("INC".to_string(), AddressingMode::Absolute, 0x06);
        instructions[0xFE] = CpuInstruction("INC".to_string(), AddressingMode::AbsoluteX, 0x07);
        instructions[0xE8] = CpuInstruction("INX".to_string(), AddressingMode::Implied, 0x02);
        instructions[0xC8] = CpuInstruction("INY".to_string(), AddressingMode::Implied, 0x02);
        instructions[0x4C] = CpuInstruction("JMP".to_string(), AddressingMode::Absolute, 0x03);
        instructions[0x6C] = CpuInstruction("JMP".to_string(), AddressingMode::Indirect, 0x05);
        instructions[0x20] = CpuInstruction("JSR".to_string(), AddressingMode::Absolute, 0x06);
        instructions[0xA9] = CpuInstruction("LDA".to_string(), AddressingMode::Immediate, 0x02);
        instructions[0xA5] = CpuInstruction("LDA".to_string(), AddressingMode::ZeroPage, 0x03);
        instructions[0xB5] = CpuInstruction("LDA".to_string(), AddressingMode::ZeroPageX, 0x04);
        instructions[0xAD] = CpuInstruction("LDA".to_string(), AddressingMode::Absolute, 0x04);
        instructions[0xBD] = CpuInstruction("LDA".to_string(), AddressingMode::AbsoluteX, 0x04);
        instructions[0xB9] = CpuInstruction("LDA".to_string(), AddressingMode::AbsoluteY, 0x04);
        instructions[0xA1] = CpuInstruction("LDA".to_string(), AddressingMode::IndirectX, 0x06);
        instructions[0xB1] = CpuInstruction("LDA".to_string(), AddressingMode::IndirectY, 0x05);
        instructions[0xA2] = CpuInstruction("LDX".to_string(), AddressingMode::Immediate, 0x02);
        instructions[0xA6] = CpuInstruction("LDX".to_string(), AddressingMode::ZeroPage, 0x03);
        instructions[0xB6] = CpuInstruction("LDX".to_string(), AddressingMode::ZeroPageY, 0x04);
        instructions[0xAE] = CpuInstruction("LDX".to_string(), AddressingMode::Absolute, 0x04);
        instructions[0xBE] = CpuInstruction("LDX".to_string(), AddressingMode::AbsoluteY, 0x04);
        instructions[0xA0] = CpuInstruction("LDY".to_string(), AddressingMode::Immediate, 0x02);
        instructions[0xA4] = CpuInstruction("LDY".to_string(), AddressingMode::ZeroPage, 0x03);
        instructions[0xB4] = CpuInstruction("LDY".to_string(), AddressingMode::ZeroPageX, 0x04);
        instructions[0xAC] = CpuInstruction("LDY".to_string(), AddressingMode::Absolute, 0x04);
        instructions[0xBC] = CpuInstruction("LDY".to_string(), AddressingMode::AbsoluteX, 0x04);
        instructions[0x4A] = CpuInstruction("LSR".to_string(), AddressingMode::Accumulator, 0x02);
        instructions[0x46] = CpuInstruction("LSR".to_string(), AddressingMode::ZeroPage, 0x05);
        instructions[0x56] = CpuInstruction("LSR".to_string(), AddressingMode::ZeroPageX, 0x06);
        instructions[0x4E] = CpuInstruction("LSR".to_string(), AddressingMode::Absolute, 0x06);
        instructions[0x5E] = CpuInstruction("LSR".to_string(), AddressingMode::AbsoluteX, 0x07);
        instructions[0xEA] = CpuInstruction("NOP".to_string(), AddressingMode::Implied, 0x02);
        instructions[0x09] = CpuInstruction("ORA".to_string(), AddressingMode::Immediate, 0x02);
        instructions[0x05] = CpuInstruction("ORA".to_string(), AddressingMode::ZeroPage, 0x03);
        instructions[0x15] = CpuInstruction("ORA".to_string(), AddressingMode::ZeroPageX, 0x04);
        instructions[0x0D] = CpuInstruction("ORA".to_string(), AddressingMode::Absolute, 0x04);
        instructions[0x1D] = CpuInstruction("ORA".to_string(), AddressingMode::AbsoluteX, 0x04);
        instructions[0x19] = CpuInstruction("ORA".to_string(), AddressingMode::AbsoluteY, 0x04);
        instructions[0x01] = CpuInstruction("ORA".to_string(), AddressingMode::IndirectX, 0x06);
        instructions[0x11] = CpuInstruction("ORA".to_string(), AddressingMode::IndirectY, 0x05);
        instructions[0x48] = CpuInstruction("PHA".to_string(), AddressingMode::Implied, 0x03);
        instructions[0x08] = CpuInstruction("PHP".to_string(), AddressingMode::Implied, 0x03);
        instructions[0x68] = CpuInstruction("PLA".to_string(), AddressingMode::Implied, 0x04);
        instructions[0x28] = CpuInstruction("PLP".to_string(), AddressingMode::Implied, 0x04);
        instructions[0x2A] = CpuInstruction("ROL".to_string(), AddressingMode::Accumulator, 0x02);
        instructions[0x26] = CpuInstruction("ROL".to_string(), AddressingMode::ZeroPage, 0x05);
        instructions[0x36] = CpuInstruction("ROL".to_string(), AddressingMode::ZeroPageX, 0x06);
        instructions[0x2E] = CpuInstruction("ROL".to_string(), AddressingMode::Absolute, 0x06);
        instructions[0x3E] = CpuInstruction("ROL".to_string(), AddressingMode::AbsoluteX, 0x07);
        instructions[0x6A] = CpuInstruction("ROR".to_string(), AddressingMode::Accumulator, 0x02);
        instructions[0x66] = CpuInstruction("ROR".to_string(), AddressingMode::ZeroPage, 0x05);
        instructions[0x76] = CpuInstruction("ROR".to_string(), AddressingMode::ZeroPageX, 0x06);
        instructions[0x6E] = CpuInstruction("ROR".to_string(), AddressingMode::Absolute, 0x06);
        instructions[0x7E] = CpuInstruction("ROR".to_string(), AddressingMode::AbsoluteX, 0x07);
        instructions[0x40] = CpuInstruction("RTI".to_string(), AddressingMode::Implied, 0x06);
        instructions[0x60] = CpuInstruction("RTS".to_string(), AddressingMode::Implied, 0x06);
        instructions[0xE9] = CpuInstruction("SBC".to_string(), AddressingMode::Immediate, 0x02);
        instructions[0xE5] = CpuInstruction("SBC".to_string(), AddressingMode::ZeroPage, 0x03);
        instructions[0xF5] = CpuInstruction("SBC".to_string(), AddressingMode::ZeroPageX, 0x04);
        instructions[0xED] = CpuInstruction("SBC".to_string(), AddressingMode::Absolute, 0x04);
        instructions[0xFD] = CpuInstruction("SBC".to_string(), AddressingMode::AbsoluteX, 0x04);
        instructions[0xF9] = CpuInstruction("SBC".to_string(), AddressingMode::AbsoluteY, 0x04);
        instructions[0xE1] = CpuInstruction("SBC".to_string(), AddressingMode::IndirectX, 0x06);
        instructions[0xF1] = CpuInstruction("SBC".to_string(), AddressingMode::IndirectY, 0x05);
        instructions[0x38] = CpuInstruction("SEC".to_string(), AddressingMode::Implied, 0x02);
        instructions[0xF8] = CpuInstruction("SED".to_string(), AddressingMode::Implied, 0x02);
        instructions[0x78] = CpuInstruction("SEI".to_string(), AddressingMode::Implied, 0x02);
        instructions[0x85] = CpuInstruction("STA".to_string(), AddressingMode::ZeroPage, 0x03);
        instructions[0x95] = CpuInstruction("STA".to_string(), AddressingMode::ZeroPageX, 0x04);
        instructions[0x8D] = CpuInstruction("STA".to_string(), AddressingMode::Absolute, 0x04);
        instructions[0x9D] = CpuInstruction("STA".to_string(), AddressingMode::AbsoluteX, 0x05);
        instructions[0x99] = CpuInstruction("STA".to_string(), AddressingMode::AbsoluteY, 0x05);
        instructions[0x81] = CpuInstruction("STA".to_string(), AddressingMode::IndirectX, 0x06);
        instructions[0x91] = CpuInstruction("STA".to_string(), AddressingMode::IndirectY, 0x06);
        instructions[0x86] = CpuInstruction("STX".to_string(), AddressingMode::ZeroPage, 0x03);
        instructions[0x96] = CpuInstruction("STX".to_string(), AddressingMode::ZeroPageY, 0x04);
        instructions[0x8E] = CpuInstruction("STX".to_string(), AddressingMode::Absolute, 0x04);
        instructions[0x84] = CpuInstruction("STY".to_string(), AddressingMode::ZeroPage, 0x03);
        instructions[0x94] = CpuInstruction("STY".to_string(), AddressingMode::ZeroPageX, 0x04);
        instructions[0x8C] = CpuInstruction("STY".to_string(), AddressingMode::Absolute, 0x04);
        instructions[0xAA] = CpuInstruction("TAX".to_string(), AddressingMode::Implied, 0x02);
        instructions[0xA8] = CpuInstruction("TAY".to_string(), AddressingMode::Implied, 0x02);
        instructions[0xBA] = CpuInstruction("TSX".to_string(), AddressingMode::Implied, 0x02);
        instructions[0x8A] = CpuInstruction("TXA".to_string(), AddressingMode::Implied, 0x02);
        instructions[0x9A] = CpuInstruction("TXS".to_string(), AddressingMode::Implied, 0x02);
        instructions[0x98] = CpuInstruction("TYA".to_string(), AddressingMode::Implied, 0x02);
    }
}

#[cfg(test)]
mod tests {
    use core::panic;

    use super::*;

    #[test]
    fn cpu_power_up_state() {
        let cpu_mem_map = generate_mem_map(&vec![0x00; 1]);

        let cpu = Cpu::new(cpu_mem_map);

        assert_eq!(cpu.r_pc, 0xAAAA);
        assert_eq!(cpu.r_a, 0x00);
        assert_eq!(cpu.r_x, 0x00);
        assert_eq!(cpu.r_y, 0x00);
        assert_eq!(cpu.r_sp, 0xFD);
        assert_eq!(cpu.r_flags, 0x04);
    }

    #[test]
    fn cpu_instruction_adc() {
        let cpu_mem_map = generate_mem_map(&vec![0x69, 150]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::Carry, true);
        cpu.r_a = 170;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_a, 65);
        assert_eq!(cpu.get_flag(CpuFlag::Carry), true);
        assert_eq!(cpu.get_flag(CpuFlag::Overflow), true);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);
    }

    #[test]
    fn cpu_instruction_and() {
        let cpu_mem_map = generate_mem_map(&vec![0x29, 150]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_a = 170;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_a, 0x82);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), true);
    }

    #[test]
    fn cpu_instruction_asl() {
        let cpu_mem_map = generate_mem_map(&vec![0x0A]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_a = 0x05;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_a, 0x0A);
        assert_eq!(cpu.get_flag(CpuFlag::Carry), false);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);

        let cpu_mem_map = generate_mem_map(&vec![0x06, 0x00]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.write_u8(0x0000, 5);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 5);

        assert_eq!(cpu.read_u8(0x0000), 0x0A);
        assert_eq!(cpu.get_flag(CpuFlag::Carry), false);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);
    }

    #[test]
    fn cpu_instruction_bcc() {
        let cpu_mem_map = generate_mem_map(&vec![0x90, 200]);
        let mut cpu = Cpu::new(cpu_mem_map);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 3);
        assert_eq!(cpu.r_pc, 0xAA74);
    }

    #[test]
    fn cpu_instruction_bcs() {
        let cpu_mem_map = generate_mem_map(&vec![0xB0, 200]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::Carry, true);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 3);
        assert_eq!(cpu.r_pc, 0xAA74);
    }

    #[test]
    fn cpu_instruction_beq() {
        let cpu_mem_map = generate_mem_map(&vec![0xF0, 200]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::Zero, true);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 3);
        assert_eq!(cpu.r_pc, 0xAA74);
    }

    #[test]
    fn cpu_instruction_bit() {
        let mut cpu_mem_map = generate_mem_map(&vec![0x24, 0x00]);
        cpu_mem_map[0x0000] = 0x99;

        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_a = 0x50;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 3);

        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), true);
        assert_eq!(cpu.get_flag(CpuFlag::Overflow), false);
    }

    #[test]
    fn cpu_instruction_bmi() {
        let cpu_mem_map = generate_mem_map(&vec![0x30, 200]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::Negative, true);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 3);
        assert_eq!(cpu.r_pc, 0xAA74);
    }

    #[test]
    fn cpu_instruction_bne() {
        let cpu_mem_map = generate_mem_map(&vec![0xD0, 200]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::Zero, false);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 3);
        assert_eq!(cpu.r_pc, 0xAA74);
    }

    #[test]
    fn cpu_instruction_bpl() {
        let cpu_mem_map = generate_mem_map(&vec![0x10, 200]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::Negative, false);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 3);
        assert_eq!(cpu.r_pc, 0xAA74);
    }

    #[test]
    fn cpu_instruction_brk() {
        let cpu_mem_map = generate_mem_map(&vec![0x00]);
        let mut cpu = Cpu::new(cpu_mem_map);

        let r_pc = cpu.r_pc;
        let flags = cpu.r_flags;
        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 0x07);

        assert_eq!(cpu.get_flag(CpuFlag::Break), true);
        assert_eq!(cpu.r_pc, 0xBBBB);

        let popped_flags = cpu.pop_stack();
        assert_eq!(popped_flags, flags);

        let ll = cpu.pop_stack();
        let hl = cpu.pop_stack();

        let pc = LittleEndian::read_u16(&vec![ll, hl]);

        assert_eq!(pc, r_pc + 1);
    }

    #[test]
    fn cpu_instruction_bvc() {
        let cpu_mem_map = generate_mem_map(&vec![0x50, 200]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::Overflow, false);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 3);
        assert_eq!(cpu.r_pc, 0xAA74);
    }

    #[test]
    fn cpu_instruction_bvs() {
        let cpu_mem_map = generate_mem_map(&vec![0x70, 200]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::Overflow, true);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 3);
        assert_eq!(cpu.r_pc, 0xAA74);
    }

    #[test]
    fn cpu_instruction_clc() {
        let cpu_mem_map = generate_mem_map(&vec![0x18]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::Carry, true);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);
        assert_eq!(cpu.get_flag(CpuFlag::Carry), false);
    }

    #[test]
    fn cpu_instruction_cld() {
        let cpu_mem_map = generate_mem_map(&vec![0xD8]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::Decimal, true);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);
        assert_eq!(cpu.get_flag(CpuFlag::Decimal), false);
    }

    #[test]
    fn cpu_instruction_cli() {
        let cpu_mem_map = generate_mem_map(&vec![0x58]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::InterruptDisable, true);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);
        assert_eq!(cpu.get_flag(CpuFlag::InterruptDisable), false);
    }

    #[test]
    fn cpu_instruction_clv() {
        let cpu_mem_map = generate_mem_map(&vec![0xB8]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::Overflow, true);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);
        assert_eq!(cpu.get_flag(CpuFlag::Overflow), false);
    }

    #[test]
    fn cpu_instruction_cmp() {
        let cpu_mem_map = generate_mem_map(&vec![0xC9, 150]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_a = 200;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.get_flag(CpuFlag::Carry), true);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);
    }

    #[test]
    fn cpu_instruction_cpx() {
        let cpu_mem_map = generate_mem_map(&vec![0xE0, 150]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_x = 200;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.get_flag(CpuFlag::Carry), true);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);
    }

    #[test]
    fn cpu_instruction_cpy() {
        let cpu_mem_map = generate_mem_map(&vec![0xC0, 150]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_y = 200;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.get_flag(CpuFlag::Carry), true);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);
    }

    #[test]
    fn cpu_instruction_dec() {
        let cpu_mem_map = generate_mem_map(&vec![0xC6, 0x00]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.mem_map[0x0000] = 1;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 5);

        assert_eq!(cpu.read_u8(0x0000), 0x00);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), true);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);
    }

    #[test]
    fn cpu_instruction_dex() {
        let cpu_mem_map = generate_mem_map(&vec![0xCA]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_x = 1;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_x, 0x00);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), true);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);
    }

    #[test]
    fn cpu_instruction_dey() {
        let cpu_mem_map = generate_mem_map(&vec![0x88]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_y = 1;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_y, 0x00);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), true);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);
    }

    #[test]
    fn cpu_instruction_eor() {
        let cpu_mem_map = generate_mem_map(&vec![0x49, 150]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_a = 170;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_a, 0x3C);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);
    }

    #[test]
    fn cpu_instruction_inc() {
        let cpu_mem_map = generate_mem_map(&vec![0xE6, 0x00]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.mem_map[0x0000] = 0xFF;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 5);

        assert_eq!(cpu.read_u8(0x0000), 0x00);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), true);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);
    }

    #[test]
    fn cpu_instruction_inx() {
        let cpu_mem_map = generate_mem_map(&vec![0xE8]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_x = 0xFF;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_x, 0x00);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), true);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);
    }

    #[test]
    fn cpu_instruction_iny() {
        let cpu_mem_map = generate_mem_map(&vec![0xC8]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_y = 0xFF;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_y, 0x00);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), true);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);
    }

    #[test]
    fn cpu_instruction_jmp() {
        let cpu_mem_map = generate_mem_map(&vec![0x4C, 0xCD, 0xAB]);
        let mut cpu = Cpu::new(cpu_mem_map);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 3);

        assert_eq!(cpu.r_pc, 0xABCD);
    }

    #[test]
    fn cpu_instruction_jsr() {
        let cpu_mem_map = generate_mem_map(&vec![0x20, 0xBB, 0xAA]);
        let mut cpu = Cpu::new(cpu_mem_map);

        let r_pc = cpu.r_pc + 2;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 0x06);
        assert_eq!(cpu.r_pc, 0xAABB);

        let ll = cpu.pop_stack();
        let hl = cpu.pop_stack();

        let pc = LittleEndian::read_u16(&vec![ll, hl]);

        assert_eq!(pc, r_pc);
    }

    #[test]
    fn cpu_instruction_lda() {
        let cpu_mem_map = generate_mem_map(&vec![0xA9, 0xBB]);
        let mut cpu = Cpu::new(cpu_mem_map);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);
        assert_eq!(cpu.r_a, 0xBB);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), true);
    }

    #[test]
    fn cpu_instruction_ldx() {
        let cpu_mem_map = generate_mem_map(&vec![0xA2, 0xBB]);
        let mut cpu = Cpu::new(cpu_mem_map);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);
        assert_eq!(cpu.r_x, 0xBB);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), true);
    }

    #[test]
    fn cpu_instruction_ldy() {
        let cpu_mem_map = generate_mem_map(&vec![0xA0, 0xBB]);
        let mut cpu = Cpu::new(cpu_mem_map);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);
        assert_eq!(cpu.r_y, 0xBB);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), true);
    }

    #[test]
    fn cpu_instruction_lsr() {
        let cpu_mem_map = generate_mem_map(&vec![0x4A]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_a = 0xAA;

        cpu.set_flag(CpuFlag::Carry, true);
        cpu.set_flag(CpuFlag::Zero, true);
        cpu.set_flag(CpuFlag::Negative, true);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_a, 0x55);
        assert_eq!(cpu.get_flag(CpuFlag::Carry), false);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);

        let cpu_mem_map = generate_mem_map(&vec![0x46, 0x00]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::Carry, true);
        cpu.set_flag(CpuFlag::Zero, true);
        cpu.set_flag(CpuFlag::Negative, true);
        cpu.write_u8(0x0000, 0xAA);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 5);

        assert_eq!(cpu.read_u8(0x0000), 0x55);
        assert_eq!(cpu.get_flag(CpuFlag::Carry), false);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);
    }

    #[test]
    fn cpu_instruction_ora() {
        let cpu_mem_map = generate_mem_map(&vec![0x09, 150]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_a = 170;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_a, 0xBE);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), true);
    }

    #[test]
    fn cpu_instruction_pha() {
        let cpu_mem_map = generate_mem_map(&vec![0x48]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_a = 0xAA;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 3);

        let popped_a = cpu.pop_stack();

        assert_eq!(popped_a, 0xAA);
    }

    #[test]
    fn cpu_instruction_php() {
        let cpu_mem_map = generate_mem_map(&vec![0x08]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_flags = 0xAA;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 3);

        let popped_a = cpu.pop_stack();

        assert_eq!(popped_a, 0xAA);
    }

    #[test]
    fn cpu_instruction_pla() {
        let cpu_mem_map = generate_mem_map(&vec![0x68]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.push_stack(0xAA);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 4);
        assert_eq!(cpu.r_a, 0xAA);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), true);
    }

    #[test]
    fn cpu_instruction_plp() {
        let cpu_mem_map = generate_mem_map(&vec![0x28]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.push_stack(0xAA);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 4);
        assert_eq!(cpu.r_flags, 0xAA);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), true);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), true);
    }

    #[test]
    fn cpu_instruction_rol() {
        let cpu_mem_map = generate_mem_map(&vec![0x2A]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_a = 0x10;

        cpu.set_flag(CpuFlag::Carry, true);
        cpu.set_flag(CpuFlag::Zero, true);
        cpu.set_flag(CpuFlag::Negative, true);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_a, 0x21);
        assert_eq!(cpu.get_flag(CpuFlag::Carry), false);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);

        let cpu_mem_map = generate_mem_map(&vec![0x26, 0x00]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::Carry, true);
        cpu.set_flag(CpuFlag::Zero, true);
        cpu.set_flag(CpuFlag::Negative, true);
        cpu.write_u8(0x0000, 0x10);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 5);

        assert_eq!(cpu.read_u8(0x0000), 0x21);
        assert_eq!(cpu.get_flag(CpuFlag::Carry), false);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);
    }

    #[test]
    fn cpu_instruction_ror() {
        let cpu_mem_map = generate_mem_map(&vec![0x6A]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_a = 0xAA;

        cpu.set_flag(CpuFlag::Carry, true);
        cpu.set_flag(CpuFlag::Zero, true);
        cpu.set_flag(CpuFlag::Negative, false);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_a, 0xD5);
        assert_eq!(cpu.get_flag(CpuFlag::Carry), false);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), true);

        let cpu_mem_map = generate_mem_map(&vec![0x66, 0x00]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::Carry, true);
        cpu.set_flag(CpuFlag::Zero, true);
        cpu.set_flag(CpuFlag::Negative, false);
        cpu.write_u8(0x0000, 0xAA);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 5);

        assert_eq!(cpu.read_u8(0x0000), 0xD5);
        assert_eq!(cpu.get_flag(CpuFlag::Carry), false);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), true);
    }

    #[test]
    fn cpu_instruction_rti() {
        let cpu_mem_map = generate_mem_map(&vec![0x40]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.push_stack(0xBB);
        cpu.push_stack(0xAA);
        cpu.push_stack(0x10);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 6);
        assert_eq!(cpu.r_pc, 0xBBAA);
        assert_eq!(cpu.r_flags, 0x10);
    }

    #[test]
    fn cpu_instruction_rts() {
        let cpu_mem_map = generate_mem_map(&vec![0x60]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.push_stack(0xBB);
        cpu.push_stack(0xA9);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 6);
        assert_eq!(cpu.r_pc, 0xBBAA);
    }

    #[test]
    fn cpu_instruction_sbc() {
        let cpu_mem_map = generate_mem_map(&vec![0xE9, 149]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::Carry, true);
        cpu.r_a = 170;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_a, 20);
        assert_eq!(cpu.get_flag(CpuFlag::Carry), false);
        assert_eq!(cpu.get_flag(CpuFlag::Overflow), true);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), false);
    }

    #[test]
    fn cpu_instruction_sec() {
        let cpu_mem_map = generate_mem_map(&vec![0x38]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::Carry, false);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);
        assert_eq!(cpu.get_flag(CpuFlag::Carry), true);
    }

    #[test]
    fn cpu_instruction_sed() {
        let cpu_mem_map = generate_mem_map(&vec![0xF8]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::Decimal, false);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);
        assert_eq!(cpu.get_flag(CpuFlag::Decimal), true);
    }

    #[test]
    fn cpu_instruction_sei() {
        let cpu_mem_map = generate_mem_map(&vec![0x78]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.set_flag(CpuFlag::InterruptDisable, false);

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);
        assert_eq!(cpu.get_flag(CpuFlag::InterruptDisable), true);
    }

    #[test]
    fn cpu_instruction_sta() {
        let cpu_mem_map = generate_mem_map(&vec![0x85, 0x00]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_a = 100;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 3);
        assert_eq!(cpu.read_u8(0x0000), 100);
    }

    #[test]
    fn cpu_instruction_stx() {
        let cpu_mem_map = generate_mem_map(&vec![0x86, 0x00]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_x = 100;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 3);
        assert_eq!(cpu.read_u8(0x0000), 100);
    }

    #[test]
    fn cpu_instruction_sty() {
        let cpu_mem_map = generate_mem_map(&vec![0x84, 0x00]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_y = 100;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 3);
        assert_eq!(cpu.read_u8(0x0000), 100);
    }

    #[test]
    fn cpu_instruction_tax() {
        let cpu_mem_map = generate_mem_map(&vec![0xAA]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_a = 150;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_a, cpu.r_x);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), true);
    }

    #[test]
    fn cpu_instruction_tay() {
        let cpu_mem_map = generate_mem_map(&vec![0xA8]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_a = 150;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_a, cpu.r_y);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), true);
    }

    #[test]
    fn cpu_instruction_tsx() {
        let cpu_mem_map = generate_mem_map(&vec![0xBA]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_sp = 150;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_sp, cpu.r_x);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), true);
    }

    #[test]
    fn cpu_instruction_txa() {
        let cpu_mem_map = generate_mem_map(&vec![0x8A]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_x = 150;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_x, cpu.r_a);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), true);
    }

    #[test]
    fn cpu_instruction_txs() {
        let cpu_mem_map = generate_mem_map(&vec![0x9A]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_x = 150;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_x, cpu.r_sp);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), true);
    }

    #[test]
    fn cpu_instruction_tya() {
        let cpu_mem_map = generate_mem_map(&vec![0x98]);
        let mut cpu = Cpu::new(cpu_mem_map);

        cpu.r_y = 150;

        let cycles = cpu.exec_instruction();

        assert_eq!(cycles, 2);

        assert_eq!(cpu.r_y, cpu.r_a);
        assert_eq!(cpu.get_flag(CpuFlag::Zero), false);
        assert_eq!(cpu.get_flag(CpuFlag::Negative), true);
    }

    fn generate_mem_map(instructions: &[u8]) -> CpuMemMap {
        let mut cpu_mem_map = vec![0x00; 0x10_000];

        cpu_mem_map[0xFFFC] = 0xAA;
        cpu_mem_map[0xFFFD] = 0xAA;
        cpu_mem_map[0xFFFE] = 0xBB;
        cpu_mem_map[0xFFFF] = 0xBB;

        match instructions.len() {
            1 => cpu_mem_map[0xAAAA] = instructions[0],
            2 => cpu_mem_map[0xAAAA..0xAAAC].copy_from_slice(instructions),
            3 => cpu_mem_map[0xAAAA..0xAAAD].copy_from_slice(instructions),
            _ => panic!("Too less or big instruction size"),
        }

        cpu_mem_map
    }
}
