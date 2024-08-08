use byteorder::{ByteOrder, LittleEndian};

use crate::mapper::CpuMemMap;

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
    Carry = 0,
    Zero = 1,
    InterruptDisable = 2,
    Decimal = 3,
    Break = 4,
    Overflow = 6,
    Negative = 7,
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
        let opcode = self.read_u8(self.r_pc);

        let (operand_value, total_cycles, addr_mode) = self.get_operand_value(opcode);

        match opcode {
            //ADC
            0x69 | 0x65 | 0x75 | 0x6D | 0x7D | 0x79 | 0x61 | 0x71 => self.adc(operand_value),
            0x29 | 0x25 | 0x35 | 0x2D | 0x3D | 0x39 | 0x21 | 0x31 => self.and(operand_value),
            _ => panic!(
                "FATAL: Unknown CPU instruction opcode. Read Opcode: 0x{:X} at the address 0x{:X}",
                opcode, self.r_pc
            ),
        };

        total_cycles as usize
    }

    fn adc(&mut self, value: u8) {
        let mut new_acc_value = (self.r_a as u16).wrapping_add(value as u16);

        if self.get_flag(CpuFlag::Carry) { 
            new_acc_value = new_acc_value.wrapping_add(1);
        }

        let overflow = (!((self.r_a as u16) ^ (value as u16)) & ((self.r_a as u16) ^ new_acc_value)) & 0x0080;

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

    #[inline]
    fn nop(&self) {
        ()
    }

    fn get_operand_value(&mut self, opcode: u8) -> (u8, u8, AddressingMode) {
        let instruction = &self.instructions[opcode as usize];

        match instruction.1 {
            AddressingMode::Implied => {
                self.r_pc += 1;
                (opcode, instruction.2, instruction.1)
            }
            AddressingMode::Accumulator => {
                self.r_pc += 1;
                (self.r_a, instruction.2, instruction.1)
            }
            AddressingMode::Immediate => {
                let value = self.read_u8(self.r_pc + 1);
                self.r_pc += 2;

                (value, instruction.2, instruction.1)
            }
            AddressingMode::ZeroPage => {
                let zero_page_addr = self.read_u8(self.r_pc + 1);
                let value = self.read_u8(zero_page_addr as u16);

                self.r_pc += 2;
                (value, instruction.2, instruction.1)
            }
            AddressingMode::ZeroPageX => {
                let zero_page_addr = self.read_u8(self.r_pc + 1) as u16;
                let zero_page_value = self.read_u8(zero_page_addr);

                let value = zero_page_value.wrapping_add(self.r_x);

                self.r_pc += 2;

                (value, instruction.2, instruction.1)
            }
            AddressingMode::ZeroPageY => {
                let zero_page_addr = self.read_u8(self.r_pc + 1) as u16;
                let zero_page_value = self.read_u8(zero_page_addr);

                let value = zero_page_value.wrapping_add(self.r_y);

                self.r_pc += 2;

                (value, instruction.2, instruction.1)
            }
            AddressingMode::Relative => {
                // Relative mode is handled within branch instructions
                (opcode, 0, instruction.1)
            }
            AddressingMode::Absolute => {
                let le_addr_arr = vec![self.read_u8(self.r_pc + 1), self.read_u8(self.r_pc + 2)];
                let addr = LittleEndian::read_u16(&le_addr_arr);

                let value = self.read_u8(addr);

                self.r_pc += 3;

                (value, instruction.2, instruction.1)
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

                (value, instruction.2 + extra_cycles, instruction.1)
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

                (value, instruction.2 + extra_cycles, instruction.1)
            }
            AddressingMode::Indirect => {
                //Indirect is implemented directly in JMP instruction
                (opcode, 0, instruction.1)
            }
            AddressingMode::IndirectX => {
                let base = self.read_u8(self.r_pc + 1);
                let ptr = base.wrapping_add(self.r_x);

                let lo = self.read_u8(ptr as u16);
                let hi = self.read_u8(ptr.wrapping_add(1) as u16);

                let addr = ((hi as u16) << 8) | lo as u16;

                let value = self.read_u8(addr);

                self.r_pc += 2;

                (value, instruction.2, instruction.1)
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

                (value, instruction.2 + extra_cycles, instruction.1)
            }
        }
    }

    fn get_flag(&self, flag: CpuFlag) -> bool {
        let bit_mask = 1 << flag as u8;

        (self.r_flags & bit_mask) > 0
    }

    fn set_flag(&mut self, flag: CpuFlag, value: bool) {
        let flag_value = value as u8;
        let bit_mask = flag_value << flag as u8;

        self.r_flags = 0xFF & bit_mask;
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

    #[inline]
    fn reset_vector(cpu_mem_map: &CpuMemMap) -> u16 {
        LittleEndian::read_u16(&cpu_mem_map[0xFFFC..0xFFFE])
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
        assert_eq!(cpu.get_flag(CpuFlag::Carry), false);
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

    fn generate_mem_map(instructions: &[u8]) -> CpuMemMap {
        let mut cpu_mem_map = vec![0x00; 0x10_000];

        cpu_mem_map[0xFFFC] = 0xAA;
        cpu_mem_map[0xFFFD] = 0xAA;

        match instructions.len() {
            1 => cpu_mem_map[0xAAAA] = instructions[0],
            2 => cpu_mem_map[0xAAAA..0xAAAC].copy_from_slice(instructions),
            3 => cpu_mem_map[0xAAAA..0xAAAD].copy_from_slice(instructions),
            _ => panic!("Too less or big instruction size"),
        }

        cpu_mem_map
    }

}
