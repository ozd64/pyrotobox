use byteorder::{ByteOrder, LittleEndian};

use crate::mapper::CpuMemMap;

pub struct Cpu {
    mem_map: CpuMemMap,
    r_pc: u16,
    r_x: u8,
    r_y: u8,
    r_a: u8,
    r_sp: u8,
    r_flags: u8
}

impl Cpu {
    pub fn new(cpu_mem_map: CpuMemMap) -> Self {
        let r_pc = Cpu::reset_vector(&cpu_mem_map);

        Self {
            mem_map: cpu_mem_map,
            r_pc,
            r_x: 0,
            r_y: 0,
            r_a: 0,
            r_sp: 0xFD,
            r_flags: 0x04
        }
    }

    #[inline]
    fn reset_vector(cpu_mem_map: &CpuMemMap) -> u16 {
        LittleEndian::read_u16(&cpu_mem_map[0xFFFC..0xFFFE])
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

    fn generate_mem_map(instructions: &[u8]) -> CpuMemMap {
        let mut cpu_mem_map = vec![0x00; 0x10_000];

        cpu_mem_map[0xFFFC] = 0xAA;
        cpu_mem_map[0xFFFD] = 0xAA;

        match instructions.len() {
            1 => cpu_mem_map[0xAAAA] = instructions[0],
            2 => cpu_mem_map[0xAAAA..0xAAAC].copy_from_slice(instructions),
            3 => cpu_mem_map[0xAAAA..0xAAAD].copy_from_slice(instructions), 
            _ => panic!("Too less or big instruction size")
        }

        cpu_mem_map
    }

}
