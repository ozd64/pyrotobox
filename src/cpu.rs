use crate::bin_ops::read_le_u16;

pub struct Cpu {
    r_pc: u16,
    r_a: u8,
    r_x: u8,
    r_y: u8,
    r_sp: u8,
    r_sr: u8,
    memory: Vec<u8>,
}

impl Cpu {
    pub fn new(memory: Vec<u8>) -> Self {
        let r_pc = read_le_u16(&memory[0xFFFC..0xFFFE]);

        Self {
            r_pc,
            r_a: 0,
            r_x: 0,
            r_y: 0,
            r_sp: 0xFD,
            r_sr: 0x34,
            memory,
        }
    }

    pub fn exec_instruction(&mut self) -> usize {
        let opcode = self.read_u8(self.r_pc);

        todo!()
    }

    fn read_u8(&self, addr: u16) -> u8 {
        match addr {
            0x0000..=0x1FFF => {
                let fmt_addr: usize = (addr & 0b0_0111_1111_1111) as usize;
                return self.memory[fmt_addr];
            }
            //PPU registers
            0x2000..=0x3FFF => {
                let fmt_addr: usize = (addr & 0x2007) as usize;
                return self.memory[fmt_addr];
            }
            _ => self.memory[addr as usize],
        }
    }
}

#[cfg(test)]
mod tests {

    #[test]
    fn test_adc_instruction() {
        //TODO: Fix the test below
        assert!(false);
    }
}
