use crate::nes::{Mapper, NesHeader};

const CPU_MEM_MAP_SIZE: usize = 0x10000;

pub trait Cartridge {
    fn get_cpu_mem_map(&self, nes_header: &NesHeader, rom_bin: &[u8]) -> Vec<u8>;
    fn get_ppu_mem_map(&self, nes_header: &NesHeader, rom_bin: &[u8]) -> Vec<u8>;
}

struct NROMCartridge;

impl Cartridge for NROMCartridge {
    fn get_cpu_mem_map(&self, nes_header: &NesHeader, rom_bin: &[u8]) -> Vec<u8> {
        let mut cpu_mem_map = vec![0x00; CPU_MEM_MAP_SIZE];

        if nes_header.prg_rom_size() == 2 {
            cpu_mem_map[0x8000..0xC000].copy_from_slice(&rom_bin[0x10..0x4010]);
            cpu_mem_map[0xC000..0x10000].copy_from_slice(&rom_bin[0x4010..0x8010]);
        } else {
            //Mirror the first PRG
            cpu_mem_map[0x8000..0xC000].copy_from_slice(&rom_bin[0x10..0x4010]);
            cpu_mem_map[0xC000..0x10000].copy_from_slice(&rom_bin[0x10..0x4010]);
        }

        cpu_mem_map
    }

    fn get_ppu_mem_map(&self, nes_header: &NesHeader, rom_bin: &[u8]) -> Vec<u8> {
        todo!()
    }
}

pub fn get_mem_map_by_mapper(mapper: &Mapper) -> impl Cartridge {
    match mapper {
        Mapper::NROM => NROMCartridge,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bin_ops::read_le_u16;

    #[test]
    fn test_nrom_cartridge() {
        let mut rom_bin = vec![0; 0x8010];

        rom_bin[0x00..0x10].copy_from_slice(&vec![
            0x4E, 0x45, 0x53, 0x1A, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ]);

        let nes_header = NesHeader::new(&rom_bin).unwrap();

        rom_bin[0x8010 - 4] = 0xFF;
        rom_bin[0x8010 - 3] = 0xAA;

        let cpu_mem_map =
            get_mem_map_by_mapper(&Mapper::NROM).get_cpu_mem_map(&nes_header, &rom_bin);

        assert_eq!(read_le_u16(&cpu_mem_map[0xFFFC..0xFFFE]), 0xAAFF);
    }
}
