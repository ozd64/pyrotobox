use crate::rom::Rom;

const CPU_MEM_MAP_SIZE: usize = 0x10_000;
const PPU_MEM_MAP_SIZE: usize = 0x4000;
const PATTERN_TABLE_SIZE: usize = 0x2000;

const TRAINER_SIZE: usize = 512;
const NROM_PRG_ROM_SIZE: usize = 0x4000;

pub type CpuMemMap = Vec<u8>;
pub type PpuMemMap = Vec<u8>;

pub trait Mapper {
    fn generate_cpu_mem_map(&self, rom: &Rom) -> CpuMemMap;
    fn generate_ppu_mem_map(&self, rom: &Rom) -> PpuMemMap;
}

pub struct NROMMapper;

impl Mapper for NROMMapper {
    fn generate_cpu_mem_map(&self, rom: &Rom) -> CpuMemMap {
        let mut cpu_mem_map = vec![0x00; CPU_MEM_MAP_SIZE];

        let trainer_size = if rom.rom_trainer_exists() {
            TRAINER_SIZE
        } else {
            0
        };

        // 0x10 is for iNES header size
        let prg_rom_first_bank_start_offset = 0x10 + trainer_size;
        let prg_rom_second_bank_start_offset = prg_rom_first_bank_start_offset + NROM_PRG_ROM_SIZE;
        let prg_rom_second_bank_end_offset = prg_rom_second_bank_start_offset + NROM_PRG_ROM_SIZE;

        cpu_mem_map[0x8000..0xC000].copy_from_slice(
            &rom.rom_binary()[prg_rom_first_bank_start_offset..prg_rom_second_bank_start_offset],
        );

        if rom.prg_rom_size() > 1 {
            cpu_mem_map[0xC000..0x10000].copy_from_slice(
                &rom.rom_binary()[prg_rom_second_bank_start_offset..prg_rom_second_bank_end_offset],
            );
        } else {
            // If the prg size is less than 2 then mirror the first bank from the rom itself.
            cpu_mem_map[0xC000..0x10000].copy_from_slice(
                &rom.rom_binary()
                    [prg_rom_first_bank_start_offset..prg_rom_second_bank_start_offset],
            );
        }

        cpu_mem_map
    }

    fn generate_ppu_mem_map(&self, rom: &Rom) -> PpuMemMap {
        let mut ppu_mem_map = vec![0x00; PPU_MEM_MAP_SIZE];

        let pattern_table_start_offset = 0x8010;
        let pattern_table_end_offset = pattern_table_start_offset + PATTERN_TABLE_SIZE;

        ppu_mem_map[0x000..0x2000].copy_from_slice(
            &rom.rom_binary()[pattern_table_start_offset..pattern_table_end_offset],
        );

        ppu_mem_map
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nrom_mapper_implementation() {
        let rom_bin = get_test_rom_bin();
        let rom = Rom::from_rom_bin(rom_bin).unwrap();

        let nrom_mapper = NROMMapper;
        let cpu_mem_map = nrom_mapper.generate_cpu_mem_map(&rom);
        let ppu_mem_map = nrom_mapper.generate_ppu_mem_map(&rom);

        assert_eq!(cpu_mem_map[0x8000], 0xCE);
        assert_eq!(cpu_mem_map[0xC000], 0xEC);
        assert_eq!(ppu_mem_map[0x0000], 0xAA);
        assert_eq!(ppu_mem_map[0x0010], 0xBB);
    }

    fn get_test_rom_bin() -> Vec<u8> {
        let mut test_rom_bin = vec![0x00; 50000];
        let ines_header = vec![
            0x4E, 0x45, 0x53, 0x1A, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ];

        test_rom_bin[0x00..0x10].copy_from_slice(&ines_header[..0x10]);
        test_rom_bin[0x10] = 0xCE;
        test_rom_bin[0x10 + NROM_PRG_ROM_SIZE] = 0xEC;
        test_rom_bin[0x8010] = 0xAA;
        test_rom_bin[0x8020] = 0xBB;

        test_rom_bin
    }
}
