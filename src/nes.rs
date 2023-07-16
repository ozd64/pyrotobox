use std::error::Error;
use std::fmt::Display;

use crate::bin_ops::read_le_u32;
use crate::cpu::Cpu;
use crate::mapper::{get_mem_map_by_mapper, Cartridge};

const INES_HEADER_SIGNATURE: u32 = 0x1A53454E;
const MIN_NES_ROM_FILE_SIZE_IN_BYTES: usize = 0x10;

#[derive(Debug, PartialEq, Eq)]
pub enum Mirroring {
    Horizontal,
    Vertical,
}

#[derive(Debug, PartialEq, Eq)]
pub enum Mapper {
    NROM,
}

pub struct NesHeader {
    mapper: Mapper,
    mirroring: Mirroring,
    prg_rom_size: usize,
    chr_rom_size: usize,
    battery_backed_prg_ram: bool,
}

#[derive(Debug, PartialEq, Eq)]
pub enum NesError {
    TooSmallRomFile(usize),
    InvalidSignature(u32),
    UnknownMapper(u8),
}

impl Display for NesError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NesError::InvalidSignature(invalid_sign) => write!(
                f,
                "Invalid iNES signature. Read signature: 0x{:X}",
                invalid_sign
            ),
            NesError::TooSmallRomFile(file_size) => write!(
                f,
                "The file read is too small to be a NES ROM file. File size: {} bytes.",
                file_size
            ),
            NesError::UnknownMapper(mapper_code) => {
                write!(f, "Unknown NES Mapper. Code: {}", mapper_code)
            }
        }
    }
}

impl Error for NesError {}

impl NesHeader {
    pub fn new(rom_bin: &[u8]) -> Result<NesHeader, NesError> {
        let rom_bin_len = rom_bin.len();

        if rom_bin_len < MIN_NES_ROM_FILE_SIZE_IN_BYTES {
            return Err(NesError::TooSmallRomFile(rom_bin_len));
        }

        let read_signature = read_le_u32(rom_bin);

        if read_signature != INES_HEADER_SIGNATURE {
            return Err(NesError::InvalidSignature(read_signature));
        }

        let prg_rom_size = rom_bin[4] as usize;
        let chr_rom_size = rom_bin[5] as usize;

        let mirroring = if (rom_bin[6] & 0b000_000_01) == 1 {
            Mirroring::Vertical
        } else {
            Mirroring::Horizontal
        };

        let battery_backed_prg_ram = (rom_bin[6] & 0b000_000_10) > 0;
        let mapper_code = rom_bin[6] >> 4;

        let mapper = match mapper_code {
            0 => Mapper::NROM,
            _ => return Err(NesError::UnknownMapper(mapper_code)),
        };

        Ok(Self {
            mapper,
            mirroring,
            prg_rom_size,
            chr_rom_size,
            battery_backed_prg_ram,
        })
    }

    pub fn prg_rom_size(&self) -> usize {
        self.prg_rom_size
    }
}

pub struct Nes {
    rom_bin: Vec<u8>,
    cpu: Cpu,
}

impl Nes {
    pub fn new(rom_bin: Vec<u8>) -> Result<Self, NesError> {
        let nes_header = NesHeader::new(&rom_bin)?;
        let cartridge = get_mem_map_by_mapper(&nes_header.mapper);

        let cpu_mem_map = cartridge.get_cpu_mem_map(&nes_header, &rom_bin);
        let cpu = Cpu::new(cpu_mem_map);

        Ok(Self { rom_bin, cpu })
    }

    pub fn run(&mut self) {
        let _ = self.cpu.exec_instruction();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_too_small_nes_rom_bin() {
        let rom_bin: Vec<u8> = vec![0x4E];
        let nes_header = NesHeader::new(&rom_bin);

        assert!(nes_header.is_err());

        let error = nes_header.err().unwrap();
        assert_eq!(error, NesError::TooSmallRomFile(1));
    }

    #[test]
    fn test_invalid_nes_header_signature() {
        let rom_bin: Vec<u8> = vec![
            0x4F, 0x45, 0x53, 0x1A, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ];
        let nes_header = NesHeader::new(&rom_bin);

        assert!(nes_header.is_err());

        let error = nes_header.err().unwrap();
        assert_eq!(error, NesError::InvalidSignature(0x1A53454F));
    }

    #[test]
    fn test_valid_nes_header() {
        let rom_bin: Vec<u8> = vec![
            0x4E, 0x45, 0x53, 0x1A, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ];
        let nes_header = NesHeader::new(&rom_bin).unwrap();

        assert_eq!(nes_header.mapper, Mapper::NROM);
        assert_eq!(nes_header.chr_rom_size, 1);
        assert_eq!(nes_header.prg_rom_size, 2);
        assert_eq!(nes_header.battery_backed_prg_ram, false);
        assert_eq!(nes_header.mirroring, Mirroring::Vertical);
    }
}
