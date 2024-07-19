use std::fmt::Display;
use std::fs::File;
use std::io::{BufReader, Error, Read};
use std::path::Path;

use byteorder::{ByteOrder, LittleEndian};

const MAX_NES_ROM_FILE_SIZE: u64 = 0x500000;
const MIN_NES_ROM_FILE_SIZE: u64 = 0x10;
const INES_SIGNATURE: u32 = 0x1A53454E;

#[derive(Debug)]
pub enum RomError {
    RomFileTooBig(u64),
    RomFileTooSmall(u64),
    IOError(Error),
    InvalidINESSignature(u32),
    UnsupportedMapper(u8),
}

impl Display for RomError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RomError::RomFileTooBig(size) => {
                write!(f, "File is too big for a NES ROM. File Size: {}", size)
            }
            RomError::RomFileTooSmall(size) => {
                write!(f, "File is too small for a NES ROM. File Size: {}", size)
            }
            RomError::IOError(err) => write!(
                f,
                "An IO error occurred while processing the ROM file.\n{}",
                err
            ),
            RomError::InvalidINESSignature(sign) => {
                write!(f, "Invalid iNES signature. Read Signature: 0x{:X}", sign)
            }
            RomError::UnsupportedMapper(mapper_code) => {
                write!(f, "Invalid iNES mapper code. Code: {}", mapper_code)
            }
        }
    }
}

impl std::error::Error for RomError {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Mirroring {
    Horizontal,
    Vertical,
}

impl Display for Mirroring {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Mirroring::Horizontal => write!(f, "Horizontal"),
            Mirroring::Vertical => write!(f, "Vertical"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mapper {
    NROM,
}

impl Display for Mapper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Mapper::NROM => write!(f, "NROM"),
        }
    }
}

pub struct Rom {
    rom_binary: Vec<u8>,
    mapper: Mapper,
    prg_rom_size: usize,
    chr_rom_size: usize,
    battery_backed_prg_ram: bool,
    mirroring: Mirroring,
}

impl Rom {
    pub fn from_ines<P: AsRef<Path>>(rom_file_path: P) -> Result<Rom, RomError> {
        let rom_bin = Rom::get_rom_file_content(rom_file_path)?;
        Rom::from_rom_bin(rom_bin)
    }

    fn from_rom_bin(rom_bin: Vec<u8>) -> Result<Rom, RomError> {
        let read_sign = LittleEndian::read_u32(&rom_bin[..4]);

        if read_sign != INES_SIGNATURE {
            return Err(RomError::InvalidINESSignature(read_sign));
        }

        let prg_rom_size = rom_bin[4] as usize;
        let chr_rom_size = rom_bin[5] as usize;
        let mapper_code = rom_bin[6] >> 4;

        let mapper = match mapper_code {
            0 => Mapper::NROM,
            _ => return Err(RomError::UnsupportedMapper(mapper_code)),
        };

        let mirroring = if (rom_bin[6] & 0x01) > 0 {
            Mirroring::Horizontal
        } else {
            Mirroring::Vertical
        };
        let battery_backed_prg_ram = (rom_bin[6] & 0x00000010) > 1;

        Ok(Self {
            rom_binary: rom_bin,
            mapper,
            prg_rom_size,
            chr_rom_size,
            battery_backed_prg_ram,
            mirroring,
        })
    }

    fn get_rom_file_content<P>(path: P) -> Result<Vec<u8>, RomError>
    where
        P: AsRef<Path>,
    {
        let file = File::open(path).map_err(|err| RomError::IOError(err))?;
        let file_size = file.metadata().map_err(|err| RomError::IOError(err))?.len();

        if file_size < MIN_NES_ROM_FILE_SIZE {
            return Err(RomError::RomFileTooSmall(file_size));
        } else if file_size > MAX_NES_ROM_FILE_SIZE {
            return Err(RomError::RomFileTooBig(file_size));
        }

        let mut buf_reader = BufReader::new(file);

        let mut rom_bytes = Vec::<u8>::with_capacity(file_size as usize);

        buf_reader
            .read_to_end(&mut rom_bytes)
            .map_err(|err| RomError::IOError(err))?;

        Ok(rom_bytes)
    }

    pub fn mapper(&self) -> Mapper {
        self.mapper
    }

    pub fn prg_rom_size(&self) -> usize {
        self.prg_rom_size
    }

    pub fn chr_rom_size(&self) -> usize {
        self.chr_rom_size
    }

    pub fn battery_backed_prg_ram(&self) -> bool {
        self.battery_backed_prg_ram
    }

    pub fn mirroring(&self) -> Mirroring {
        self.mirroring
    }
}

#[cfg(test)]
mod tests {

    use core::panic;

    use super::*;

    #[test]
    fn rom_bytes_should_have_correct_ines_signature() {
        let bytes = vec![
            0x4E, 0x45, 0x53, 0x1B, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ];
        let rom = Rom::from_rom_bin(bytes);

        assert!(rom.is_err());

        let err = rom.err().unwrap();

        if let RomError::InvalidINESSignature(sign) = err {
            assert_eq!(0x1B53454E, sign)
        } else {
            panic!("The Rom Error was not invalid iNES signature error")
        }
    }

    #[test]
    fn rom_mapper_should_be_supported_mapper() {
        let bytes = vec![
            0x4E, 0x45, 0x53, 0x1A, 0x02, 0x01, 0xF1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ];
        let rom = Rom::from_rom_bin(bytes);

        assert!(rom.is_err());

        let err = rom.err().unwrap();

        if let RomError::UnsupportedMapper(mapper_code) = err {
            assert_eq!(0xF, mapper_code)
        } else {
            panic!("The Rom Error was not invalid iNES signature error")
        }
    }

    #[test]
    fn rom_should_be_parsed_successfully() {
        let bytes = vec![
            0x4E, 0x45, 0x53, 0x1A, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ];
        let rom = Rom::from_rom_bin(bytes).unwrap();

        assert_eq!(Mapper::NROM, rom.mapper());
        assert_eq!(2, rom.prg_rom_size());
        assert_eq!(1, rom.chr_rom_size());
        assert_eq!(Mirroring::Horizontal, rom.mirroring());
        assert!(!rom.battery_backed_prg_ram());
    }
}
