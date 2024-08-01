use std::error::Error;
use std::{fmt::Display, path::Path};

use crate::cpu::Cpu;
use crate::mapper::NROMMapper;
use crate::rom::{Mapper, Rom, RomError};

#[derive(Debug)]
pub enum NesError {
    RomError(RomError),
}

impl Display for NesError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NesError::RomError(err) => {
                write!(f, "An error occured during NES ROM creation\n{}", err)
            }
        }
    }
}

impl Error for NesError {}

pub struct Nes {
    cpu: Cpu,
}

impl Nes {
    pub fn new<P: AsRef<Path>>(rom_file_path: P) -> Result<Self, NesError> {
        let rom = Rom::from_ines(rom_file_path).map_err(|err| NesError::RomError(err))?;

        let rom_mapper: Box<dyn crate::mapper::Mapper> = match rom.mapper() {
            Mapper::NROM => Box::new(NROMMapper),
        };

        let cpu_mem_map = rom_mapper.generate_cpu_mem_map(&rom);

        let cpu = Cpu::new(cpu_mem_map);

        Ok(Nes { cpu })
    }

    pub fn start(&self) {
        //TODO: Implement NES processing
    }
}
