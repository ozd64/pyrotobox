use std::env::args;
use std::fs::File;
use std::io::Read;
use std::io::{Error, ErrorKind};
use std::path::{Path, PathBuf};

use crate::nes::Nes;

mod bin_ops;
mod cpu;
mod mapper;
mod nes;

const MISSING_NES_ROM_FILE_PATH_ERROR_CODE: i32 = -1;
const NES_ROM_BIN_READ_IO_ERROR_CODE: i32 = -2;
const NES_VIRTUAL_PLATFORM_ERROR_CODE: i32 = -3;

const MAX_ROM_FILE_SIZE_IN_BYTES: u64 = 0x500000;

fn main() {
    let rom_bin_path = args()
        .nth(1)
        .map(|path_str| PathBuf::from(path_str))
        .unwrap_or_else(|| {
            print_help();
            std::process::exit(MISSING_NES_ROM_FILE_PATH_ERROR_CODE);
        });

    let rom_bin = match read_bin(rom_bin_path) {
        Ok(bin) => bin,
        Err(err) => {
            eprintln!("An error occurred while reading the NES ROM file. {}", err);
            std::process::exit(NES_ROM_BIN_READ_IO_ERROR_CODE);
        }
    };

    let mut nes = match Nes::new(rom_bin) {
        Ok(n) => n,
        Err(err) => {
            eprintln!(
                "An error occurred while creating the NES virtual environment. {}",
                err
            );
            std::process::exit(NES_VIRTUAL_PLATFORM_ERROR_CODE);
        }
    };

    nes.run();
}

fn read_bin<P>(file_path: P) -> Result<Vec<u8>, Error>
where
    P: AsRef<Path>,
{
    let mut file = File::open(file_path)?;
    let file_metadata = file.metadata()?;
    let file_size = file_metadata.len();

    if file_size > MAX_ROM_FILE_SIZE_IN_BYTES {
        let max_file_size_in_mb = MAX_ROM_FILE_SIZE_IN_BYTES / 1024 / 1024;
        let error_msg = format!(
            "ROM file size cannot be more than {} MB.",
            max_file_size_in_mb
        );

        return Err(Error::new(ErrorKind::Other, error_msg));
    }

    let mut rom_bin = Vec::with_capacity(file_size as usize);

    file.read_to_end(&mut rom_bin)?;

    Ok(rom_bin)
}

fn print_help() {
    //TODO: Somehow get the version from the TOML file
    println!("pyrotobox v0.1.0");
    println!("USAGE: pyrotobox <NES_ROM_FILE_PATH>");
}
