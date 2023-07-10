use std::env::args;
use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};

const MISSING_NES_ROM_FILE_PATH_ERROR_CODE: i32 = -1;
const NES_ROM_BIN_READ_IO_ERROR_CODE: i32 = -2;

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

    println!("First character of the rom file: 0x{:X}", &rom_bin[0]);
}

fn read_bin<P>(file_path: P) -> Result<Vec<u8>, std::io::Error>
where
    P: AsRef<Path>,
{
    let mut file = File::open(file_path)?;
    let file_metadata = file.metadata()?;
    let mut rom_bin = Vec::with_capacity(file_metadata.len() as usize);

    file.read_to_end(&mut rom_bin)?;

    Ok(rom_bin)
}

fn print_help() {
    //TODO: Somehow get the version from the TOML file
    println!("pyrotobox v0.1.0");
    println!("USAGE: pyrotobox <NES_ROM_FILE_PATH>");
}
