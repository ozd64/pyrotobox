use std::env::args;
use std::path::PathBuf;

use pyrotobox::rom::Rom;

const ROM_FILE_NOT_PROVIDED_ERROR_CODE: i32 = -1;
const UNABLE_PARSE_NES_ROM_FILE: i32 = -2;

fn main() {
    let rom_file_path = args()
        .into_iter()
        .nth(1)
        .map(|file_path| PathBuf::from(file_path))
        .unwrap_or_else(|| {
            print_help();
            std::process::exit(ROM_FILE_NOT_PROVIDED_ERROR_CODE);
        });

    println!("----- pyrotobox v0.1.0 BETA -----");
    println!("ROM File Path: {:?}", rom_file_path);

    let rom = match Rom::from_ines(rom_file_path) {
        Ok(nes_rom) => nes_rom,
        Err(err) => {
            eprintln!("An error occurred while parsing the NES ROM file.\n{}", err);
            std::process::exit(UNABLE_PARSE_NES_ROM_FILE)
        }
    };

    println!("PRG ROM Size: {}", rom.prg_rom_size());
    println!("CHR ROM Size: {}", rom.chr_rom_size());
    println!("Mapper: {}", rom.mapper());
    println!("Mirroring: {}", rom.mirroring());
}

fn print_help() {
    println!("USAGE: pyrotobox <NES_ROM_FILE_PATH>");
}
