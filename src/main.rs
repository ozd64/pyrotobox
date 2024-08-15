use std::env::args;
use std::path::PathBuf;

use pyrotobox::nes::Nes;

const ROM_FILE_NOT_PROVIDED_ERROR_CODE: i32 = -1;
const NES_EMULATOR_BUILD_FAILED_ERROR_CODE: i32 = -2;

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

    let mut nes = match Nes::new(rom_file_path) {
        Ok(nes_instance) => nes_instance,
        Err(err) => {
            eprintln!(
                "An error occurred while building the NES emulator.\n{}",
                err
            );
            std::process::exit(NES_EMULATOR_BUILD_FAILED_ERROR_CODE);
        }
    };

    nes.start();
}

fn print_help() {
    println!("USAGE: pyrotobox <NES_ROM_FILE_PATH>");
}
