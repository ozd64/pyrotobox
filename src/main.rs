mod file_io;

use std::env::args;
use std::path::PathBuf;

use file_io::get_rom_file_content;

const ROM_FILE_NOT_PROVIDED_ERROR_CODE: i32 = -1;

fn main() {
    let rom_file_path = args()
        .into_iter()
        .nth(1)
        .map(|file_path| PathBuf::from(file_path))
        .unwrap_or_else(|| {
            print_help();
            std::process::exit(ROM_FILE_NOT_PROVIDED_ERROR_CODE);
        });

    let rom_bytes = get_rom_file_content(rom_file_path).unwrap();

    println!("First Byte: {:X}", &rom_bytes[0]);
}

fn print_help() {
    println!("USAGE: pyrotobox <NES_ROM_FILE_PATH>");
}
