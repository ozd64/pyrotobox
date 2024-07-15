use std::env::args;
use std::fmt::Display;
use std::fs::File;
use std::io::{BufReader, Error, Read};
use std::path::{Path, PathBuf};

const ROM_FILE_NOT_PROVIDED_ERROR_CODE: i32 = -1;

const MAX_NES_ROM_FILE_SIZE: u64 = 0x500000;
const MIN_NES_ROM_FILE_SIZE: u64 = 0x10;

#[derive(Debug)]
enum RomFileError {
    RomFileTooBig(u64),
    RomFileTooSmall(u64),
    IOError(Error),
}

impl Display for RomFileError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RomFileError::RomFileTooBig(size) => {
                write!(f, "File is too big for a NES ROM. File Size: {}", size)
            }
            RomFileError::RomFileTooSmall(size) => {
                write!(f, "File is too small for a NES ROM. File Size: {}", size)
            }
            RomFileError::IOError(err) => write!(
                f,
                "An IO error occurred while processing the ROM file.\n{}",
                err
            ),
        }
    }
}

impl std::error::Error for RomFileError {}

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

fn get_rom_file_content<P>(path: P) -> Result<Vec<u8>, RomFileError>
where
    P: AsRef<Path>,
{
    let file = File::open(path).map_err(|err| RomFileError::IOError(err))?;
    let file_size = file
        .metadata()
        .map_err(|err| RomFileError::IOError(err))?
        .len();

    if file_size < MIN_NES_ROM_FILE_SIZE {
        return Err(RomFileError::RomFileTooSmall(file_size));
    } else if file_size > MAX_NES_ROM_FILE_SIZE {
        return Err(RomFileError::RomFileTooBig(file_size));
    }

    let mut buf_reader = BufReader::new(file);

    let mut rom_bytes = Vec::<u8>::with_capacity(file_size as usize);

    buf_reader
        .read_to_end(&mut rom_bytes)
        .map_err(|err| RomFileError::IOError(err))?;

    Ok(rom_bytes)
}

fn print_help() {
    println!("USAGE: pyrotobox <NES_ROM_FILE_PATH>");
}
