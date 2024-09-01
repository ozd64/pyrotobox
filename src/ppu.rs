use crate::mapper::PpuMemMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ColorIndex {
    One = 1,
    Two = 2,
    Three = 3,
    Four = 4,
}

type Tile = [[ColorIndex; 8]; 8];

pub struct Ppu {
    mem_map: PpuMemMap,
    tiles: Vec<Tile>,
}

impl Ppu {
    pub fn new(mem_map: PpuMemMap) -> Self {
        let mut tiles = Vec::with_capacity(512);

        for i in 0..512 {
            let tile_start_index = i * 16;
            let tile_end_index = tile_start_index + 16;

            let tile_bytes = &mem_map[tile_start_index..tile_end_index];
            let tile = Ppu::generate_tile(tile_bytes);

            tiles.push(tile);
        }

        Self { mem_map, tiles }
    }

    fn generate_tile(tile_bytes: &[u8]) -> Tile {
        assert_eq!(tile_bytes.len(), 16);

        let mut tile = [[ColorIndex::One; 8]; 8];

        let first_section = &tile_bytes[..8];
        let second_section = &tile_bytes[8..];

        for i in 0..8 {
            let num1 = first_section[i];
            let num2 = second_section[i];

            for j in 0..8 {
                let bit1 = (num1 >> j) & 1;
                let bit2 = (num2 >> j) & 1;

                let result = match bit2 << 1 | bit1 {
                    0b00 => ColorIndex::One,
                    0b01 => ColorIndex::Two,
                    0b10 => ColorIndex::Three,
                    0b11 => ColorIndex::Four,
                    _ => panic!("Bit manipulation within generation of tiles yield a result not between 0-4!")
                };

                tile[i][j] = result;
            }
        }

        tile
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generating_tile() {
        let mut tile_bytes = vec![0x00; 16];

        tile_bytes[0x00] = 0x80;
        tile_bytes[0x08] = 0x80;

        let tile = Ppu::generate_tile(&tile_bytes);

        let mut expected_result = [[ColorIndex::One; 8]; 8];
        expected_result[0][7] = ColorIndex::Four;

        assert_eq!(tile, expected_result);
    }
}
