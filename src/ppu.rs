use crate::mapper::PpuMemMap;

pub struct Ppu {
    mem_map: PpuMemMap,
}

impl Ppu {
    pub fn new(mem_map: PpuMemMap) -> Self {
        Self { mem_map }
    }
}

#[cfg(test)]
mod tests {

    #[test]
    fn test() {}
}
