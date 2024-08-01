use crate::mapper::CpuMemMap;

pub struct Cpu {
    mem_map: CpuMemMap,
}

impl Cpu {
    pub fn new(cpu_mem_map: CpuMemMap) -> Self {
        Self {
            mem_map: cpu_mem_map,
        }
    }
}
