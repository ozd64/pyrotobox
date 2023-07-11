pub fn read_le_u32(bin_arr: &[u8]) -> u32 {
    assert!(bin_arr.len() >= 4);

    let msb1 = (bin_arr[3] as u32) << 24;
    let msb2 = (bin_arr[2] as u32) << 16;
    let msb3 = (bin_arr[1] as u32) << 8;
    let msb4 = bin_arr[0] as u32;

    msb1 | msb2 | msb3 | msb4
}
