/// Patch `Header.stamp` in CDR-serialized data by adding `offset_ns`.
///
/// CDR layout when `std_msgs/msg/Header` is the first field:
///   bytes 0-3: encapsulation header (byte 1: 0x00=BE, 0x01=LE)
///   bytes 4-7: stamp.sec (int32)
///   bytes 8-11: stamp.nanosec (uint32)
///
/// Returns `false` if data is too short (<12 bytes).
pub fn patch_header_stamp(data: &mut [u8], offset_ns: i64) -> bool {
    if data.len() < 12 {
        return false;
    }

    let little_endian = data[1] == 0x01;

    let sec: i64;
    let nanosec: i64;

    if little_endian {
        sec = i32::from_le_bytes([data[4], data[5], data[6], data[7]]) as i64;
        nanosec = u32::from_le_bytes([data[8], data[9], data[10], data[11]]) as i64;
    } else {
        sec = i32::from_be_bytes([data[4], data[5], data[6], data[7]]) as i64;
        nanosec = u32::from_be_bytes([data[8], data[9], data[10], data[11]]) as i64;
    }

    let total_ns = sec * 1_000_000_000 + nanosec + offset_ns;

    let new_sec = total_ns.div_euclid(1_000_000_000) as i32;
    let new_nanosec = total_ns.rem_euclid(1_000_000_000) as u32;

    if little_endian {
        data[4..8].copy_from_slice(&new_sec.to_le_bytes());
        data[8..12].copy_from_slice(&new_nanosec.to_le_bytes());
    } else {
        data[4..8].copy_from_slice(&new_sec.to_be_bytes());
        data[8..12].copy_from_slice(&new_nanosec.to_be_bytes());
    }

    true
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: build a minimal CDR buffer with LE encapsulation.
    fn make_cdr_le(sec: i32, nanosec: u32) -> Vec<u8> {
        let mut buf = vec![0u8; 12];
        buf[0] = 0x00; // encapsulation byte 0
        buf[1] = 0x01; // LE
        buf[2] = 0x00;
        buf[3] = 0x00;
        buf[4..8].copy_from_slice(&sec.to_le_bytes());
        buf[8..12].copy_from_slice(&nanosec.to_le_bytes());
        buf
    }

    /// Helper: build a minimal CDR buffer with BE encapsulation.
    fn make_cdr_be(sec: i32, nanosec: u32) -> Vec<u8> {
        let mut buf = vec![0u8; 12];
        buf[0] = 0x00;
        buf[1] = 0x00; // BE
        buf[2] = 0x00;
        buf[3] = 0x00;
        buf[4..8].copy_from_slice(&sec.to_be_bytes());
        buf[8..12].copy_from_slice(&nanosec.to_be_bytes());
        buf
    }

    fn read_stamp_le(data: &[u8]) -> (i32, u32) {
        let sec = i32::from_le_bytes([data[4], data[5], data[6], data[7]]);
        let nsec = u32::from_le_bytes([data[8], data[9], data[10], data[11]]);
        (sec, nsec)
    }

    fn read_stamp_be(data: &[u8]) -> (i32, u32) {
        let sec = i32::from_be_bytes([data[4], data[5], data[6], data[7]]);
        let nsec = u32::from_be_bytes([data[8], data[9], data[10], data[11]]);
        (sec, nsec)
    }

    #[test]
    fn test_positive_offset() {
        let mut buf = make_cdr_le(100, 500_000_000);
        assert!(patch_header_stamp(&mut buf, 2_000_000_000)); // +2s
        let (sec, nsec) = read_stamp_le(&buf);
        assert_eq!(sec, 102);
        assert_eq!(nsec, 500_000_000);
    }

    #[test]
    fn test_negative_offset() {
        let mut buf = make_cdr_le(100, 500_000_000);
        assert!(patch_header_stamp(&mut buf, -1_500_000_000)); // -1.5s
        let (sec, nsec) = read_stamp_le(&buf);
        assert_eq!(sec, 99);
        assert_eq!(nsec, 0);
    }

    #[test]
    fn test_nanosec_overflow() {
        let mut buf = make_cdr_le(10, 800_000_000);
        assert!(patch_header_stamp(&mut buf, 500_000_000)); // +0.5s -> should carry
        let (sec, nsec) = read_stamp_le(&buf);
        assert_eq!(sec, 11);
        assert_eq!(nsec, 300_000_000);
    }

    #[test]
    fn test_short_buffer() {
        let mut buf = vec![0u8; 8];
        assert!(!patch_header_stamp(&mut buf, 1_000_000_000));
    }

    #[test]
    fn test_big_endian() {
        let mut buf = make_cdr_be(50, 200_000_000);
        assert!(patch_header_stamp(&mut buf, 3_000_000_000)); // +3s
        let (sec, nsec) = read_stamp_be(&buf);
        assert_eq!(sec, 53);
        assert_eq!(nsec, 200_000_000);
    }

    #[test]
    fn test_zero_offset() {
        let mut buf = make_cdr_le(42, 123_456_789);
        assert!(patch_header_stamp(&mut buf, 0));
        let (sec, nsec) = read_stamp_le(&buf);
        assert_eq!(sec, 42);
        assert_eq!(nsec, 123_456_789);
    }

    #[test]
    fn test_large_offset_multiple_iterations() {
        // Simulate 3 iterations of a 10-second bag.
        let mut buf = make_cdr_le(5, 0);
        let bag_duration_ns = 10_000_000_000i64;
        let offset = 3 * bag_duration_ns;
        assert!(patch_header_stamp(&mut buf, offset));
        let (sec, nsec) = read_stamp_le(&buf);
        assert_eq!(sec, 35);
        assert_eq!(nsec, 0);
    }
}
