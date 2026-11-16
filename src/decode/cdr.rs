//! A reader for ROS 2's wire format: OMG CDR (XCDR1) with its 4-byte encapsulation header, as
//! `foxglove_bridge` forwards it. Primitives align to their own size, counted from the end of
//! the header; strings are a u32 length (including the NUL) then bytes; sequences a u32 count.

use super::DecodeError;

pub struct Cdr<'a> {
    body: &'a [u8],
    at: usize,
    little: bool,
}

impl<'a> Cdr<'a> {
    pub fn new(bytes: &'a [u8]) -> Result<Cdr<'a>, DecodeError> {
        if bytes.len() < 4 {
            return Err(DecodeError::new("CDR payload shorter than its header"));
        }
        // Byte 1 of the representation id: 0/2 big-endian, 1/3 little-endian (CDR / PL_CDR).
        let little = match (bytes[0], bytes[1]) {
            (0, 0) | (0, 2) => false,
            (0, 1) | (0, 3) => true,
            (a, b) => return Err(DecodeError::new(format!("unsupported CDR representation {a:#04x}{b:02x}"))),
        };
        Ok(Cdr { body: &bytes[4..], at: 0, little })
    }

    fn take(&mut self, n: usize, align: usize) -> Result<&'a [u8], DecodeError> {
        let start = self.at.next_multiple_of(align);
        let end = start.checked_add(n).filter(|&e| e <= self.body.len())
            .ok_or_else(|| DecodeError::new(format!("CDR payload ends early: need {n} bytes at {start}, have {}", self.body.len())))?;
        self.at = end;
        Ok(&self.body[start..end])
    }

    pub fn u8(&mut self) -> Result<u8, DecodeError> {
        Ok(self.take(1, 1)?[0])
    }

    pub fn bool(&mut self) -> Result<bool, DecodeError> {
        Ok(self.u8()? != 0)
    }

    pub fn u32(&mut self) -> Result<u32, DecodeError> {
        let b: [u8; 4] = self.take(4, 4)?.try_into().unwrap();
        Ok(if self.little { u32::from_le_bytes(b) } else { u32::from_be_bytes(b) })
    }

    pub fn i32(&mut self) -> Result<i32, DecodeError> {
        Ok(self.u32()? as i32)
    }

    pub fn f64(&mut self) -> Result<f64, DecodeError> {
        let b: [u8; 8] = self.take(8, 8)?.try_into().unwrap();
        Ok(if self.little { f64::from_le_bytes(b) } else { f64::from_be_bytes(b) })
    }

    pub fn string(&mut self) -> Result<String, DecodeError> {
        let n = self.u32()? as usize;
        let raw = self.take(n, 1)?;
        let raw = raw.strip_suffix(&[0]).unwrap_or(raw);
        String::from_utf8(raw.to_vec()).map_err(|_| DecodeError::new("CDR string is not UTF-8"))
    }

    /// A `uint8[]`, borrowed.
    pub fn bytes(&mut self) -> Result<&'a [u8], DecodeError> {
        let n = self.u32()? as usize;
        self.take(n, 1)
    }

    /// A sequence length, checked against what's left so a corrupt count can't ask for an
    /// absurd allocation: every element takes at least `min_elem` bytes.
    pub fn seq_len(&mut self, min_elem: usize) -> Result<usize, DecodeError> {
        let n = self.u32()? as usize;
        if n.saturating_mul(min_elem) > self.body.len() - self.at {
            return Err(DecodeError::new(format!("CDR sequence of {n} elements is longer than the payload")));
        }
        Ok(n)
    }
}
