const LOW_BYTE_MASK: u16 = 0x00FF;
const HIGH_BYTE_MASK: u16 = 0xFF00;

#[derive(Debug)]
pub enum RegisterPair { BC, DE, HL, SP }

#[derive(Debug)]
#[derive(Default)]
pub struct Registers {
    pub a: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
    pub e: u8,
    pub h: u8,
    pub l: u8,
}

impl Registers {
    //--------------------------------------
    //--- Sixteen Bit Register Accessors ---
    //--------------------------------------

    pub fn get_bc(&self) -> u16 {
      (self.b as u16) << 8 | self.c as u16
    }

    pub fn get_de(&self) -> u16 {
        (self.d as u16) << 8 | self.e as u16
    }

    pub fn get_hl(&self) -> u16 {
        (self.h as u16) << 8 | self.l as u16
    }

    pub fn set_bc(&mut self, value: u16) {
        self.b = ((value & LOW_BYTE_MASK) >> 8) as u8;
        self.c = (value & HIGH_BYTE_MASK) as u8;
    }

    pub fn set_de(&mut self, value: u16) {
        self.d = ((value & LOW_BYTE_MASK) >> 8) as u8;
        self.e = (value & HIGH_BYTE_MASK) as u8;
    }

    pub fn set_hl(&mut self, value: u16) {
        self.h = ((value & LOW_BYTE_MASK) >> 8) as u8;
        self.l = (value & HIGH_BYTE_MASK) as u8;
    }
}


