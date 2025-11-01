const LOW_BYTE_MASK: u16 = 0x00FF;
const HIGH_BYTE_MASK: u16 = 0xFF00;

const ZERO_FLAG_BYTE_POSITION: u8 = 7;
const SUBTRACT_FLAG_BYTE_POSITION: u8 = 6;
const HALF_CARRY_FLAG_BYTE_POSITION: u8 = 5;
const CARRY_FLAG_BYTE_POSITION: u8 = 4;

#[derive(Debug, Copy, Clone)]
pub enum R16 { AF, BC, DE, HL, SP }

#[derive(Debug, Copy, Clone)]
pub enum R8 { A, B, C, D, E, H, L, HL }

#[derive(Debug, Copy, Clone)]
pub enum Conditions {
    Z,  // Zero Flag set
    NZ, // Zero Flag not set
    C,  // Carry Flag set
    NC, // Carry Flag not set
    NONE,
}

#[derive(Debug)]
#[derive(Default)]
pub struct Registers {
    pub a: u8,
    pub f: u8,
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

    pub fn get_af(&self) -> u16 {
        (self.a as u16) << 8 | self.f as u16
    }

    pub fn get_bc(&self) -> u16 {
      (self.b as u16) << 8 | self.c as u16
    }

    pub fn get_de(&self) -> u16 {
        (self.d as u16) << 8 | self.e as u16
    }

    pub fn get_hl(&self) -> u16 {
        (self.h as u16) << 8 | self.l as u16
    }

    pub fn set_af(&mut self, value: u16) {
        self.a = ((value & HIGH_BYTE_MASK) >> 8) as u8;
        self.f = (value & LOW_BYTE_MASK) as u8 & 0xF0;
    }

    pub fn set_bc(&mut self, value: u16) {
        self.b = ((value & HIGH_BYTE_MASK) >> 8) as u8;
        self.c = (value & LOW_BYTE_MASK) as u8;
    }

    pub fn set_de(&mut self, value: u16) {
        self.d = ((value & HIGH_BYTE_MASK) >> 8) as u8;
        self.e = (value & LOW_BYTE_MASK) as u8;
    }

    pub fn set_hl(&mut self, value: u16) {
        self.h = ((value & HIGH_BYTE_MASK) >> 8) as u8;
        self.l = (value & LOW_BYTE_MASK) as u8;
    }

    //------------------------------
    //--- Register Read/Write ------
    //------------------------------

    pub fn read_r8(&self, register: R8, memory: &crate::gameboy::memory::Memory) -> u8 {
        match register {
            R8::A => self.a,
            R8::B => self.b,
            R8::C => self.c,
            R8::D => self.d,
            R8::E => self.e,
            R8::H => self.h,
            R8::L => self.l,
            R8::HL => memory.read_byte(self.get_hl()),
        }
    }

    pub fn write_r8(&mut self, register: R8, value: u8, memory: &mut crate::gameboy::memory::Memory) {
        match register {
            R8::A => self.a = value,
            R8::B => self.b = value,
            R8::C => self.c = value,
            R8::D => self.d = value,
            R8::E => self.e = value,
            R8::H => self.h = value,
            R8::L => self.l = value,
            R8::HL => memory.write_byte(self.get_hl(), value),
        }
    }

    pub fn write_r16(&mut self, register: R16, value: u16) {
        match register {
            R16::AF => self.set_af(value),
            R16::BC => self.set_bc(value),
            R16::DE => self.set_de(value),
            R16::HL => self.set_hl(value),
            R16::SP => panic!("SP register not implemented in Registers struct"),
        }
    }

    pub fn read_r16(&self, register: R16) -> u16 {
        match register {
            R16::AF => self.get_af(),
            R16::BC => self.get_bc(),
            R16::DE => self.get_de(),
            R16::HL => self.get_hl(),
            R16::SP => panic!("SP register not implemented in Registers struct"),
        }
    }

    //------------------------------
    //--- Flag Accessors -----------
    //------------------------------

    pub fn get_zero_flag(&self) -> bool {
        ((self.f >> ZERO_FLAG_BYTE_POSITION) & 0b1) != 0
    }

    pub fn get_subtract_flag(&self) -> bool {
        ((self.f >> SUBTRACT_FLAG_BYTE_POSITION) & 0b1) != 0
    }

    pub fn get_half_carry_flag(&self) -> bool {
        ((self.f >> HALF_CARRY_FLAG_BYTE_POSITION) & 0b1) != 0
    }

    pub fn get_carry_flag(&self) -> bool {
        ((self.f >> CARRY_FLAG_BYTE_POSITION) & 0b1) != 0
    }

    pub fn set_zero_flag(&mut self, value: bool) {
        if value {
            self.f |= 1 << ZERO_FLAG_BYTE_POSITION;
        } else {
            self.f &= !(1 << ZERO_FLAG_BYTE_POSITION);
        }
    }

    pub fn set_subtract_flag(&mut self, value: bool) {
        if value {
            self.f |= 1 << SUBTRACT_FLAG_BYTE_POSITION;
        } else {
            self.f &= !(1 << SUBTRACT_FLAG_BYTE_POSITION);
        }
    }

    pub fn set_half_carry_flag(&mut self, value: bool) {
        if value {
            self.f |= 1 << HALF_CARRY_FLAG_BYTE_POSITION;
        } else {
            self.f &= !(1 << HALF_CARRY_FLAG_BYTE_POSITION);
        }
    }

    pub fn set_carry_flag(&mut self, value: bool) {
        if value {
            self.f |= 1 << CARRY_FLAG_BYTE_POSITION;
        } else {
            self.f &= !(1 << CARRY_FLAG_BYTE_POSITION);
        }
    }

    pub fn set_flags(&mut self, zero: bool, subtract: bool, half_carry: bool, carry: bool) {
        self.f = (if zero       { 1 } else { 0 }) << ZERO_FLAG_BYTE_POSITION |
                 (if subtract   { 1 } else { 0 }) << SUBTRACT_FLAG_BYTE_POSITION |
                 (if half_carry { 1 } else { 0 }) << HALF_CARRY_FLAG_BYTE_POSITION |
                 (if carry      { 1 } else { 0 }) << CARRY_FLAG_BYTE_POSITION;
    }

    pub fn check_condition(&self, condition: Conditions) -> bool {
        match condition {
            Conditions::Z  => self.get_zero_flag(),
            Conditions::NZ => !self.get_zero_flag(),
            Conditions::C  => self.get_carry_flag(),
            Conditions::NC => !self.get_carry_flag(),
            Conditions::NONE => true,
        }
    }
}


