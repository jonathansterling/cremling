const ZERO_FLAG_BYTE_POSITION: u8 = 7;
const SUBTRACT_FLAG_BYTE_POSITION: u8 = 6;
const HALF_CARRY_FLAG_BYTE_POSITION: u8 = 5;
const CARRY_FLAG_BYTE_POSITION: u8 = 4;

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
pub struct Flags {
    pub zero: bool,
    pub subtract: bool,
    pub half_carry: bool,
    pub carry: bool
}

impl Flags {
    pub fn set(&mut self, zero: bool, subtract: bool, half_carry: bool, carry: bool) {
        self.zero = zero;
        self.subtract = subtract;
        self.half_carry = half_carry;
        self.carry = carry;
    }

    pub fn condition(&self, condition: Conditions) -> bool {
        match condition {
            Conditions::Z  => self.zero,
            Conditions::NZ => !self.zero,
            Conditions::C  => self.carry,
            Conditions::NC => !self.carry,
            Conditions::NONE => true,
        }
    }
}

impl std::convert::From<Flags> for u8  {
    fn from(flag: Flags) -> u8 {
        (if flag.zero       { 1 } else { 0 }) << ZERO_FLAG_BYTE_POSITION |
        (if flag.subtract   { 1 } else { 0 }) << SUBTRACT_FLAG_BYTE_POSITION |
        (if flag.half_carry { 1 } else { 0 }) << HALF_CARRY_FLAG_BYTE_POSITION |
        (if flag.carry      { 1 } else { 0 }) << CARRY_FLAG_BYTE_POSITION
    }
}

impl std::convert::From<u8> for Flags {
    fn from(byte: u8) -> Self {
        let zero = ((byte >> ZERO_FLAG_BYTE_POSITION) & 0b1) != 0;
        let subtract = ((byte >> SUBTRACT_FLAG_BYTE_POSITION) & 0b1) != 0;
        let half_carry = ((byte >> HALF_CARRY_FLAG_BYTE_POSITION) & 0b1) != 0;
        let carry = ((byte >> CARRY_FLAG_BYTE_POSITION) & 0b1) != 0;

        Flags {
            zero,
            subtract,
            half_carry,
            carry
        }
    }
}

