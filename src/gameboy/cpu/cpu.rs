#[derive(Debug)]
#[derive(Default)]
pub struct CPU {
    pc: u16,         // Program Counter
    sp: u16,         // Stack Pointer
    af: u16,         // Accumulator & Flags
    bc: u16,         // Registers B & C
    de: u16,         // Registers D & E
    hl: u16,         // Registers H & L
    instruction: u8, // Instruction Register
    interrupt: u8,   // Interrupt Enable
}

impl CPU {
    pub fn skip_boot_rom() -> Self {
        CPU {
            pc: 0x0100,
            sp: 0xFFFE,
            af: 0x01B0,
            bc: 0x13,
            de: 0xD8,
            hl: 0x4D,
            instruction: 0x00,
            interrupt: 0x00,
        }
    }

    pub fn fetch_decode_execute(&mut self, memory: &crate::gameboy::memory::Memory) {
        // Fetch the next instruction
        let instruction = self.fetch_instruction(memory);
        // println!("Fetched instruction: 0x{:02X}", instruction);

        // Decode and execute the instruction
        dbg!(&self);
        match instruction {
            0x31 => self.ld_sp_nn(memory),
            0xAF => self.xor_a(),
            _ => panic!("Unknown opcode: 0x{:02X}", instruction),
        }
        dbg!(&self);
    }

    fn fetch_instruction(&mut self, memory: &crate::gameboy::memory::Memory) -> u8 {
        let instruction = memory.read_byte(self.pc);
        self.pc = self.pc.wrapping_add(1);
        instruction
    }

    // ------------------------------------
    // --- Sixteen Bit Fetching Helpers ---
    // ------------------------------------

    fn fetch_two_bytes(&mut self, memory: &crate::gameboy::memory::Memory) -> u16 {
        let low_byte = memory.read_byte(self.pc) as u16;
        self.pc = self.pc.wrapping_add(1);
        let high_byte = memory.read_byte(self.pc) as u16;
        self.pc = self.pc.wrapping_add(1);
        (high_byte << 8) | low_byte
    }

    // ------------------------------------
    // --- Eight Bit Register Accessors ---  
    // ------------------------------------

    fn a(&self) -> u8 {
        (self.af & 0xFF) as u8
    }

    fn f(&self) -> u8 {
        (self.af >> 8) as u8
    }

    fn b(&self) -> u8 {
        (self.bc >> 8) as u8
    }

    fn c(&self) -> u8 {
        (self.bc & 0xFF) as u8
    }

    fn d(&self) -> u8 {
        (self.de >> 8) as u8
    }

    fn e(&self) -> u8 {
        (self.de & 0xFF) as u8
    }

    fn h(&self) -> u8 {
        (self.hl >> 8) as u8
    }

    fn l(&self) -> u8 {
        (self.hl & 0xFF) as u8
    }

    // ---------------------------------
    // --- Flag Manipulation Helpers ---
    // ---------------------------------

    fn set_flags(&mut self, z: bool, n: bool, h: bool, c: bool) {
        self.set_zero_flag(z);
        self.set_subtract_flag(n);
        self.set_half_carry_flag(h);
        self.set_carry_flag(c);
    }

    fn set_flag(&mut self, flag_mask: u8, value: bool) {
        if value {
            self.af |= flag_mask as u16;
        } else {
            self.af &= !(flag_mask as u16);
        }
    }

    // The zero flag is set if the result of an operation is zero.
    //     Example: After an addition, if the result is 0, set Z flag to 1.
    fn set_zero_flag(&mut self, value: bool) {
        self.set_flag(0x80, value);
    }

    // The subtract flag is set if the last operation was a subtraction.
    //     Example: After a subtraction operation, set N flag to 1.
    fn set_subtract_flag(&mut self, value: bool) {
        set_flag(&mut self.af, 0x40, value);
    }

    // The half-carry flag is set if there was a carry from bit 3 to bit 4 in the last operation.
    //     Example
    fn set_half_carry_flag(&mut self, value: bool) {
        self.set_flag(0x20, value);
    }

    // The carry flag is set if there was a carry from bit 7 in the last operation.
    fn set_carry_flag(&mut self, value: bool) {
        if value {
            self.af |= 0x10;
        } else {
            self.af &= !0x10;
        }
    }

    // -----------------------------------
    // --- Instruction Implementations ---
    // -----------------------------------

    fn ld_sp_nn(&mut self, memory: &crate::gameboy::memory::Memory) {
        // LD SP, nn - Load 16-bit immediate value into SP
        // println!("Executing LD SP, nn");

        // Fetch the next two bytes for the immediate value
        let low_byte = self.fetch_instruction(memory) as u16;
        let high_byte = self.fetch_instruction(memory) as u16;
        let nn = (high_byte << 8) | low_byte;
        self.sp = nn;
        println!("Loaded 0x{:04X} into SP", nn);
    }

    fn xor_a(&mut self) {
        // XOR A - Logical XOR A with itself
        // println!("Executing XOR A");

        // Perform XOR operation
        let result = (self.af & 0xFF) ^ (self.af & 0xFF);
        self.af = (self.af & 0xFF00) | result;

        // Set flags: Z=1, N=0, H=0, C=0
        self.af = (self.af & 0xFF00) | (if result == 0 { 0x80 } else { 0x00 });
    }
}

// impl CPU {
//     pub fn new() -> CPU {
//         CPU {}
//     }
// }


