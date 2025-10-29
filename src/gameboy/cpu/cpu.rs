use crate::gameboy::cpu::registers::Registers;
use crate::gameboy::cpu::flags::Flags;

#[derive(Debug)]
#[derive(Default)]
pub struct CPU {
    pc: u16,         // Program Counter
    sp: u16,         // Stack Pointer
    interrupt: u8,   // Interrupt Enable
    registers: Registers,
    flags: Flags,
}

impl CPU {
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
        // let result = (self.af & 0xFF) ^ (self.af & 0xFF);
        // self.af = (self.af & 0xFF00) | result;
        let result = self.registers.a ^ self.registers.a;
        self.registers.a = result;

        // Set flags: Z=1, N=0, H=0, C=0
        self.flags.zero = result == 0;
        // self.set_zero_flag(result == 0);
    }
}

