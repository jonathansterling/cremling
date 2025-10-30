use crate::gameboy::cpu::registers::{RegisterPair, Registers};
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
        println!("PC: 0x{:04X}", self.pc);
        let instruction = self.fetch_instruction(memory);
        // println!("Fetched instruction: 0x{:02X}", instruction);

        // Decode and execute the instruction
        // dbg!(&self);
        match instruction {

            0x01 => self.ld_nn(memory, self.get_register_pair_from_opcode(instruction)),
            0x11 => self.ld_nn(memory, self.get_register_pair_from_opcode(instruction)),
            0x21 => self.ld_nn(memory, self.get_register_pair_from_opcode(instruction)),
            0x31 => self.ld_nn(memory, self.get_register_pair_from_opcode(instruction)),

            // Single register ALU operations
            // 0x80..=0xBF => self.alu_r(opcode), // TODO: Decode the opcode to determine the operation and register
            0xA8..=0xAF => self.xor(self.get_register_from_opcode(instruction)),

            _ => panic!("Unknown opcode: 0x{:02X}", instruction),
        }
        // dbg!(&self);
    }

    fn fetch_instruction(&mut self, memory: &crate::gameboy::memory::Memory) -> u8 {
        let instruction = memory.read_byte(self.pc);
        self.pc = self.pc.wrapping_add(1);
        instruction
    }

    // -------------------------------
    // --- Opcode Decoding Helpers ---
    // -------------------------------

    // Gets the register value
    fn get_register_from_opcode(&self, opcode: u8) -> u8 {
        match opcode & 0x07 {
            0x00 => self.registers.b,
            0x01 => self.registers.c,
            0x02 => self.registers.d,
            0x03 => self.registers.e,
            0x04 => self.registers.h,
            0x05 => self.registers.l,
            0x07 => self.registers.a,
            _ => panic!("Invalid register code in opcode: 0x{:02X}", opcode),
        }
    }

    // Gets the pair enum
    fn get_register_pair_from_opcode(&self, opcode: u8) -> RegisterPair {
        match (opcode >> 4) & 0x03 {
            0x00 => RegisterPair::BC,
            0x01 => RegisterPair::DE,
            0x02 => RegisterPair::HL,
            0x03 => RegisterPair::SP,
            _ => panic!("Invalid register pair code in opcode: 0x{:02X}", opcode),
        }
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

    // -----------------------------------
    // --- Instruction Implementations ---
    // -----------------------------------

    fn ld_nn(&mut self, memory: &crate::gameboy::memory::Memory, register_pair: RegisterPair) {
        // LD rr, nn - Load 16-bit immediate value into register pair
        // println!("Executing LD rr, nn");

        // Fetch the next two bytes for the immediate value
        let value = self.fetch_two_bytes(memory);

        // Load the value into the specified register pair
        match register_pair {
            RegisterPair::BC => self.registers.set_bc(value),
            RegisterPair::DE => self.registers.set_de(value),
            RegisterPair::HL => self.registers.set_hl(value),
            RegisterPair::SP => self.sp = value,
        }
        println!("Loaded 0x{:04X} into {:?}", value, register_pair);
    }

    fn xor(&mut self, value: u8) {
        // XOR A - Logical XOR A with itself
        // println!("Executing XOR A");

        // Perform XOR operation
        let result = self.registers.a ^ value;
        self.registers.a = result;

        // Set flags: Z=1, N=0, H=0, C=0
        self.flags.set(result == 0, false, false, false);
        println!("Result of XOR A: 0x{:02X}", result);
    }


}

