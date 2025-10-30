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
    pub fn fetch_decode_execute(&mut self, memory: &mut crate::gameboy::memory::Memory) {
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

            // 0x32 => self.ld_hl_indirect_dec(memory),
            0x02 => self.ld_indirect(memory, RegisterPair::BC, 0),
            0x12 => self.ld_indirect(memory, RegisterPair::DE, 0),
            0x22 => self.ld_indirect(memory, RegisterPair::HL, 1),
            0x32 => self.ld_indirect(memory, RegisterPair::HL, -1),

            // Single register ALU operations
            // 0x80..=0xBF => self.alu_r(opcode), // TODO: Decode the opcode to determine the operation and register
            0xA8..=0xAF => self.xor(self.get_alu_register(instruction, memory)),

            0xCB => self.extended_instruction(memory),

            _ => panic!("Unknown opcode: 0x{:02X}", instruction),
        }
        // dbg!(&self);
    }

    fn fetch_instruction(&mut self, memory: &crate::gameboy::memory::Memory) -> u8 {
        let instruction = memory.read_byte(self.pc);
        self.pc = self.pc.wrapping_add(1);
        instruction
    }

    fn extended_instruction(&mut self, memory: &mut crate::gameboy::memory::Memory) {
        let extended_opcode = self.fetch_instruction(memory);
        match extended_opcode {
            0x40..=0x7F => self.bit(extended_opcode, memory),
            _ => panic!("Unknown extended opcode: 0x{:02X}", extended_opcode),
        }
    }

    // -------------------------------
    // --- Opcode Decoding Helpers ---
    // -------------------------------

    // Gets the register value from bits 0-2 of opcode (for ALU and load instructions)
    fn get_alu_register(&self, opcode: u8, memory: &crate::gameboy::memory::Memory) -> u8 {
        match opcode & 0x07 {
            0x00 => self.registers.b,
            0x01 => self.registers.c,
            0x02 => self.registers.d,
            0x03 => self.registers.e,
            0x04 => self.registers.h,
            0x05 => self.registers.l,
            0x06 => memory.read_byte(self.registers.get_hl()), // (HL) indirect
            0x07 => self.registers.a,
            _ => unreachable!(),
        }
    }

    // Gets the pair enum from bits 4-5 of opcode (for 16-bit operations)
    fn get_register_pair_from_opcode(&self, opcode: u8) -> RegisterPair {
        match (opcode >> 4) & 0x03 {
            0x00 => RegisterPair::BC,
            0x01 => RegisterPair::DE,
            0x02 => RegisterPair::HL,
            0x03 => RegisterPair::SP,
            _ => panic!("Invalid register pair code in opcode: 0x{:02X}", opcode),
        }
    }

    fn get_bit_from_opcode(&self, opcode: u8) -> u8 {
        (opcode >> 3) & 0x07
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
        // Perform XOR operation
        let result = self.registers.a ^ value;
        self.registers.a = result;

        // Set flags: Z=1, N=0, H=0, C=0
        self.flags.set(result == 0, false, false, false);
        println!("Result of XOR A: 0x{:02X}", result);
    }

    fn ld_indirect(&mut self, memory: &mut crate::gameboy::memory::Memory, register_pair: RegisterPair, delta: i8) {
        // Get the base address from the specified register pair
        let base_address = match register_pair {
            RegisterPair::BC => self.registers.get_bc(),
            RegisterPair::DE => self.registers.get_de(),
            RegisterPair::HL => self.registers.get_hl(),
            RegisterPair::SP => self.sp,
        };

        // Store the value of A into memory at the base address
        memory.write_byte(base_address, self.registers.a);

        // Adjust the base address by delta if needed
        if delta != 0 {
            let adjusted_address = base_address.wrapping_add(delta as u16);
            match register_pair {
                RegisterPair::BC => self.registers.set_bc(adjusted_address),
                RegisterPair::DE => self.registers.set_de(adjusted_address),
                RegisterPair::HL => self.registers.set_hl(adjusted_address),
                RegisterPair::SP => self.sp = adjusted_address,
            }
        }

        println!("Stored 0x{:02X} into memory at 0x{:04X}, adjusted by delta {}", self.registers.a, base_address, delta);
    }

    fn bit(&mut self, opcode: u8, memory: &crate::gameboy::memory::Memory) {
        let bit_position = self.get_bit_from_opcode(opcode);
        let value = self.get_alu_register(opcode, memory);
        let bit_set = (value >> bit_position) & 0x01 != 0;

        // Set flags: Z = !(bit set), N = 0, H = 1
        self.flags.set(!bit_set, false, true, self.flags.carry);
        println!("Tested bit {} of value 0x{:02X}, bit set: {}", bit_position, value, bit_set);
    }
}

