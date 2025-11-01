// TODO: Flag resolution and updating can be refactored to reduce code duplication

use crate::gameboy::cpu::registers::{R8, R16, Registers, Conditions};

#[derive(Debug)]
#[derive(Default)]
pub struct CPU {
    pc: u16,         // Program Counter
    sp: u16,         // Stack Pointer
    registers: Registers,
    instruction: u8,
    ime: bool,   // Interrupt Master Enable
    ime_enable_pending: bool,
    halted: bool,
    halt_bug: bool,
}

impl CPU {
    pub fn fetch_decode_execute(&mut self, memory: &mut crate::gameboy::memory::Memory) {
        // Fetch the next instruction
        println!("PC: 0x{:04X}", self.pc);
        self.instruction = self.fetch_instruction(memory);
        // println!("Fetched instruction: 0x{:02X}", instruction);

        // TODO: Handle HALTS and service interrupts correctly. This is mostly placeholder for now.
        // See https://gbdev.io/pandocs/halt.html
        if self.halted {
            if self.ime_enable_pending {
                self.halted = false;
                if self.ime == true {
                    self.service_interrupt(memory, 0x40); // Example interrupt vector
                } else {
                    self.tick();
                    return;
                }
            }
        }

        // Decode and execute the instruction
        // dbg!(&self);
        match self.instruction {
            0x06 | 0x16 | 0x26 | 0x36 | 0x0E | 0x1E | 0x2E | 0x3E => 
                self.ld_immediate(memory),

            0x07 => self.rotate_left_circular_a(),
            0x17 => self.rotate_left_a(),

            0x18 => self.conditional_jump_relative(memory, Conditions::NONE),
            0x20 => self.conditional_jump_relative(memory, Conditions::NZ),
            0x28 => self.conditional_jump_relative(memory, Conditions::Z),
            0x30 => self.conditional_jump_relative(memory, Conditions::NC),
            0x38 => self.conditional_jump_relative(memory, Conditions::C),

            0x01 | 0x11 | 0x21 | 0x31 =>
                self.ld_nn(memory),

            0x02 => self.ld_indirect_dest(memory, R16::BC, 0),
            0x12 => self.ld_indirect_dest(memory, R16::DE, 0),
            0x22 => self.ld_indirect_dest(memory, R16::HL, 1),
            0x32 => self.ld_indirect_dest(memory, R16::HL, -1),

            0x0A => self.ld_indirect_src(memory, R16::BC, 0),
            0x1A => self.ld_indirect_src(memory, R16::DE, 0),
            0x2A => self.ld_indirect_src(memory, R16::HL, 1),
            0x3A => self.ld_indirect_src(memory, R16::HL, -1),

            // ALU operations
            // 0x80..=0xBF => self.alu_r(opcode), // TODO: Decode the opcode to determine the operation and register
            0xA8..=0xAF => self.xor(memory),
            0xB8..=0xBF => self.compare_r8(memory),
            0xFE => self.compare_d8(memory),
            0x40..=0x7F if self.instruction != 0x76 => self.ld_r8_r8(memory),
            0x76 => self.halt(),
            0x04 | 0x14 | 0x24 | 0x34 | 0x0C | 0x1C | 0x2C | 0x3C =>
                self.inc_r8(memory),
            0x05 | 0x15 | 0x25 | 0x35 | 0x0D | 0x1D | 0x2D | 0x3D =>
                self.dec_r8(memory),
            0x03 | 0x13 | 0x23 | 0x33 =>
                self.inc_r16(memory),
            0x0B | 0x1B | 0x2B | 0x3B =>
                self.dec_r16(memory),

            0xE2 => self.ld_c_indirect_a(memory),

            // Calls
            0xC4 => self.call(memory, Conditions::NZ),
            0xD4 => self.call(memory, Conditions::NC),
            0xCC => self.call(memory, Conditions::Z),
            0xDC => self.call(memory, Conditions::C),
            0xCD => self.call(memory, Conditions::NONE),

            // Returns
            0xC0 => self.ret(memory, Conditions::NZ),
            0xC8 => self.ret(memory, Conditions::Z),
            0xD0 => self.ret(memory, Conditions::NC),
            0xD8 => self.ret(memory, Conditions::C),
            0xC9 => self.ret(memory, Conditions::NONE),

            // Interrupts
            0xD9 => self.reti(memory),
            0xFB => self.pending_enable_interrupts(),
            0xF3 => self.disable_interrupts(),

            // LDH
            0xE0 => self.ldh_imm_a(memory),
            0xF0 => self.ldh_a_imm(memory),
            0xE8 => self.ldh_c_a(memory),
            0xF8 => self.ldh_a_c(memory),

            0xC1 | 0xD1 | 0xE1 | 0xF1 =>
                self.pop(memory),
            0xC5 | 0xD5 | 0xE5 | 0xF5 =>
                self.push(memory),

            0xCB => self.extended_instruction(memory),

            _ => panic!("Unknown opcode: 0x{:02X}", self.instruction),
        }
        // dbg!(&self);

        if self.ime_enable_pending {
            self.enable_interrupts();
            self.ime_enable_pending = false;
        }
    }

    fn fetch_instruction(&mut self, memory: &crate::gameboy::memory::Memory) -> u8 {
        let instruction = memory.read_byte(self.pc);
        self.pc = self.pc.wrapping_add(1);
        instruction
    }

    fn extended_instruction(&mut self, memory: &mut crate::gameboy::memory::Memory) {
        self.instruction = self.fetch_instruction(memory);
        match self.instruction {
            0x10..=0x17 => self.rotate_left(memory),
            0x40..=0x7F => self.bit(memory),
            _ => panic!("Unknown extended opcode: 0x{:02X}", self.instruction),
        }
    }

    fn tick(&mut self) {
        // Placeholder for tick implementation
        println!("CPU tick");
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

    fn get_r8_from_opcode(&self, opcode: u8) -> crate::gameboy::cpu::registers::R8 {
        match opcode & 0x07 {
            0x00 => R8::B,
            0x01 => R8::C,
            0x02 => R8::D,
            0x03 => R8::E,
            0x04 => R8::H,
            0x05 => R8::L,
            0x06 => R8::HL, // (HL) indirect
            0x07 => R8::A,
            _ => panic!("Invalid single register code in opcode: 0x{:02X}", opcode),
        }
    }

    // Gets the pair enum from bits 4-5 of opcode (for 16-bit operations)
    fn get_r16_from_ld_inc_dec_opcode(&self, opcode: u8) -> R16 {
        match (opcode >> 4) & 0x03 {
            0x00 => R16::BC,
            0x01 => R16::DE,
            0x02 => R16::HL,
            0x03 => R16::SP,
            _ => panic!("Invalid register pair code in opcode: 0x{:02X}", opcode),
        }
    }

    fn get_r16_from_push_pop_opcode(&self, opcode: u8) -> R16 {
        match (opcode >> 4) & 0x03 {
            0x00 => R16::BC,
            0x01 => R16::DE,
            0x02 => R16::HL,
            0x03 => R16::AF,
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

    // --- Misc Loads ---

    fn ld_immediate(&mut self, memory: &mut crate::gameboy::memory::Memory) {
        // Fetch the immediate byte
        let value = self.fetch_instruction(memory);

        // Determine the target register from the opcode and write to it
        let target_register = self.get_r8_from_opcode(self.instruction);
        self.registers.write_r8(target_register, value, memory);

        println!("Loaded immediate 0x{:02X} into {:?}", value, target_register);
    }

    fn ld_nn(&mut self, memory: &crate::gameboy::memory::Memory) {
        // Fetch the next two bytes for the immediate value
        let value = self.fetch_two_bytes(memory);

        // Load the value into the specified register pair
        let register_pair = self.get_r16_from_ld_inc_dec_opcode(self.instruction);

        // Handle SP separately since it's in CPU, not Registers
        if let R16::SP = register_pair {
            self.sp = value;
        } else {
            self.registers.write_r16(register_pair, value);
        }

        println!("Loaded 0x{:04X} into {:?}", value, register_pair);
    }

    fn ld_r8_r8(&mut self, memory: &mut crate::gameboy::memory::Memory) {
        let dest_register = self.get_r8_from_opcode((self.instruction >> 3) & 0x07);
        let src_register = self.get_r8_from_opcode(self.instruction & 0x07);
        let value = self.registers.read_r8(src_register, memory);
        self.registers.write_r8(dest_register, value, memory);

        println!("Loaded 0x{:02X} from {:?} into {:?}", value, src_register, dest_register);
    }

    fn ld_indirect_dest(&mut self, memory: &mut crate::gameboy::memory::Memory, register_pair: R16, delta: i8) {
        // Get the base address from the specified register pair
        let base_address = match register_pair {
            R16::BC => self.registers.get_bc(),
            R16::DE => self.registers.get_de(),
            R16::HL => self.registers.get_hl(),
            R16::SP => self.sp,
            R16::AF => panic!("AF register pair cannot be used for indirect memory access"),
        };

        // Store the value of A into memory at the base address
        memory.write_byte(base_address, self.registers.a);

        // Adjust the base address by delta if needed
        if delta != 0 {
            let adjusted_address = base_address.wrapping_add_signed(delta as i16);
            match register_pair {
                R16::BC => self.registers.set_bc(adjusted_address),
                R16::DE => self.registers.set_de(adjusted_address),
                R16::HL => self.registers.set_hl(adjusted_address),
                R16::SP => self.sp = adjusted_address,
                R16::AF => panic!("AF register pair cannot be used for indirect memory access"),
            }
        }

        println!("Stored 0x{:02X} into memory at 0x{:04X}, adjusted by delta {}", self.registers.a, base_address, delta);
    }

    fn ld_indirect_src(&mut self, memory: &crate::gameboy::memory::Memory, register_pair: R16, delta: i8) {
        // Get the base address from the specified register pair
        let base_address = match register_pair {
            R16::BC => self.registers.get_bc(),
            R16::DE => self.registers.get_de(),
            R16::HL => self.registers.get_hl(),
            R16::SP => self.sp,
            R16::AF => panic!("AF register pair cannot be used for indirect memory access"),
        };

        // Load the value from memory at the base address into A
        let value = memory.read_byte(base_address);
        self.registers.a = value;

        // Adjust the source address by delta if needed
        if delta != 0 {
            let adjusted_address = base_address.wrapping_add_signed(delta as i16);
            match register_pair {
                R16::BC => self.registers.set_bc(adjusted_address),
                R16::DE => self.registers.set_de(adjusted_address),
                R16::HL => self.registers.set_hl(adjusted_address),
                R16::SP => self.sp = adjusted_address,
                R16::AF => panic!("AF register pair cannot be used for indirect memory access"),
            }
        }

        println!("Loaded 0x{:02X} from memory at 0x{:04X} into A, adjusted by delta {}", value, base_address, delta);
    }

    fn ld_c_indirect_a(&mut self, memory: &mut crate::gameboy::memory::Memory) {
        let address = 0xFF00 | (self.registers.c as u16);
        memory.write_byte(address, self.registers.a);
        println!("Stored A (0x{:02X}) into address 0x{:04X}", self.registers.a, address);
    }

    // --- High RAM / I/O Region Loads ---

    fn ldh_imm_a(&mut self, memory: &mut crate::gameboy::memory::Memory) {
        // Fetch the immediate 8-bit address offset
        let offset = self.fetch_instruction(memory);
        let address = 0xFF00 | (offset as u16);

        // Temporary panic to stop execution once the boot ROM execution is complete
        if address == 0xFF50 {
            panic!("Boot ROM access disabled! Ready to switch to cartridge ROM.");
        }

        // Load the value of A into the calculated address
        memory.write_byte(address, self.registers.a);

        println!("Stored A (0x{:02X}) into address 0x{:04X}", self.registers.a, address);
    }

    fn ldh_a_imm(&mut self, memory: &crate::gameboy::memory::Memory) {
        // Fetch the immediate 8-bit address offset
        let offset = self.fetch_instruction(memory);
        let address = 0xFF00 | (offset as u16);

        // Load the value from the calculated address into A
        let value = memory.read_byte(address);
        self.registers.a = value;

        println!("Loaded 0x{:02X} from address 0x{:04X} into A", value, address);
    }

    fn ldh_c_a(&mut self, memory: &mut crate::gameboy::memory::Memory) {
        let address = 0xFF00 | (self.registers.c as u16);
        memory.write_byte(address, self.registers.a);
        println!("Stored A (0x{:02X}) into address 0x{:04X}", self.registers.a, address);
    }

    fn ldh_a_c(&mut self, memory: &crate::gameboy::memory::Memory) {
        let address = 0xFF00 | (self.registers.c as u16);
        let value = memory.read_byte(address);
        self.registers.a = value;
        println!("Loaded 0x{:02X} from address 0x{:04X} into A", value, address);
    }

    // --- ALU ---

    fn xor(&mut self, memory: &crate::gameboy::memory::Memory) {
        let register = self.get_r8_from_opcode(self.instruction);
        let value = self.registers.read_r8(register, memory);

        // Perform XOR operation
        let result = self.registers.a ^ value;
        self.registers.a = result;

        // Set flags: Z=1, N=0, H=0, C=0
        self.registers.set_flags(result == 0, false, false, false);
        println!("Result of XOR A with {:?}: 0x{:02X}", register, result);
    }

    fn inc_r8(&mut self, memory: &mut crate::gameboy::memory::Memory) {
        let register = self.get_r8_from_opcode(self.instruction);
        let old_value = self.registers.read_r8(register, memory);
        let new_value = old_value.wrapping_add(1);
        self.registers.write_r8(register, new_value, memory);

        // Set flags: Z if result is zero, N=0, H if carry from bit 3, C unchanged
        let half_carry = (old_value & 0x0F) == 0x0F;
        let carry = self.registers.get_carry_flag();
        self.registers.set_flags(new_value == 0, false, half_carry, carry);
        println!("Incremented {:?}: 0x{:02X} -> 0x{:02X}", register, old_value, new_value);
    }

    fn dec_r8(&mut self, memory: &mut crate::gameboy::memory::Memory) {
        let register = self.get_r8_from_opcode(self.instruction);
        let old_value = self.registers.read_r8(register, memory);
        let new_value = old_value.wrapping_sub(1);
        self.registers.write_r8(register, new_value, memory);

        // Set flags: Z if result is zero, N=1, H if borrow from bit 4, C unchanged
        let half_borrow = (old_value & 0x0F) == 0x00;
        let carry = self.registers.get_carry_flag();
        self.registers.set_flags(new_value == 0, true, half_borrow, carry);
        println!("Decremented {:?}: 0x{:02X} -> 0x{:02X}", register, old_value, new_value);
    }

    fn inc_r16(&mut self, _memory: &mut crate::gameboy::memory::Memory) {
        let register_pair = self.get_r16_from_ld_inc_dec_opcode(self.instruction);

        // Handle SP separately since it's in CPU, not Registers
        let old_value = if let R16::SP = register_pair {
            self.sp
        } else {
            self.registers.read_r16(register_pair)
        };

        let new_value = old_value.wrapping_add(1);

        if let R16::SP = register_pair {
            self.sp = new_value;
        } else {
            self.registers.write_r16(register_pair, new_value);
        }

        println!("Incremented {:?}: 0x{:04X} -> 0x{:04X}", register_pair, old_value, new_value);
    }

    fn compare_d8(&mut self, memory: &mut crate::gameboy::memory::Memory) {
        // TODO: compare methods can probably be refactored to reduce code duplication

        // Compare the value in register A with an immediate 8-bit value
        let value = self.fetch_instruction(memory);
        let a_value = self.registers.a;
        let result = a_value.wrapping_sub(value);
        // Set flags: Z if result is zero, N=1, H if borrow from bit 4, C if borrow
        let half_borrow = (a_value & 0x0F) < (value & 0x0F);
        let carry = a_value < value;
        self.registers.set_flags(result == 0, true, half_borrow, carry);
        println!("Compared A (0x{:02X}) with immediate 0x{:02X}", a_value, value);
    }

    fn compare_r8(&mut self, memory: &mut crate::gameboy::memory::Memory) {
        // Compare the value in register A with another register
        let register = self.get_r8_from_opcode(self.instruction);
        let value = self.registers.read_r8(register, memory);
        let a_value = self.registers.a;
        let result = a_value.wrapping_sub(value);
        // Set flags: Z if result is zero, N=1, H if borrow from bit 4, C if borrow
        let half_borrow = (a_value & 0x0F) < (value & 0x0F);
        let carry = a_value < value;
        self.registers.set_flags(result == 0, true, half_borrow, carry);
        println!("Compared A (0x{:02X}) with {:?} (0x{:02X})", a_value, register, value);
    }

    fn dec_r16(&mut self, _memory: &mut crate::gameboy::memory::Memory) {
        let register_pair = self.get_r16_from_ld_inc_dec_opcode(self.instruction);

        // Handle SP separately since it's in CPU, not Registers
        let old_value = if let R16::SP = register_pair {
            self.sp
        } else {
            self.registers.read_r16(register_pair)
        };

        let new_value = old_value.wrapping_sub(1);

        if let R16::SP = register_pair {
            self.sp = new_value;
        } else {
            self.registers.write_r16(register_pair, new_value);
        }

        println!("Decremented {:?}: 0x{:04X} -> 0x{:04X}", register_pair, old_value, new_value);
    }

    // --- Uncategorized ---

    fn rotate_left_a(&mut self) {
        // RLA: Rotate A left through carry
        let old_value = self.registers.a;
        let carry_in = if self.registers.get_carry_flag() { 1 } else { 0 };
        let new_value = (old_value << 1) | carry_in;
        self.registers.a = new_value;

        // Set flags: Z=0, N=0, H=0, C=old bit 7
        // Note: Z is always 0 for RLA (unlike extended rotate instructions)
        let carry_out = (old_value & 0x80) != 0;
        self.registers.set_flags(false, false, false, carry_out);
        println!("Rotated left A: 0x{:02X} -> 0x{:02X}", old_value, new_value);
    }

    fn rotate_left_circular_a(&mut self) {
        // RLCA: Rotate A left circular (bit 7 to carry and bit 0)
        let old_value = self.registers.a;
        let bit_7 = (old_value & 0x80) >> 7;
        let new_value = (old_value << 1) | bit_7;
        self.registers.a = new_value;

        // Set flags: Z=0, N=0, H=0, C=old bit 7
        // Note: Z is always 0 for RLCA (unlike extended rotate instructions)
        let carry_out = (old_value & 0x80) != 0;
        self.registers.set_flags(false, false, false, carry_out);
        println!("Rotated left circular A: 0x{:02X} -> 0x{:02X}", old_value, new_value);
    }

    fn conditional_jump_relative(&mut self, memory: &crate::gameboy::memory::Memory, condition: Conditions) {
        // Fetch the signed 8-bit offset
        let offset = self.fetch_instruction(memory) as i8;

        if self.registers.check_condition(condition) {
            // Calculate the new PC by adding the signed offset
            self.pc = self.pc.wrapping_add(offset as u16);
            println!("Jumped to address 0x{:04X} on condition {:?} (offset: {})", self.pc, condition, offset);
        } else {
            println!("Did not jump on condition {:?}, PC remains at 0x{:04X}", condition, self.pc);
        }
    }

    fn pop(&mut self, memory: &mut crate::gameboy::memory::Memory) {
        let register_pair = self.get_r16_from_push_pop_opcode(self.instruction);

        // Pop low byte, then high byte
        let low_byte = memory.read_byte(self.sp) as u16;
        self.sp = self.sp.wrapping_add(1);
        let high_byte = memory.read_byte(self.sp) as u16;
        self.sp = self.sp.wrapping_add(1);

        let value = (high_byte << 8) | low_byte;
        self.registers.write_r16(register_pair, value);

        println!("Popped 0x{:04X} into {:?}", value, register_pair);
    }

    fn push(&mut self, memory: &mut crate::gameboy::memory::Memory) {
        let register_pair = self.get_r16_from_push_pop_opcode(self.instruction);
        let value = self.registers.read_r16(register_pair);

        // Push high byte, then low byte
        let high_byte = (value >> 8) as u8;
        let low_byte = (value & 0x00FF) as u8;
        self.sp = self.sp.wrapping_sub(1);
        memory.write_byte(self.sp, high_byte);
        self.sp = self.sp.wrapping_sub(1);
        memory.write_byte(self.sp, low_byte);

        println!("Pushed 0x{:04X} from {:?} onto stack", value, register_pair);
    }

    fn call(&mut self, memory: &mut crate::gameboy::memory::Memory, condition: Conditions) {
        if self.registers.check_condition(condition) {
            // Fetch the target address
            let target_address = self.fetch_two_bytes(memory);

            // Push the current PC onto the stack
            let pc_high = (self.pc >> 8) as u8;
            let pc_low = (self.pc & 0x00FF) as u8;
            self.sp = self.sp.wrapping_sub(1);
            memory.write_byte(self.sp, pc_high);
            self.sp = self.sp.wrapping_sub(1);
            memory.write_byte(self.sp, pc_low);

            // Jump to the target address
            self.pc = target_address;

            println!("Called address 0x{:04X} on condition {:?}", target_address, condition);
        } else {
            // If the condition is not met, just skip the two bytes
            self.pc = self.pc.wrapping_add(2);
            println!("Did not call on condition {:?}, PC remains at 0x{:04X}", condition, self.pc);
        }
    }

    fn ret(&mut self, memory: &crate::gameboy::memory::Memory, condition: Conditions) {
        if self.registers.check_condition(condition) {
            self.return_from_interrupt(memory);
            println!("Returned to address 0x{:04X} on condition {:?}", self.pc, condition);
        } else {
            println!("Did not return on condition {:?}, PC remains at 0x{:04X}", condition, self.pc);
        }
    }

    fn reti(&mut self, memory: &crate::gameboy::memory::Memory) {
        self.return_from_interrupt(memory);

        // Enable interrupts after returning
        self.enable_interrupts();

        println!("Returned from interrupt to address 0x{:04X}", self.pc);
    }

    fn return_from_interrupt(&mut self, memory: &crate::gameboy::memory::Memory) {
        // Pop the return address from the stack
        let low_byte = memory.read_byte(self.sp) as u16;
        self.sp = self.sp.wrapping_add(1);
        let high_byte = memory.read_byte(self.sp) as u16;
        self.sp = self.sp.wrapping_add(1);
        self.pc = (high_byte << 8) | low_byte;
    }

    fn pending_enable_interrupts(&mut self) {
        self.ime_enable_pending = true;
        println!("Interrupts will be enabled after the next instruction");
    }

    fn disable_interrupts(&mut self) {
        self.ime = false;
        println!("Disabled interrupts");
    }

    fn enable_interrupts(&mut self) {
        self.ime = true;
        println!("Enabled interrupts");
    }

    fn halt(&mut self) {
        self.halted = true;
    }

    fn service_interrupt(&mut self, memory: &mut crate::gameboy::memory::Memory, interrupt_vector: u16) {
        // Disable further interrupts
        self.ime = false;

        // Push the current PC onto the stack
        let pc_high = (self.pc >> 8) as u8;
        let pc_low = (self.pc & 0x00FF) as u8;
        self.sp = self.sp.wrapping_sub(1);
        memory.write_byte(self.sp, pc_high);
        self.sp = self.sp.wrapping_sub(1);
        memory.write_byte(self.sp, pc_low);

        // Jump to the interrupt vector
        self.pc = interrupt_vector;

        println!("Serviced interrupt, jumped to vector 0x{:04X}", interrupt_vector);
    }

    // --- Extended ---

    fn bit(&mut self, memory: &crate::gameboy::memory::Memory) {
        let bit_position = self.get_bit_from_opcode(self.instruction);
        let value = self.get_alu_register(self.instruction, memory);
        let bit_set = (value >> bit_position) & 0x01 != 0;

        // Set flags: Z = !(bit set), N = 0, H = 1
        let carry = self.registers.get_carry_flag();
        self.registers.set_flags(!bit_set, false, true, carry);
        println!("Tested bit {} of value 0x{:02X}, bit set: {}", bit_position, value, bit_set);
    }

    fn rotate_left(&mut self, memory: &mut crate::gameboy::memory::Memory) {
        let register = self.get_r8_from_opcode(self.instruction);
        let old_value = self.registers.read_r8(register, memory);
        let carry_in = if self.registers.get_carry_flag() { 1 } else { 0 };
        let new_value = (old_value << 1) | carry_in;
        self.registers.write_r8(register, new_value, memory);

        // Set flags: Z if result is zero, N=0, H=0, C=old bit 7
        let carry_out = (old_value & 0x80) != 0;
        self.registers.set_flags(new_value == 0, false, false, carry_out);
        println!("Rotated left {:?}: 0x{:02X} -> 0x{:02X}", register, old_value, new_value);
    }
}

