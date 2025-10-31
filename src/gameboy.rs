mod cpu;
mod memory;
use cpu::CPU;
use memory::Memory;

#[derive(Debug)]
pub struct Gameboy {
    // APU
    pub cpu: CPU,
    // GPU
    pub memory: Memory,
}

impl Gameboy {
     pub fn new() -> Gameboy {
        Gameboy {
            cpu: CPU::default(),
            memory: Memory::new(),
        }
    }

    pub fn configure(&mut self) {
        println!("Configuring Gameboy");
        self.memory.load_boot_rom();
        self.memory.print_boot_rom();
        self.memory.load_rom();
    }

    pub fn run(&mut self) {
        println!("Running Gameboy");
       
        let mut instruction_count = 0;
        loop {
            self.cpu.fetch_decode_execute(&mut self.memory);
            instruction_count += 1;
            println!("Instruction Count: {}", instruction_count);
        }
    }
}
