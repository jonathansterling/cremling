const BOOT_ROM_SIZE: usize = 0x100; // 256 bytes
const ADDR_SPACE_SIZE: usize = 0x10000; // 64KB

#[derive(Debug)]
pub struct Memory {
    boot_rom: [u8; BOOT_ROM_SIZE],
    addr_space: [u8; ADDR_SPACE_SIZE],
    boot_rom_enabled: bool, // This should be controlled by a specific address or register
}

impl Memory {
    pub fn new() -> Self {
        Self {
            boot_rom: [0; BOOT_ROM_SIZE],
            addr_space: [0; ADDR_SPACE_SIZE],
            boot_rom_enabled: true,
        }
    }

    pub fn enable_boot_rom(&mut self) {
        self.boot_rom_enabled = true;
    }

    pub fn disable_boot_rom(&mut self) {
        self.boot_rom_enabled = false;
    }

    pub fn load_boot_rom(&mut self) {
        // Load boot ROM data from /roms/dmg_boot.bin
        let boot_rom_data = include_bytes!("../../../roms/dmg_boot.bin");
        self.boot_rom[..BOOT_ROM_SIZE].copy_from_slice(boot_rom_data);
    }

    pub fn load_rom(&mut self) {
        let rom_data = include_bytes!("../../../roms/tetris.gb");
        let length = rom_data.len().min(ADDR_SPACE_SIZE);
        self.addr_space[..length].copy_from_slice(&rom_data[..length]);
    }

    pub fn print_boot_rom(&self) {
        for (i, byte) in self.boot_rom.iter().enumerate() {
            println!("0x{:04X}: 0x{:02X}", i, byte);
        }
    }

    pub fn read_byte(&self, addr: u16) -> u8 {
        if self.boot_rom_enabled && (addr as usize) < BOOT_ROM_SIZE {
            self.boot_rom[addr as usize]
        } else {
            self.addr_space[addr as usize]
        }
    }

    pub fn write_byte(&mut self, addr: u16, value: u8) {
        // Prevent writing to boot ROM area when enabled
        if !(self.boot_rom_enabled && (addr as usize) < BOOT_ROM_SIZE) {
            self.addr_space[addr as usize] = value;
        }
    }
}
