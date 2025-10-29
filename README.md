# Game Boy Emulator

A Game Boy emulator written in Rust.

## Proposed Architecture

The project would follow a modular architecture with these directories:

**src/gameboy/** - Core emulation logic
- `cpu/` - Sharp LR35902 CPU implementation (registers, instructions, opcodes)
- `memory/` - Memory management and cartridge handling
- `gpu/` - Picture Processing Unit for graphics rendering
- `apu/` - Audio Processing Unit for sound generation
- Individual modules for timer, interrupts, and system bus

**src/frontend/** - Platform-specific UI layers
- `sdl/` - Desktop frontend using SDL2 for window/input/audio
- `web/` - WebAssembly frontend for browser deployment

**src/debugger/** - Development tools for disassembly and memory inspection

**src/utils/** - Shared utility functions for bit manipulation

**tests/** - Integration tests for emulation components
**examples/** - Usage examples and demos
**docs/** - Technical documentation
**benches/** - Performance benchmarks

This separates core emulation from UI concerns and allows testing components in isolation.

## Commands

cargo build
cargo run
cargo test
cargo build --release
cargo run --release
