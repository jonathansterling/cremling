mod gameboy;
use crate::gameboy::Gameboy;

fn main() {
    let mut gameboy = Gameboy::new();
    gameboy.configure();
    gameboy.run();
}
