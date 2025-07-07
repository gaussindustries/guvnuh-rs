#![no_std]#![no_main]
use cortex_m_rt::entry;
#[entry] fn main() -> ! { loop { cortex_m::asm::nop(); } }
