#![no_std]
#![no_main]
#![no_mangle]

use panic_itm;
use cortex_m_rt::{entry, exception, ExceptionFrame};
use cortex_m_semihosting::hio;
use core::ptr::{read_volatile, write_volatile};
use core::str;
// use stm32f3::stm32f303::{interrupt, Interrupt, NVIC};
use core::fmt::Write;

#[entry]
fn main() -> ! {

    // First we'll establish the locations of the registers we need to turn on an LED
    // We need to employ type casting in this situation, since the "type" of the register variables
    // is a raw pointer of type u32.


    const rcc_reg: *mut u32 = 0x4002_1014 as *mut u32;
    const pe_mode: *mut u32 = 0x4800_1000 as *mut u32;
    const pe_odr: *mut u32 = 0x4800_1014 as *mut u32;


    // Step 1 -  Enable the clock by setting bit 21 of this RCC Port E register
    // Since this is a memory mapped register, we'll need to treat access as volatile (by using unsafe).
    // The variable itself is not marked as unsafe, but rather the code which implements access.
    unsafe {write_volatile(rcc_reg, read_volatile(rcc_reg) | (1 << 21)); }

    // Step 2 -  Clear the 28th & 29th Bit Positions, then write a 1 to the 28thth Bit Position
    // Port E, Pin 14 covers two bits, and we'd like to configure as output, therefore we must put
    // 01 in the registers.
    unsafe {write_volatile(pe_mode, read_volatile(pe_mode) & !(3 << 28)); }
    unsafe {write_volatile(pe_mode, read_volatile(pe_mode) | (1 << 28)); }

    unsafe {write_volatile(pe_odr, read_volatile(pe_odr) | (1 << 14)); }


    loop {}
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    if let Ok(mut hstdout) = hio::hstdout() {
        writeln!(hstdout, "{:#?}", ef).ok();
    }

    loop {}
}

