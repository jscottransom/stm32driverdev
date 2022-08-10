#![no_std]
#![no_main]
#![no_mangle]


use panic_itm;
use log as _;
use rtt_target::{rtt_init_print, rprintln};
use cortex_m_rt::{entry, exception, ExceptionFrame};
use cortex_m_semihosting::hio;
use core::ptr::{read_volatile, write_volatile};
use core::str;
// use stm32f3::stm32f303::{interrupt, Interrupt, NVIC};
use core::fmt::Write;
use volatile_register::RW;
use modular_bitfield::prelude::*;

#[repr(C)]
pub struct RCC_AHBENR {
    rcc_ahbenr: RW<u32>,
    }

#[repr(C)]
struct GPIOx_MODE {
     mode_r0: [RW<u8>; 2],
     mode_r1: [RW<u8>; 2],
     mode_r2: [RW<u8>; 2],
     mode_r3: [RW<u8>; 2],
     mode_r4: [RW<u8>; 2],
     mode_r5: [RW<u8>; 2],
     mode_r6: [RW<u8>; 2],
     mode_r7: [RW<u8>; 2],
     mode_r8: [RW<u8>; 2],
     mode_r9: [RW<u8>; 2],
     mode_r10: [RW<u8>; 2],
     mode_r11: [RW<u8>; 2],
     mode_r12: [RW<u8>; 2],
     mode_r13: [RW<u8>; 2],
     mode_r14: [RW<u8>; 2],
     mode_r15: [RW<u8>; 2]}

#[repr(C)]
struct GPIOx_ODR {

     odr_r0: [RW<bool>; 1],
     odr_r1: [RW<bool>; 1],
     odr_r2: [RW<bool>; 1],
     odr_r3: [RW<bool>; 1],
     odr_r4: [RW<bool>; 1],
     odr_r5: [RW<bool>; 1],
     odr_r6: [RW<bool>; 1],
     odr_r7: [RW<bool>; 1],
     odr_r8: [RW<bool>; 1],
     odr_r9: [RW<bool>; 1],
     odr_r10: [RW<bool>; 1],
     odr_r11: [RW<bool>; 1],
     odr_r12: [RW<bool>; 1],
     odr_r13: [RW<bool>; 1],
     odr_r14: [RW<bool>; 1],
     odr_r15: [RW<bool>; 1],
     reserved: [u16; 16]

}

// #[bitfield]
// struct GPIOx_IDR {
//
//      idr_r0: [RW<bool>; 1],
//      idr_r1: [RW<bool>; 1],
//      idr_r2: [RW<bool>; 1],
//      idr_r3: [RW<bool>; 1],
//      idr_r4: [RW<bool>; 1],
//      idr_r5: [RW<bool>; 1],
//      idr_r6: [RW<bool>; 1],
//      idr_r7: [RW<bool>; 1],
//      idr_r8: [RW<bool>; 1],
//      idr_r9: [RW<bool>; 1],
//      idr_r10: [RW<bool>; 1],
//      idr_r11: [RW<bool>; 1],
//      idr_r12: [RW<bool>; 1],
//      idr_r13: [RW<bool>; 1],
//      idr_r14: [RW<bool>; 1],
//      idr_r15: [RW<bool>; 1],
//      reserved: [RW<bool>; 1]6
// }

#[entry]
fn main() -> ! {

    let mut rcc_reg = 0x4002_1014 as *const RCC_AHBENR;
    let mut pe_mode = 0x4800_1000 as *const GPIOx_MODE;
    let mut pe_odr = 0x4800_1014 as *const GPIOx_ODR;

    rtt_init_print!();
    unsafe {(*rcc_reg).rcc_ahbenr[21].write(1)}

    rprintln!("RCC Val: {}", unsafe {(*rcc_reg).rcc_ahbenr[21].read()});
    unsafe {(*pe_mode).mode_r15[0].write(0)}
    unsafe {(*pe_mode).mode_r15[1].write(0)}
    unsafe {(*pe_mode).mode_r15[1].write(1)}
    rprintln!("RCC Val: {}", unsafe {(*pe_mode).mode_r15[0].read()});
    unsafe {(*pe_odr).odr_r0[0].write(true)}
    rprintln!("RCC Val: {}", unsafe {(*pe_odr).odr_r15[0].read()});

    loop {}
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    if let Ok(mut hstdout) = hio::hstdout() {
        writeln!(hstdout, "{:#?}", ef).ok();
    }

    loop {}
}