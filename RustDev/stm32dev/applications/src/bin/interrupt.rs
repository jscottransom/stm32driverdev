#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use panic_halt as _;
use stm32f3xx_hal::{self as hal, pac, prelude::*};
use stm32f3::stm32f303;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    let mut core_peripherals = stm32f303::CorePeripherals::take().unwrap();

    // Enable the EXTI0 Line Interrupt
    core_peripherals.NVIC.enable(stm32f303::Interrupt::EXTI0);
    dp.EXTI0.enable(stm32f303::Interrupt::EXTI10);

    loop {}
}

#[interrupt]
fn EXTI0() {
    let response = hprintln!("Button Pressed!");
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    let mut led = gpioe
        .pe13
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

    // Toggle LED
    led.toggle().unwrap();


}