#  STM32F3 Discovery Board Drivers

This repository contains custom low-level drivers and sample code for the [STM32F3 Discovery Board](https://www.st.com/en/evaluation-tools/stm32f3discovery.html). It includes both **bare-metal C drivers** written directly from the datasheet, and **Rust drivers** with varying levels of abstraction (from register-level to embedded-hal traits).

---

## What’s Included

### C Drivers
- Written directly from the STM32F3 reference manual and datasheets
- Focused on learning embedded systems fundamentals

Modules:
- GPIO (input/output, pull-up/pull-down, toggling)
- USART (serial transmission)
- EXTI (interrupt-driven GPIO)
- NVIC (interrupt configuration)
- RCC (clock setup)

### 🦀 Rust Drivers
- Mix of bare-metal and higher-level embedded Rust abstractions
- Based on `stm32f3` PAC (Peripheral Access Crate) and optionally `embedded-hal`
- Demonstrates low-level register access as well as idiomatic Rust embedded development

Examples:
- LED blinking with direct register access
- Button + LED interaction with `embedded-hal`

---

##  Getting Started

### Hardware Requirements
- STM32F3DISCOVERY board
- ST-Link v2 debugger (onboard)
- USB to UART (for some UART examples)

#### C Toolchain
- `arm-none-eabi-gcc`
- `make` or `cmake`

#### Rust Toolchain
- `rustup target add thumbv7em-none-eabihf`
- `probe-rs` or `cargo-flash` for flashing
- Optionally: `cargo-embed`, `gdb`, or `openocd` for debugging

---


