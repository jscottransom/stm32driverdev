/// GPIO Driver
#![no_std]
#![no_main]
#![no_mangle]

use stmf3::stmf303;


/// Establish an object to interact with the Ports 
/// The User will need to pass this as an argument to get access
enum Port {
    A,
    B,
    C,
    D,
    E,
    F
}

impl Port {

    fn get() -> stm32f303::Peripherals {
        // Generalized Peripheral Instance
        
        

    }
 

}


struct GPIOConfig {
    Pin: u8,
    Mode: String,
    Speed: String,
    PuPd: String,
    OType: String,
    AltFun: String
}


