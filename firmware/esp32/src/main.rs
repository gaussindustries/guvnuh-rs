#![no_std]
#![no_main]

/**
 * this will entail with the help of shared models and forethought of the data herein collected from the stm32
 *
 * insofar as the stm32 can manage to do so, I don't think it makes much sense to gather second hand information,
 * it may be prudent to develop the control loop so that within different priorities and where data is more crucial we can assign
 * differing or dynamic priorities in order to assert that the stm32 will remain in tolerance no matter what,
 * running efficiently and within spec is top priority of course but ensuring that esp32 is hydrated so we have ample resoultion of data
 * is crucial to developing the project further and in a clever manner.
 *
 * the esp32 will facilitate communications to and from the stm32 and the api server separately once recieved data,
 * which said server will store data for display to the masses
 * 		(and of course to our desktop application,
 * 			this application will be able to be deployed as a webclient via dioxus
 * 				with sub/prime collected data and more of a professional write up to show thought process, progress, and more importantly 67)
 */
use esp_backtrace as _;
use esp_hal::{
    gpio::{Level, Output, OutputConfig},
    main,
    uart::{Config, Uart},
};
use esp_println::{print, println};

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Onboard LED on GPIO2 — lights up when booted and ready
    let mut led = Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default());

    let mut uart1 = Uart::new(peripherals.UART1, Config::default())
        .unwrap()
        .with_tx(peripherals.GPIO17)
        .with_rx(peripherals.GPIO16);

    // Ready — LED on
    led.set_high();
    println!("ESP32 Gateway Booted. Listening to STM32...");

    let mut buf = [0u8; 1];
    loop {
        if let Ok(bytes_read) = uart1.read(&mut buf) {
            if bytes_read > 0 {
                led.toggle();
                print!("{}", buf[0] as char);
            }
        }
    }
}
