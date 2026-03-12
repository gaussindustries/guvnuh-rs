use stm32h7xx_hal::{
    device::{TIM1, TIM2},
    gpio::{self, Output, PushPull},
    pac,
    prelude::*,
    pwm,
    qei::{Qei, QeiExt},
    rcc::CoreClocks, // We only need CoreClocks, not the full Ccdr
    serial,
};

pub struct Board {
    pub clocks: CoreClocks,

    //OUTPUTS
    //LEDs red, yellow, green respectively
    pub ld1: gpio::Pin<'B', 0, Output<PushPull>>,
    pub ld2: gpio::Pin<'E', 1, Output<PushPull>>,
    pub ld3: gpio::Pin<'B', 14, Output<PushPull>>,

    //this enables motor power, will be used in E-Stop
    pub relay: gpio::Pin<'E', 0, Output<PushPull>>,

    pub motor_pwm: pwm::Pwm<TIM1, 0, pwm::ComplementaryDisabled>,

    //UART
    pub tx: serial::Tx<pac::UART4>,

    //INPUTS
    //batch for AMT102-V | rotary encoder (sys rpm)
    pub encoder: Qei<TIM2>,

    //UART
    pub rx: serial::Rx<pac::UART4>,
}

pub fn setup(dp: pac::Peripherals) -> Board {
    // 1. Power & Clocks
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();
    let rcc = dp.RCC.constrain();

    // 'ccdr' holds the Tokens (peripheral) and the Speeds (clocks)
    let ccdr = rcc
        .sysclk(64.MHz()) // Lower speed for stability
        .freeze(pwrcfg, &dp.SYSCFG);

    // 2. GPIO Split (Consumes GPIOB/E tokens from ccdr)
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    // 3. Pin Config
    let mut ld1 = gpiob.pb0.into_push_pull_output();
    let mut ld2 = gpioe.pe1.into_push_pull_output();
    let mut ld3 = gpiob.pb14.into_push_pull_output();

    // PWM Setup (Pin D7[PWM])
    let pwm_pin = gpioe.pe9.into_alternate::<1>();

    // Consumes TIM1 token from ccdr
    let mut motor_pwm = dp.TIM1.pwm(
        pwm_pin,
        20.kHz(),
        ccdr.peripheral.TIM1, // <--- Token MOVED here
        &ccdr.clocks,
    );

    motor_pwm.enable();
    motor_pwm.set_duty(motor_pwm.get_max_duty());

    let mut relay = gpioe.pe0.into_push_pull_output();

    //a & b channel pair for rotary encoder
    let enc_pin_a = gpioa.pa0.into_alternate::<1>();
    let enc_pin_b = gpioa.pa1.into_alternate::<1>();

    // Call .qei() with ONLY the pins and the peripheral token
    let encoder = dp.TIM2.qei((enc_pin_a, enc_pin_b), ccdr.peripheral.TIM2);

    // 1. Configure UART4 Pins (for DMA to esp32)
    let tx_pin = gpioc.pc10.into_alternate::<8>();
    let rx_pin = gpioc.pc11.into_alternate::<8>();

    // 2. Initialize the Serial port at 115200 baud
    let mut serial = dp
        .UART4
        .serial(
            (tx_pin, rx_pin),
            115_200.bps(),
            ccdr.peripheral.UART4,
            &ccdr.clocks,
        )
        .unwrap();

    // 3. Split it so we only keep the Transmitter (Tx)
    let (tx, rx) = serial.split();

    // 4. Initial Safety State
    ld1.set_low();
    ld2.set_low();
    ld3.set_low();

    //we need to send our pwm signal into the pwm controller first before we enable power to the dc motor
    relay.set_low();

    Board {
        ld1,
        ld2,
        ld3,
        relay,
        motor_pwm,
        encoder,
        tx,
        rx,
        clocks: ccdr.clocks,
    }
}
