use stm32h7xx_hal::{
    device::{TIM1, TIM2},
    gpio::{self, ExtiPin, Input, Output, PushPull},
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

    //estop
    pub estop: gpio::Pin<'E', 6, Input>,

    //this enables motor power, will be used in E-Stop
    pub motor_relay: gpio::Pin<'E', 0, Output<PushPull>>,

    pub motor_pwm: pwm::Pwm<TIM1, 0, pwm::ComplementaryDisabled>,

    // these enable load the 2.2kΩ resistors,
    // 1 must be enabled in order for 2 to be enabled can be enabled but each are
    pub load_step_1: gpio::Pin<'E', 15, Output<PushPull>>,
    pub load_step_2: gpio::Pin<'E', 8, Output<PushPull>>,

    //UART
    pub rx: serial::Rx<pac::UART4>,
    pub tx: serial::Tx<pac::UART4>,

    //INPUTS
    //batch for AMT102-V | rotary encoder (sys rpm)
    pub encoder: Qei<TIM2>,

    pub adc_spi: crate::guv::adc::AdcSpi,
    pub adc_cs1: crate::guv::adc::Cs1,
    pub adc_cs2: crate::guv::adc::Cs2,
    pub adc_sync: crate::guv::adc::Sync,
    pub adc_rst: crate::guv::adc::Rst,
    pub adc_drdy: crate::guv::adc::Drdy,
}

pub fn setup(mut dp: pac::Peripherals) -> Board {
    defmt::info!("BOOT: entering setup");

    // 1. Power & Clocks
    defmt::info!("BOOT: constraining PWR");
    let pwr = dp.PWR.constrain();

    defmt::info!("BOOT: freezing PWR");
    let pwrcfg = pwr.freeze();

    defmt::info!("BOOT: constraining RCC");
    let rcc = dp.RCC.constrain();

    defmt::info!("BOOT: freezing RCC");
    // Modify this block in your setup function:
    let ccdr = rcc
        .sys_ck(64.MHz())
        .pll1_q_ck(64.MHz())
        // Ensure peripheral management is fully passed in
        .freeze(pwrcfg, &dp.SYSCFG);

    //DELETE THIS AFTER TEST
    let pll1_q = ccdr.clocks.pll1_q_ck().map(|rate| rate.raw()).unwrap_or(0); // Uses 0 if the clock is disabled

    defmt::info!(
        "BOOT: clocks ready: SYS={} Hz PLL1_Q={} Hz",
        ccdr.clocks.sys_ck().raw(),
        pll1_q
    );

    // 2. GPIO Split
    defmt::info!("BOOT: splitting GPIOA");
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

    defmt::info!("BOOT: splitting GPIOB");
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    defmt::info!("BOOT: splitting GPIOC");
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    defmt::info!("BOOT: splitting GPIOD");
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);

    defmt::info!("BOOT: splitting GPIOE");
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    // 3. Pin Config
    let mut ld1 = gpiob.pb0.into_push_pull_output();
    let mut ld2 = gpioe.pe1.into_push_pull_output();
    let mut ld3 = gpiob.pb14.into_push_pull_output();

    // PWM Setup (Pin D7[PWM])
    let pwm_pin = gpioe.pe9.into_alternate::<1>();
    // Consumes TIM1 token from ccdr
    let mut motor_pwm = dp
        .TIM1
        .pwm(pwm_pin, 20.kHz(), ccdr.peripheral.TIM1, &ccdr.clocks);

    motor_pwm.enable();
    motor_pwm.set_duty(motor_pwm.get_max_duty());
    defmt::info!("BOOT: PWM configured");
    //estop
    let estop = gpioe.pe6.into_pull_up_input();
    //relays
    let mut motor_relay = gpioe.pe0.into_push_pull_output();
    let mut load_step_1 = gpioe.pe15.into_push_pull_output();
    let mut load_step_2 = gpioe.pe8.into_push_pull_output();

    //a & b channel pair for rotary encoder
    let enc_pin_a = gpioa.pa0.into_alternate::<1>();
    let enc_pin_b = gpioa.pa1.into_alternate::<1>();
    // Call .qei() with ONLY the pins and the peripheral token
    let encoder = dp.TIM2.qei((enc_pin_a, enc_pin_b), ccdr.peripheral.TIM2);
    defmt::info!("BOOT: encoder configured");

    let sck = gpioa.pa5.into_alternate::<5>();
    let miso = gpioa.pa6.into_alternate::<5>();
    let mosi = gpioa.pa7.into_alternate::<5>();
    defmt::info!("BOOT: ADC pins configured");
    // CS1=PD8, CS2=PD9, DRDY=PD10(EXTI in), SYNC=PD11, RST=PD12
    // ─────────────────────────────────────────────────────────────────────────────
    let mut adc_cs1 = gpiod.pd3.into_push_pull_output();
    let mut adc_cs2 = gpiod.pd4.into_push_pull_output();
    adc_cs1.set_high(); // CS idle high (active low)
    adc_cs2.set_high();
    let adc_sync = gpiod.pd6.into_push_pull_output();
    let adc_rst = gpiod.pd7.into_push_pull_output();

    // pub type Cs1 = gpio::Pin<'D', 3, Output<PushPull>>;
    // pub type Cs2 = gpio::Pin<'D', 4, Output<PushPull>>;
    // pub type Drdy = gpio::Pin<'D', 5, Input>;
    // pub type Sync = gpio::Pin<'D', 6, Output<PushPull>>;
    // pub type Rst = gpio::Pin<'D', 7, Output<PushPull>>;
    let mut adc_drdy = gpiod.pd5.into_pull_up_input();
    adc_drdy.make_interrupt_source(&mut dp.SYSCFG);
    adc_drdy.trigger_on_edge(&mut dp.EXTI, stm32h7xx_hal::gpio::Edge::Falling);
    if crate::guv::adc::ADC_HARDWARE_ENABLED {
        adc_drdy.enable_interrupt(&mut dp.EXTI); // only arm when hardware present
    }
    //     // PD10 → EXTI15_10 line; the ISR in main binds EXTI15_10.
    defmt::info!("BOOT: DRDY EXTI configured");
    let adc_spi = dp.SPI1.spi(
        (sck, miso, mosi),
        stm32h7xx_hal::spi::MODE_1,
        8.MHz(),
        ccdr.peripheral.SPI1,
        &ccdr.clocks,
    );
    defmt::info!("BOOT: SPI1 configured");
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

    defmt::info!("BOOT: UART configured");
    // 4. Initial Safety State
    ld1.set_low();
    ld2.set_low();
    ld3.set_low();

    //we need to send our pwm signal into the pwm controller first before we enable power to the dc motor
    motor_relay.set_low();
    //no load should be applied at startup

    load_step_1.set_low(); // both open at boot — no load connected
    load_step_2.set_low();
    defmt::info!("BOOT: setup complete, returning Board");
    Board {
        ld1,
        ld2,
        ld3,
        estop,
        motor_relay,
        load_step_1,
        load_step_2,
        motor_pwm,
        encoder,
        tx,
        rx,
        clocks: ccdr.clocks,
        adc_spi,
        adc_cs1,
        adc_cs2,
        adc_sync,
        adc_rst,
        adc_drdy,
    }
}
