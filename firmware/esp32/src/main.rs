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
extern crate alloc;

use embassy_executor::Spawner;
use embassy_net::{tcp::TcpSocket, IpListenEndpoint, Runner, Stack, StackResources};
use embassy_time::{Duration, Timer};
use embedded_io::Read;
use embedded_io::Write;
use embedded_io_async::Write as AsyncWrite;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output},
    rng::Rng,
    timer::timg::TimerGroup,
    uart::{Config, Uart},
};
use esp_println::println;
use esp_wifi::{
    init,
    wifi::{
        ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiStaDevice,
        WifiState,
    },
    EspWifiController,
};

const SSID: &str = env!("WIFI_SSID");
const PASS: &str = env!("WIFI_PASS");
const PORT: u16 = {
    let s = env!("TCP_PORT").as_bytes();
    let mut val: u16 = 0;
    let mut i = 0;
    while i < s.len() {
        val = val * 10 + (s[i] - b'0') as u16;
        i += 1;
    }
    val
};

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(72 * 1024);

    let mut led = Output::new(peripherals.GPIO2, Level::Low);

    let mut uart1 = Uart::new(peripherals.UART1, Config::default())
        .unwrap()
        .with_tx(peripherals.GPIO17)
        .with_rx(peripherals.GPIO16);

    println!("ESP32 Booted. Waiting for STM32 handshake...");

    // --- HANDSHAKE ---
    let mut buf = [0u8; 1];
    let mut window = [0u8; 5];
    loop {
        if uart1.read(&mut buf).is_ok() {
            window[0] = window[1];
            window[1] = window[2];
            window[2] = window[3];
            window[3] = window[4];
            window[4] = buf[0];
            if &window == b"HELLO" {
                uart1.write_all(b"OK\r\n").ok();
                led.set_high();
                println!("Handshake complete — STM32 is GO");
                break;
            }
        }
    }

    // --- WIFI INIT ---
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut rng = Rng::new(peripherals.RNG);

    let init = &*mk_static!(
        EspWifiController<'static>,
        init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap()
    );

    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(init, peripherals.WIFI, WifiStaDevice).unwrap();

    let timg1 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timg1.timer0);

    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        embassy_net::Config::dhcpv4(Default::default()),
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    spawner.spawn(wifi_task(controller)).ok();
    spawner.spawn(net_task(runner)).ok();
    spawner.spawn(tcp_task(stack, uart1, led)).ok();

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

#[embassy_executor::task]
async fn wifi_task(mut controller: WifiController<'static>) {
    loop {
        match esp_wifi::wifi::wifi_state() {
            WifiState::StaConnected => {
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await;
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: SSID.try_into().unwrap(),
                password: PASS.try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            controller.start_async().await.unwrap();
            println!("WiFi started!");
        }
        println!("Connecting to {}...", SSID);
        match controller.connect_async().await {
            Ok(_) => println!("WiFi connected!"),
            Err(e) => {
                println!("Connect failed: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static, WifiStaDevice>>) {
    runner.run().await
}
#[embassy_executor::task]
async fn tcp_task(
    stack: Stack<'static>,
    mut uart: Uart<'static, esp_hal::Blocking>,
    mut led: Output<'static>,
) {
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
    println!("Waiting for DHCP...");
    loop {
        if let Some(config) = stack.config_v4() {
            println!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    let mut rx_buf = [0u8; 1024];
    let mut tx_buf = [0u8; 1024];
    let mut uart_buf = [0u8; 1];
    let mut cobs_buf = [0u8; 128];
    let mut cobs_len = 0usize;

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        socket.set_timeout(Some(Duration::from_secs(30)));

        println!("Listening on port {}...", PORT);
        if socket
            .accept(IpListenEndpoint {
                addr: None,
                port: PORT,
            })
            .await
            .is_err()
        {
            continue;
        }
        println!("Client connected!");
        led.set_high();

        'conn: loop {
            // UART -> TCP: drain ALL available bytes, not one per iteration
            let mut chunk = [0u8; 64];
            match uart.read(&mut chunk) {
                Ok(n) if n > 0 => {
                    led.toggle();
                    for &b in &chunk[..n] {
                        if b == 0x00 {
                            if cobs_len > 0 {
                                if socket.write_all(&cobs_buf[..cobs_len]).await.is_err() {
                                    break 'conn;
                                }
                                if socket.write_all(&[0x00]).await.is_err() {
                                    break 'conn;
                                }
                                cobs_len = 0;
                            }
                        } else if cobs_len < cobs_buf.len() {
                            cobs_buf[cobs_len] = b;
                            cobs_len += 1;
                        } else {
                            cobs_len = 0; // overflow guard
                        }
                    }
                }
                _ => {}
            }

            // TCP -> UART (unchanged)
            let mut cmd_buf = [0u8; 32];
            match embassy_futures::select::select(
                socket.read(&mut cmd_buf),
                Timer::after(Duration::from_millis(0)),
            )
            .await
            {
                embassy_futures::select::Either::First(Ok(0)) => break 'conn,
                embassy_futures::select::Either::First(Ok(n)) => {
                    uart.write_all(&cmd_buf[..n]).ok();
                    println!("CMD -> STM32: {:?}", &cmd_buf[..n]);
                }
                _ => {}
            }
        }

        println!("Client disconnected.");
        cobs_len = 0; // reset on disconnect
        led.set_low();
    }
}
