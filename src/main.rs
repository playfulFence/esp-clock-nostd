#![no_std]
#![no_main]

#[cfg(feature="esp32")]
use esp32_hal as hal;
#[cfg(feature="esp32s2")]
use esp32s2_hal as hal;
#[cfg(feature="esp32s3")]
use esp32s3_hal as hal;
#[cfg(feature="esp32c3")]
use esp32c3_hal as hal;

use hal::{
    adc::{AdcConfig, Attenuation, ADC, ADC2},
    clock::{ClockControl, CpuClock},
    peripherals::Peripherals,
    gpio::*,
    prelude::*,
    spi,
    timer::TimerGroup,
    systimer::SystemTimer,
    Rtc,
    IO,
    Delay,
    Rng
};

use embedded_io::blocking::*;
use embedded_svc::ipv4::Interface;
use embedded_svc::wifi::{AccessPointInfo, ClientConfiguration, Configuration, Wifi};

use esp_backtrace as _;
// use esp_println::logger::init_logger;
use esp_println::{print, println};
use esp_wifi::wifi::utils::create_network_interface;
use esp_wifi::wifi::{WifiError, WifiMode};
use esp_wifi::wifi_interface::WifiStack;
use esp_wifi::{current_millis, initialize};
use smoltcp::iface::SocketStorage;
use smoltcp::wire::Ipv4Address;

use core::mem::transmute;
use core::time::Duration;
use lexical_core;


// #[toml_cfg::toml_config]
// pub struct Config {
//     #[default("")]
//     wifi_ssid: &'static str,
//     #[default("")]
//     wifi_pass: &'static str,
// }

const NTP_VERSION: u8 = 0b00100011; // NTP version 4, mode 3 (client)
const NTP_MODE: u8 = 0b00000011;
const NTP_PACKET_SIZE: usize = 48;
const NTP_TIMESTAMP_DELTA: u64 = 2_208_988_800; // 70 years in seconds (since 01.01.1900)
const TIMESTAMP_LEN : usize = 10;
const UNIXTIME_LEN : usize = 8;


// 1

// Define a struct for the NTP request   LI |VN |mod|     stratum     |       poll      |     precision   | root delay | root dispersion | reference ID | reference timestamp | originate timestamp | receive timestamp | transmit timestamp |
                                        //00 100 011| 0 0 0 0 0 0 0 0 | 0 0 0 0 0 0 0 0 | 0 0 0 0 0 0 0 0 | _




type NtpRequest = [u8;NTP_PACKET_SIZE];


pub fn new_request(timestamp: u64) -> NtpRequest {
    let mut buf:[u8;48] = [0u8; 48];

    // Set Leap Indicator (LI), Protocol Version (VN), and Mode (3 = Client)
    buf[0] = 0b00_011_011;

    // Set Stratum (0 = unspecified)
    buf[1] = 0;

    // Set Poll Interval (4 = 16 seconds)
    buf[2] = 4;

    // Set Precision (-6 = 15.26 microseconds)
    buf[3] = 0xFA;

    // Set Root Delay
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;

    // Set Root Dispersion
    buf[8] = 0;
    buf[9] = 0;
    buf[10] = 0;
    buf[11] = 0;

    // Set Reference Identifier (unspecified)
    buf[12] = 0;
    buf[13] = 0;
    buf[14] = 0;
    buf[15] = 0;

    // Set Originate Timestamp to current time
    let secs = timestamp + 2_208_988_800;
    let frac = ((timestamp % 1_000_000_000) as f64 / 1_000_000_000.0) * ((2.0 as u32).pow(32) as f64);
    let frac = frac as u32;
    buf[16..24].copy_from_slice(&secs.to_be_bytes());
    buf[24..32].copy_from_slice(&frac.to_be_bytes());

    // Leave Transmit Timestamp and Receive Timestamp as 0

    buf

}


fn find_uxtime(response: &str) -> u64 {
    let response_len = response.len();

    for i in 0..(response_len - UNIXTIME_LEN + 1) {
        if &response[i..(i + UNIXTIME_LEN)] == "unixtime" {
            return lexical_core::parse(&response[i+UNIXTIME_LEN+2..i+UNIXTIME_LEN+2+UNIXTIME_LEN].as_bytes()).unwrap();
        }
    }

    0
}



#[entry]
fn main() -> ! {
    esp_wifi::init_heap();
    // let app_config = CONFIG;

    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    #[cfg(feature = "esp32c3")]
    let mut clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();
    #[cfg(any(feature = "esp32", feature = "esp32s3", feature = "esp32s2"))]
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();

    let timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;





let mut delay = Delay::new(&clocks);
    
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    println!("About to initialize the SPI LED driver ILI9341");
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, sockets) =
        create_network_interface(WifiMode::Sta, &mut socket_set_entries);
    let wifi_stack = WifiStack::new(iface, device, sockets, current_millis);

    initialize(timer, Rng::new(peripherals.RNG), &clocks).unwrap();

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: "EspressifSystems".into(),
        password: "Espressif32".into(),
        ..Default::default()
    });
    let res = controller.set_configuration(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    controller.start();
    println!("is wifi started: {:?}", controller.is_started());

    println!("Start Wifi Scan");
    let res: Result<(heapless::Vec<AccessPointInfo, 10>, usize), WifiError> = controller.scan_n();
    if let Ok((res, _count)) = res {
        for ap in res {
            println!("{:?}", ap);
        }
    }

    println!("{:?}", controller.get_capabilities());
    println!("wifi_connect {:?}", controller.connect());

    // wait to get connected
    println!("Wait to get connected");
    loop {
        let res = controller.is_connected();
        match res {
            Ok(connected) => {
                if connected {
                    break;
                }
            }
            Err(err) => {
                println!("{:?}", err);
                loop {}
            }
        }
    }
    println!("{:?}", controller.is_connected());

    // wait for getting an ip address
    println!("Wait to get an ip address");
    loop {
        wifi_stack.work();

        if wifi_stack.is_iface_up() {
            println!("got ip {:?}", wifi_stack.get_ip_info());
            break;
        }
    }

    println!("Start busy loop on main");

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);



    let mut request = [0u8; 48];
    request[0] = 0x1b; // LI, Version, Mode

    println!("your request is: {:?}", request);

    loop {
        println!("Making HTTP request");
        socket.work();

        socket
        .open(Ipv4Address::new(213, 188, 196, 246), 80)
        .unwrap();


        socket
            .write("GET /api/timezone/Europe/Prague HTTP/1.1\r\nHost: worldtimeapi.org\r\n\r\n".as_bytes())
            .unwrap();
        socket.flush().unwrap();

        let wait_end = current_millis() + 20 * 1000;
        loop {
            let mut buffer = [0u8; 512];
            if let Ok(len) = socket.read(&mut buffer) {
                let to_print = unsafe { core::str::from_utf8_unchecked(&buffer[..len]) };
                print!("{}", to_print);
            } else {
                break;
            }

            if current_millis() > wait_end {
                println!("Timeout");
                break;
            }
        }
        println!();

        socket.disconnect();

    let wait_end = current_millis() + 5 * 1000;
        while current_millis() < wait_end {
            socket.work();
        }
    }
}