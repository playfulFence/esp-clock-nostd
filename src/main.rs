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


// Define a struct for the NTP request packet
#[repr(C)]
pub struct NtpRequest {
    li_vn_mode: u8,
    stratum: u8,
    poll: u8,
    precision: u8,
    root_delay: u32,
    root_dispersion: u32,
    reference_id: [u8; 4],
    reference_timestamp: u64,
    originate_timestamp: u64,
    receive_timestamp: u64,
    transmit_timestamp: u64,
}

// Implementation of NtpRequest
impl NtpRequest {
    pub fn new() -> NtpRequest {
        NtpRequest {
            li_vn_mode: NTP_VERSION, // 3 | 4 << 3 
            stratum: 0,
            poll: 0,
            precision: 0,
            root_delay: 0,
            root_dispersion: 0,
            reference_id: [0; 4],
            reference_timestamp: 0,
            originate_timestamp: 0,
            receive_timestamp: 0,
            transmit_timestamp: 0,
        }
    }
}

// Function to convert a u64 timestamp to NTP format
fn to_ntp_time(timestamp: u64) -> u64 {
    let duration = Duration::from_secs(timestamp + NTP_TIMESTAMP_DELTA);
    let seconds = duration.as_secs() as u64;
    let fraction = ((duration.subsec_nanos() as u64) << 32) / 1_000_000_000;
    (seconds << 32) | fraction
}

// Function to assemble an NTP request packet
pub fn build_ntp_request() -> [u8; NTP_PACKET_SIZE] {
    let mut packet = [0u8; NTP_PACKET_SIZE];
    let mut request = NtpRequest::new();

    request.transmit_timestamp = to_ntp_time(0); // Set transmit timestamp to 0
    request.originate_timestamp = to_ntp_time(0); // Set originate timestamp to 0

    unsafe {
        // Convert NtpRequest struct to byte array
        let request_bytes = transmute::<NtpRequest, [u8; NTP_PACKET_SIZE]>(request);

        // Copy byte array to packet buffer
        packet.copy_from_slice(&request_bytes);
    }

    packet
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

    // let mut request:[u8; NTP_PACKET_SIZE] = build_ntp_request();

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
            .write("GET /api/timezone/Europe/London HTTP/1.1\r\nHost: worldtimeapi.org\r\n\r\n".as_bytes())
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