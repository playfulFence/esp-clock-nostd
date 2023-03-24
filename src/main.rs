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

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    att::Uuid,
    Ble, HciConnector,
};

use heapless::spsc::Queue as SimpleQueue;

use mipidsi::Orientation;

extern crate cfg_if;
use cfg_if::cfg_if;

use display_interface_spi::SPIInterfaceNoCS;

// use core::f32::consts::PI;
// use libm::{sin, cos};

use embedded_graphics::{
    prelude::RgbColor,
    mono_font::{
        ascii::FONT_10X20,
        MonoTextStyleBuilder,
        MonoTextStyle,
    },
    prelude::*,
    text::{Alignment, Text},
    Drawable,
    pixelcolor::*,
    primitives::{Circle, PrimitiveStyleBuilder, PrimitiveStyle, Rectangle},
    text::*,
    image::Image,
    geometry::*,
    draw_target::DrawTarget,
};

//use embedded_hal;

use profont::{PROFONT_24_POINT, PROFONT_18_POINT};

use embedded_io::blocking::*;
use embedded_svc::{ipv4::Interface, wifi};
use embedded_svc::wifi::{AccessPointInfo, ClientConfiguration, Configuration, Wifi};

use esp_wifi::initialize;
use esp_wifi::wifi::{utils::create_network_interface, WifiError};
use esp_wifi::{
    ble::controller::BleConnector, current_millis, wifi::WifiMode, wifi_interface::{WifiStack, UdpSocket},
};
use embedded_svc::ipv4::{SocketAddr,ToSocketAddrs};

use smoltcp::{iface::SocketStorage, wire::Ipv4Address};

// getting current time
use sntpc::{Error, NtpContext, NtpTimestampGenerator, NtpUdpSocket, Result};
use core::time::Duration;
use core::mem::transmute;

use esp_println::println;
use esp_backtrace as _;

// /* Debouncing algorythm */
// pub enum Event {
//     Pressed,
//     Released,
//     Nothing,
// }
// pub struct Button<T> {
//     button: T,
//     pressed: bool,
// }

// impl<T: ::embedded_hal::digital::v2::InputPin<Error = core::convert::Infallible>> Button<T> {
//     pub fn new(button: T) -> Self {
//         Button {
//             button,
//             pressed: true,
//         }
//     }
//     pub fn check(&mut self){
//         self.pressed = !self.button.is_low().unwrap();
//     }

//     pub fn poll(&mut self, delay :&mut Delay) -> Event {
//         let pressed_now = !self.button.is_low().unwrap();
//         if !self.pressed  &&  pressed_now
//         {
//             delay.delay_ms(30 as u32);
//             self.check();
//             if !self.button.is_low().unwrap() {
//                 Event::Pressed
//             }
//             else {
//                 Event::Nothing
//             }
//         }
//         else if self.pressed && !pressed_now{
//             delay.delay_ms(30 as u32);
//             self.check();
//             if self.button.is_low().unwrap()
//             {
//                 Event::Released
//             }
//             else {
//                 Event::Nothing
//             }
//         }
//         else{
//             Event::Nothing
//         }
        
//     }
// }


#[toml_cfg::toml_config]
pub struct Config {
    #[default("")]
    wifi_ssid: &'static str,
    #[default("")]
    wifi_pass: &'static str,
}

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
            li_vn_mode: NTP_VERSION,
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


// struct UdpSocketWrapper<'a>(UdpSocket<'a, 'a>);

// impl NtpUdpSocket for UdpSocketWrapper<'_> {
//     fn send_to<T: ToSocketAddrs>(
//         &self,
//         buf: &[u8],
//         addr: T,
//     ) -> Result<usize> {
//         match self.0.send_to(buf, addr) {
//             Ok(usize) => Ok(usize),
//             Err(_) => Err(Error::Network),
//         }
//     }

//     fn recv_from(&self, buf: &mut [u8]) -> Result<(usize, SocketAddr)> {
//         match self.0.recv_from(buf) {
//             Ok((size, addr)) => Ok((size, addr)),
//             Err(_) => Err(Error::Network),
//         }
//     }
// }


#[entry]
fn main() -> ! {

    esp_wifi::init_heap();
    let app_config = CONFIG;

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
        ssid: app_config.wifi_ssid.into(),
        password: app_config.wifi_pass.into(),
        ..Default::default()
    });

    let res = controller.set_configuration(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    controller.start().unwrap();
    println!("is wifi started: {:?}", controller.is_started());

    println!("Start Wifi Scan");
    let res = controller.scan_n::<10>();
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
                    println!("Connected!");
                    break;
                }
            }
            Err(err) => {
                println!("{:?}", err);
                loop {}
            }
        }
    }

    unsafe{

      // wait for getting an ip address
      println!("Wait to get an ip address");
      loop {
            delay.delay_ms(300 as u32);
            println!("work..");
            wifi_stack.work();
    
            if wifi_stack.is_iface_up() {
                println!("iface is up");
                println!("got ip {:?}", wifi_stack.get_ip_info());
                break;
            }
      }
    
        println!("{:?}", controller.is_connected());


        println!("Getting socket from wifi stack...");
        let mut rx_buffer = [0u8; 1500];
        let mut tx_buffer = [0u8; 1500];

        let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);
        println!("Socket received, opening...");
        
        loop
        {
            socket.work();
            println!("Socket works!");

            socket.open(Ipv4Address::new(192, 168, 32, 3), 53).unwrap();

            println!("Socket opened");

            socket
                .write(&build_ntp_request())
                .unwrap();

            socket.flush().unwrap();

            let wait_end = current_millis() + 20 * 1000;
            loop {
                let mut buffer = [0u8; 512];
                if let Ok(len) = socket.read(&mut buffer) {
                    let to_print = unsafe { core::str::from_utf8_unchecked(&buffer[..len]) };
                    println!("{}", to_print);
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

    // let sock_wrapper = UdpSocketWrapper(socket);

    // let mut ntp = NtpContext::new(NtpTimestampGenerator::new());

    //let result = sntpc::get_time("time.google.com:123", socket, ntp);

    // match result {
    //     Ok(time) => {
    //         println!("Got time: {}.{}", time.sec(), time.sec_fraction());
    //     }
    //     Err(err) => println!("Err: {:?}", err),
    //  }
    
    // /* Set corresponding pins */
    // let mosi = io.pins.gpio7;
    // let cs = io.pins.gpio2;
    // let rst = io.pins.gpio10;
    // let dc = io.pins.gpio3;
    // let sck = io.pins.gpio6;
    // let miso = io.pins.gpio9;
    // let backlight = io.pins.gpio4;

    /* Then set backlight (set_low() - display lights up when signal is in 0, set_high() - opposite case(for example.)) */
    // let mut backlight = backlight.into_push_pull_output();
    //backlight.set_low().unwrap();

    /* Configure SPI */
    // let spi = spi::Spi::new(
    //     peripherals.SPI2,
    //     sck,
    //     mosi,
    //     miso,
    //     cs,
    //     80u32.MHz(),
    //     spi::SpiMode::Mode0,
    //     &mut system.peripheral_clock_control,
    //     &mut clocks,
    // );

    // let di = SPIInterfaceNoCS::new(spi, dc.into_push_pull_output());
    // let reset = rst.into_push_pull_output();
    // let mut delay = Delay::new(&clocks);


    // let mut display = mipidsi::Builder::ili9341_rgb565(di)
    //     .with_display_size(240 as u16, 320 as u16)
    //     .with_framebuffer_size(240 as u16, 320 as u16)
    //     .with_orientation(Orientation::LandscapeInverted(true))
    //     .init(&mut delay, Some(reset))
    //     .unwrap();
        
    // println!("Initialized");

    // display.clear(Rgb565::WHITE);

     
    // let mut button_up = Button::new(io.pins.gpio0.into_pull_up_input());
    // let mut button_down  = Button::new(io.pins.gpio1.into_pull_up_input());
    // let mut button_ok = Button::new(io.pins.gpio8.into_pull_up_input());

    loop {

    }
}