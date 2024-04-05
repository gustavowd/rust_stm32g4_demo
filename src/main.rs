#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::peripherals::ADC1;
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::adc::{self, Adc, SampleTime};
use embassy_time::{Delay, Timer};
use {defmt_rtt as _, panic_probe as _};

// Declare async tasks
#[embassy_executor::task]
async fn adc_task(mut adc: adc::Adc<'static, ADC1>, mut adc_pin: embassy_stm32::peripherals::PA1) {
    
    adc.set_sample_time(SampleTime::Cycles32_5);

    loop {
        let measured = adc.read(&mut adc_pin);
        info!("measured: {}", measured);
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;

        config.rcc.pll = Some(Pll {
            source: PllSource::HSE(Hertz::mhz(24)),
            prediv_m: PllM::DIV6,
            mul_n: PllN::MUL85,
            div_p: None,
            div_q: None,
            // Main system clock at 170 MHz
            div_r: Some(PllR::DIV2),
        });

        config.rcc.mux = ClockSrc::PLL;

        config.rcc.adc12_clock_source = AdcClockSource::SYS;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
    }

    let p = embassy_stm32::init(config);
    info!("Hello World!");
    //defmt::println!("Hello, world!");

    let adc = Adc::new(p.ADC1, &mut Delay);
    let adc_pin = p.PA1;

    // Spawned tasks run in the background, concurrently.
    spawner.spawn(adc_task(adc, adc_pin)).unwrap();

    let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    loop {
        info!("high");
        led.set_high();
        Timer::after_millis(500).await;

        info!("low");
        led.set_low();
        Timer::after_millis(500).await;
    }
}


/*
// Some panic handler needs to be included. This one halts the processor on panic.
use cortex_m_rt::entry;
use rtt_target::{rtt_init_print, rprintln};

use hal::prelude::*;
use hal::stm32;
use stm32g4xx_hal as hal;

#[link_section = ".ram2bss"]
static mut TESTE: i32 = 10;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Olá mundo!");
    unsafe {
        rprintln!("Teste de variável na memória SRAM2 {}", TESTE);
    }

    let teste2 = 10;
    rprintln!("Teste de variável na memória SRAM2 {}", teste2);

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut led = gpioa.pa5.into_push_pull_output();

    let core_periphs=cortex_m::Peripherals::take().unwrap();
    let clocks = rcc.clocks.sys_clk;

    // Create a delay abstraction based on SysTick
    let mut delay = hal::delay::Delay::new(core_periphs.SYST, clocks.0);
    loop{
        rprintln!("High");
        led.set_high().unwrap();
        delay.delay_ms(500_u32);

        rprintln!("Low");
        led.set_low().unwrap();
        delay.delay_ms(500_u32);
    }
}
*/
