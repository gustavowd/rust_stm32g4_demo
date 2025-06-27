#![no_std]
#![no_main]

//use cortex_m::Peripherals;
use cortex_m_rt::pre_init;
use core::arch::asm;
use defmt::*;
use embassy_executor::Spawner;
//use embassy_stm32::pac::metadata::Peripheral;
use embassy_stm32::peripherals::ADC1;
use embassy_stm32::time::Hertz;
//use embassy_stm32::rcc::low_level::RccPeripheral;
//use embassy_stm32::timer::low_level::GeneralPurpose16bitInstance;
use embassy_stm32::Config;
use embassy_stm32::adc::{self, Adc, AdcChannel, AnyAdcChannel, SampleTime};
use embassy_stm32::gpio::{Output, Pull, Level, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::bind_interrupts;
use embassy_stm32::usart::{self, Uart};
//use embassy_stm32::timer::pwm_input::PwmInput;
//use embassy_stm32::time::hz;
//use embassy_stm32::timer::CountingMode;
use embassy_stm32::exti::ExtiInput;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

// Declare async tasks
#[embassy_executor::task]
async fn adc_task(mut adc: adc::Adc<'static, ADC1>, mut adc_pin: AnyAdcChannel<ADC1>) {
    
    adc.set_sample_time(SampleTime::CYCLES47_5);

    loop {
        let measured = adc.blocking_read(&mut adc_pin);
        info!("measured: {}", measured);
        Timer::after_millis(500).await;
    }
}

// Declare async tasks
#[embassy_executor::task]
async fn button_task(mut button: ExtiInput<'static>) {
    info!("Press the USER button...");

    loop {
        button.wait_for_rising_edge().await;
        info!("Pressed!");
        button.wait_for_falling_edge().await;
        info!("Released!");
    }
}

// Declare async tasks
#[embassy_executor::task]
async fn uart_task(mut lpuart: Uart<'static, embassy_stm32::mode::Async>) {
    info!("UART started, type something...");
    lpuart.write("UART started, type something...".as_bytes()).await.unwrap();

    let mut buffer = [0u8; 1];

    // Loop to read from UART and echo back
    loop {
        lpuart.read(&mut buffer).await.unwrap();
        lpuart.write(&buffer).await.unwrap();
    }
}

//bind_interrupts!(struct Irqs {
//    TIM2 => timer::CaptureCompareInterruptHandler<peripherals::TIM2>;
//});

bind_interrupts!(struct Irqs {
    LPUART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::LPUART1>;
});

//#[link_section = ".ram2bss"]
#[link_section = ".ccmram"]
static mut TESTE: i32 = 60;

#[link_section = ".data2"]
static mut TESTE2: i32 = 70;

#[pre_init]
unsafe fn before_main() {
    unsafe {
        asm!{
            "ldr r0, =__sccmdata
            ldr r1, =__eccmdata
            ldr r2, =__siccmdata
            0:
            cmp r1, r0
            beq 1f
            ldm r2!, {{r3}}
            stm r0!, {{r3}}
            b 0b
            1:"
        }

        asm!{
            "ldr r0, =__sdata2
            ldr r1, =__edata2
            ldr r2, =__sidata2
            2:
            cmp r1, r0
            beq 3f
            ldm r2!, {{r3}}
            stm r0!, {{r3}}
            b 2b
            3:"
        }
    }
}

#[interrupt]
unsafe fn TIM3(){
    
    // reset interrupt flag
    //unsafe {
    //    let pin = embassy_stm32::peripherals::PA5::steal();
    //    let mut pin = Output::new(pin, Level::High, Speed::Low);
    //    pin.set_high();
    //}
    //pac::TIM3.sr().modify(|r| r.set_uif(false));
    info!("interrupt happens: tim20");
}



#[embassy_executor::main]
async fn main(spawner: Spawner) {
    //unsafe {
        //TESTE = 10;
        //TESTE2 = 20;
    //}
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(24_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV6,
            mul: PllMul::MUL85,
            divp: None,
            divq: None,
            // Main system clock at 170 MHz
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_R;
        //config.rcc.mux = ClockSrc::PLL;

        //config.rcc.adc12_clock_source = AdcClockSource::SYS;
        config.rcc.mux.adc12sel = mux::Adcsel::SYS;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
    }

    let p: embassy_stm32::Peripherals = embassy_stm32::init(config);

    info!("Hello World!");
    unsafe {
        println!("Teste de variável na memória CCMRAM {}", TESTE);
        println!("Teste de variável na memória SRAM2 {}", TESTE2);
    }
    //defmt::println!("Hello, world!");

    let button = ExtiInput::new(p.PC13, p.EXTI13, Pull::Down);

    //let adc = Adc::new(p.ADC1, &mut Delay);
    //let adc_pin = p.PA1;

    let adc = Adc::new(p.ADC1);

    // Spawned tasks run in the background, concurrently.
    spawner.spawn(adc_task(adc, p.PA1.degrade_adc())).unwrap();
    spawner.spawn(button_task(button)).unwrap();

    let mut config = usart::Config::default();
    config.baudrate = 115_200;
    let lpusart = Uart::new(p.LPUART1, p.PA3, p.PA2, Irqs, p.DMA1_CH1, p.DMA1_CH2, config).unwrap();
    spawner.spawn(uart_task(lpusart)).unwrap();

    //let mut pwm_input = PwmInput::new(p.TIM2, p.PA0, Pull::None, khz(10));
    //pwm_input.enable();

    let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    loop {
        //info!("high");
        led.set_high();
        Timer::after_millis(500).await;

        //info!("low");
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
