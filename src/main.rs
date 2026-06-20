#![no_std]
#![no_main]

use core::arch::asm;
use cortex_m_rt::pre_init;

use adxl345_async::{Adxl345Async, Address, I2cBus};
use defmt::*;
use {defmt_rtt as _, panic_probe as _};
use embassy_executor::{Spawner, InterruptExecutor};
use embassy_stm32::{bind_interrupts, interrupt, peripherals};
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::adc::{Adc, AdcChannel, SampleTime};
use embassy_stm32::Config;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, OutputType, Pull, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::{Hertz, khz};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart::{self, Uart};
use embassy_time::Timer;
use embedded_hal_bus::spi::{ExclusiveDevice};
use embedded_sdmmc::{VolumeManager, TimeSource, Timestamp};


mod tasks;

#[link_section = ".ccmram"]
static TESTE: i32 = 60;

#[link_section = ".data2"]
static TESTE2: i32 = 70;

const ADDRESS: u8 = 0x53;
const WHOAMI: u8 = 0;

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

static INT_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

// 2. Na exceção PendSV, nós chamamos o 'run' do executor.
// Como ele roda dentro da interrupção, ele herda a prioridade dela automaticamente!
#[interrupt]
unsafe fn EXTI0() {
    unsafe {
        // O método 'poll' ou 'run' faz o executor processar as tasks da fila dele
        INT_EXECUTOR.on_interrupt();
    }
}
bind_interrupts!(struct Irqs {
    LPUART1 => embassy_stm32::usart::InterruptHandler<peripherals::LPUART1>;
    DMA1_CHANNEL1 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH2>;
    
    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<peripherals::I2C1>;
    DMA1_CHANNEL3 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH3>;
    DMA1_CHANNEL4 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH4>;

    EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
    //TIM2 => timer::CaptureCompareInterruptHandler<peripherals::TIM2>;

    DMA1_CHANNEL5 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH5>;
    DMA1_CHANNEL6 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH6>;
});

struct DummyTimeSource;

impl TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 56,     // 1970 + 56 = 2026
            zero_indexed_month: 5,   // Junho (0 a 11)
            zero_indexed_day: 17,    // Dia 18 (0 a 30)
            hours: 13,
            minutes: 45,
            seconds: 0,
        }
    }
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
        config.rcc.mux.adc12sel = mux::Adcsel::SYS;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
    }

    let mut p: embassy_stm32::Peripherals = embassy_stm32::init(config);

    info!("Hello World!");
    //unsafe {
        println!("Teste de variável na memória CCMRAM {}", TESTE);
        println!("Teste de variável na memória SRAM2 {}", TESTE2);
    //}

    // --- INICIALIZAÇÃO DO EXECUTOR ---
    // O start() vincula o executor ao token da interrupção e limpa o estado inicial
    let irq = embassy_stm32::interrupt::EXTI0;
    irq.set_priority(interrupt::Priority::P15);
    let int_spawner = INT_EXECUTOR.start(irq);

    let button = ExtiInput::new(p.PC13, p.EXTI13, Pull::Down, Irqs);

    let mut adc = Adc::new(p.ADC2, Default::default());

    let mut adc_temp = Adc::new(p.ADC1, Default::default());
    let mut temperature = adc_temp.enable_temperature();

    let measured = adc.blocking_read(&mut p.PA4, SampleTime::CYCLES247_5); // Note a mudança no ciclo também!
    info!("measured: {}", measured);

    // (Certifique-se de que o objeto do sensor de temperatura interna do chip se chama p.ADC_TEMP ou similar no G4)
    let temp = adc_temp.blocking_read(&mut temperature, SampleTime::CYCLES247_5);
    info!("measured: {}", temp);

    // 1. Cria o canal genérico a partir do pino PA4 configurando o SampleTime
    let adc_channel = p.PA4.degrade_adc();

    // Spawned tasks run in the background, concurrently.
    int_spawner.spawn(unwrap!(tasks::adc::adc_task(adc, adc_channel)));

    int_spawner.spawn(unwrap!(tasks::button::button_task(button)));

    let mut config = usart::Config::default();
    config.baudrate = 115_200;
    let lpusart = Uart::new(p.LPUART1, p.PA3, p.PA2,p.DMA1_CH1, p.DMA1_CH2, Irqs, config).unwrap();
    int_spawner.spawn(unwrap!(tasks::uart::uart_task(lpusart)));

    let ch1_pin = PwmPin::new(p.PC0, OutputType::PushPull);
    let pwm = SimplePwm::new(p.TIM1, Some(ch1_pin), None, None, None, khz(10), Default::default());
    //let mut ch1: embassy_stm32::timer::simple_pwm::SimplePwmChannel<'_, embassy_stm32::peripherals::TIM1> = pwm.ch1();
    //ch1.enable();

    int_spawner.spawn(unwrap!(tasks::pwm::pwm_task(pwm)));

    //let mut i2c = I2c::new_blocking(p.I2C1, p.PB8, p.PB9, Hertz(100_000), i2c::Config::default());
    //println!("{:?}", p.PB8.af_num());

    let mut config: i2c::Config = Default::default();
    config.scl_pullup = true;
    config.sda_pullup = true;
    config.frequency = Hertz(100_000); // 100kHz I2C speed
    config.timeout = embassy_time::Duration::from_millis(1000);

    let mut i2c = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        p.DMA1_CH3,
        p.DMA1_CH4,
        Irqs,
        config,
    );


    //let mut i2c_cs = Output::new(p.PC9, Level::Low, Speed::Low);
    //Timer::after_millis(50).await;

    let mut data = [0u8; 1];
    //i2c_cs.set_high();
    //Timer::after_millis(5).await;
    match i2c.write_read(ADDRESS, &[WHOAMI], &mut data).await {
        Ok(()) => {
            if data[0] == 0xE5 {
                info!("ADXL345 found!");
            }else{
                info!("Whoami: {}", data[0]);
            }
        },
        Err(e) => error!("I2c Error: {:?}", e),
    }
    //i2c_cs.set_low();

    let bus = I2cBus::new(i2c, Some(Address::SECONDARY));
    let mut accel = Adxl345Async::new(bus);
    let _ = accel.setup().await;
    match accel.get_accel_raw().await {
        Ok((x, y, z)) => {
            info!("ADXL345 Accel Raw: x={}, y={}, z={}", x, y, z);
        },
        Err(e) => error!("Error reading accel: {:?}", e),
    }

    int_spawner.spawn(unwrap!(tasks::accel::accel_task(accel)));

    //  Configurar o SPI (Ajuste os pinos para o G474)
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(400_000); // Começa lento para inicialização do SD
    let spi = Spi::new(p.SPI1, p.PB3, p.PA7, p.PA6, p.DMA1_CH5, p.DMA1_CH6, Irqs, spi_config);

    // Configurar o Chip Select (CS) manual
    let cs = Output::new(p.PB6, Level::High, Speed::VeryHigh);

    let delay = embassy_time::Delay;

    let spi_device;
    let sdcard;
    match ExclusiveDevice::new(spi, cs, embassy_time::Delay){
        Ok(device) => {
            spi_device = device;
            sdcard = embedded_sdmmc::SdCard::new(spi_device, delay);
            // Get the card size (this also triggers card initialisation because it's not been done yet)
            info!("Card size is {} bytes", sdcard.num_bytes().unwrap());

            sdcard.spi(|dev| {
                // Acessamos o SPI bruto do Embassy de dentro do ExclusiveDevice
                let spi_tmp = dev.bus_mut();
                
                // Aplicamos a nova configuração de clock
                let mut spi_config = spi_tmp.get_current_config();
                spi_config.frequency = Hertz(25_000_000); 
                let _ = spi_tmp.set_config(&spi_config);
            });

            let volume_mgr = VolumeManager::new(sdcard, DummyTimeSource);
            spawner.spawn(unwrap!(tasks::sdcard::sd_task(volume_mgr)));
        },
        Err(e) => {
            error!("Failed to create SPI device: {:?}", e);
        }
    }


    let led = Output::new(p.PA5, Level::High, Speed::Low);
    int_spawner.spawn(unwrap!(tasks::led::adc_task(led)));

    loop {
        Timer::after_millis(1000).await;
    }
}


//use embassy_stm32::timer::pwm_input::PwmInput;
//use embassy_stm32::time::hz;
//use embassy_stm32::timer::CountingMode;
//use embassy_stm32::rcc::low_level::RccPeripheral;
//use embassy_stm32::timer::low_level::GeneralPurpose16bitInstance;
//use embassy_stm32::pac::metadata::Peripheral;

/*
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
     */