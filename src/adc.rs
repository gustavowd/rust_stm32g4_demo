use embassy_stm32::peripherals::ADC2;
use embassy_stm32::adc::{Adc, AnyAdcChannel, SampleTime};
use {defmt_rtt as _, panic_probe as _};

// Declare async tasks
#[embassy_executor::task]
pub async fn adc_task(
    mut adc: Adc<'static, ADC2>, 
    mut adc_pin: AnyAdcChannel<'static, ADC2>
) {
    loop {
        // Na versão 0.6, o SampleTime mudou e é passado direto no método de leitura
        let measured = adc.blocking_read(&mut adc_pin, SampleTime::CYCLES247_5);
        
        defmt::info!("ADC Valor: {}", measured);
        
        // Evita travar a CPU em busy-waiting infinito na task
        embassy_time::Timer::after_millis(500).await;
    }
}