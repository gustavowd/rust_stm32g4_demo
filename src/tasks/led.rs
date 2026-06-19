// Declare async tasks
#[embassy_executor::task]
pub async fn adc_task(mut led: embassy_stm32::gpio::Output<'static>) {
    loop {
        //info!("high");
        led.set_high();
        embassy_time::Timer::after_millis(500).await;

        //info!("low");
        led.set_low();
        embassy_time::Timer::after_millis(500).await;
    }
}