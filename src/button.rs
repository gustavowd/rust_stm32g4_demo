use defmt::*;
use embassy_stm32::exti::ExtiInput;
use {defmt_rtt as _, panic_probe as _};

// Declare async tasks
#[embassy_executor::task]
pub async fn button_task(mut button: ExtiInput<'static, embassy_stm32::mode::Async>) {
    info!("Press the USER button...");

    loop {
        button.wait_for_rising_edge().await;
        info!("Pressed!");
        button.wait_for_falling_edge().await;
        info!("Released!");
    }
}