//use cortex_m::Peripherals;
use defmt::*;
use embassy_stm32::usart::Uart;
use {defmt_rtt as _, panic_probe as _};

// Declare async tasks
#[embassy_executor::task]
pub async fn uart_task(mut lpuart: Uart<'static, embassy_stm32::mode::Async>) {
    info!("UART started, type something...");
    match lpuart.write("UART started, type something...".as_bytes()).await {
        Ok(()) => info!("Message sent"),
        Err(e) => error!("Error sending message: {:?}", e),
    }

    let mut buffer = [0u8; 1];

    // Loop to read from UART and echo back
    loop {
        let _ = lpuart.read(&mut buffer).await ;
        let _ = lpuart.write(&buffer).await;
    }
}
