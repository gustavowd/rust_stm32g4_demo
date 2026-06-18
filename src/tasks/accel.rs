use defmt::*;
use embassy_stm32::i2c::I2c;
use adxl345_async::{Adxl345Async, Range, DataRate, I2cBus};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

// Declare async tasks
#[embassy_executor::task]
pub async fn accel_task(mut accel: Adxl345Async<I2cBus<I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::mode::Master>>>) {
    let _ = accel.set_range(Range::G2).await;
    let _ = accel.set_data_rate(DataRate::Rate100Hz).await;
    loop {
        if let Ok((x, y, z)) = accel.get_accel().await{
            info!("ADXL345 Accel Raw: x={}, y={}, z={}", x, y, z);
        }
        Timer::after_millis(1000).await;
    }
}