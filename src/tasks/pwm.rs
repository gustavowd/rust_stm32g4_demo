use embassy_time::Timer;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use {defmt_rtt as _, panic_probe as _};


// Declare async tasks
#[embassy_executor::task]
pub async fn pwm_task(mut pwm: SimplePwm<'static, embassy_stm32::peripherals::TIM1>) {
    let mut ch1 = pwm.ch1();
    ch1.enable();

    // Loop to read from UART and echo back
    loop {
        ch1.set_duty_cycle_fully_off();
        Timer::after_millis(300).await;
        ch1.set_duty_cycle_fraction(1, 4);
        Timer::after_millis(300).await;
        ch1.set_duty_cycle_fraction(1, 2);
        Timer::after_millis(300).await;
        ch1.set_duty_cycle(ch1.max_duty_cycle() - 1);
        Timer::after_millis(300).await;
    }
}