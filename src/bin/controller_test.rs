#![no_std]
#![no_main]
#![feature(async_trait_bounds, impl_trait_in_assoc_type)]

use {
    defmt_rtt as _,
    embassy_executor::Spawner,
    embassy_rp::{
        bind_interrupts,
        peripherals::{UART1, USB},
        pwm::SetDutyCycle as _,
        uart, usb,
    },
    embassy_time::{Duration, Ticker, Timer},
    eye_bot_inverse_kinematics::pwm,
    panic_probe as _,
};

bind_interrupts!(struct Irqs {
    UART1_IRQ => uart::InterruptHandler<UART1>;
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

const MAIN_LOOP_PERIOD_MS: u16 = 1; // pwm::PULSE_PERIOD_MS;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    {
        // USB background task:
        #[embassy_executor::task]
        pub async fn task(driver: usb::Driver<'static, USB>) {
            embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
        }
        let () = match spawner.spawn(task(usb::Driver::new(p.USB, Irqs))) {
            Ok(()) => defmt::info!("Spawned USB task"),
            Err(e) => {
                log::error!("Error spawning USB task");
                Timer::after(Duration::from_secs(1)).await;
                defmt::panic!("Error spawning USB task: {}", e);
            }
        };
    }

    let (_pwm0, _pwm1) = pwm::init_slice(p.PWM_SLICE5, p.PIN_10, p.PIN_11).await;
    let (_pwm2, mut pwm3) = pwm::init_slice(p.PWM_SLICE6, p.PIN_12, p.PIN_13).await;

    let pulse_center = pwm::pulse_center().await;
    let pulse_range_plus_minus = pwm::pulse_range_plus_minus().await;

    let mut counter: u8 = 0;
    let mut ticker = Ticker::every(Duration::from_millis(MAIN_LOOP_PERIOD_MS as _));
    loop {
        let theta = if counter > 192 { 0.1 } else { 0.0 };

        match pwm3.set_duty_cycle((pulse_center + pulse_range_plus_minus * theta) as _) {
            Ok(()) => {}
            Err(e) => log::error!("Couldn't set duty cycle: {e:?}"),
        }

        let () = ticker.next().await;
        counter = counter.wrapping_add(1);
    }
}
