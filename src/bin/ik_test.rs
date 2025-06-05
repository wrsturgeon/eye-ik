#![no_std]
#![no_main]
#![feature(async_trait_bounds, impl_trait_in_assoc_type)]

use {
    defmt_rtt as _,
    embassy_executor::Spawner,
    embassy_rp::{
        bind_interrupts,
        peripherals::{UART1, USB},
        uart, usb,
    },
    embassy_time::{Duration, Ticker, Timer},
    eye_bot_inverse_kinematics::{ik, leg::Leg, pwm},
    panic_probe as _,
};

bind_interrupts!(struct Irqs {
    UART1_IRQ => uart::InterruptHandler<UART1>;
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

const MAIN_LOOP_PERIOD_MS: u16 = pwm::PULSE_PERIOD_MS;

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

    let (pwm0, pwm1) = pwm::init_slice(p.PWM_SLICE5, p.PIN_10, p.PIN_11).await;
    let (pwm2, _pwm3) = pwm::init_slice(p.PWM_SLICE6, p.PIN_12, p.PIN_13).await;

    let mut leg = match Leg::with_home_yaw(0.0, pwm0, pwm1, pwm2).await {
        Ok(ok) => ok,
        Err(e) => {
            let mut ticker = Ticker::every(Duration::from_secs(1));
            loop {
                log::error!("Couldn't initialize a leg: {e:?}");
                let () = ticker.next().await;
            }
        }
    };

    let mut counter: u16 = 0;
    let mut ticker = Ticker::every(Duration::from_millis(MAIN_LOOP_PERIOD_MS as _));
    loop {
        let foot_pos = ik::CartesianDisplacementFromEyeCenterLookingForward {
            x: 2.0 * libm::sinf(counter as f32 / 100.0)
                + 2.0
                + ik::LENGTH_CENTER_TO_YAW
                + ik::LENGTH_YAW_TO_HIP
                + ik::LENGTH_HIP_TO_KNEE,
            y: 2.0 * libm::cosf(counter as f32 / 100.0),
            z: 1.0 * libm::sinf(counter as f32 / 1_000.0) + 2.0 - ik::LENGTH_KNEE_TO_FOOT,
        };

        match leg.ik_to(foot_pos) {
            Ok(()) => {}
            Err(e) => log::error!("Leg inverse kinematics error: {e:?}"),
        }

        counter += MAIN_LOOP_PERIOD_MS;
        let () = ticker.next().await;
    }
}
