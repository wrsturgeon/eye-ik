#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use {
    defmt_rtt as _,
    embassy_executor::Spawner,
    embassy_rp::{
        bind_interrupts,
        peripherals::{UART1, USB},
        pwm::{self, Pwm},
        uart, usb,
    },
    embassy_time::{Duration, Ticker, Timer},
    fixed::{FixedU16, FixedU32, traits::LosslessTryFrom},
    panic_probe as _,
};

bind_interrupts!(struct Irqs {
    UART1_IRQ => uart::InterruptHandler<UART1>;
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

type Radix = fixed::types::extra::U4;

const WAVE_PERIOD_MS: u64 = 5000;
const SERVO_PWM_PERIOD_MS: u16 = 20;
const SERVO_PWM_FREQ_HZ: u16 = 1000 / SERVO_PWM_PERIOD_MS;

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

    // From <https://docs.embassy.dev/embassy-rp/git/rp2040/pwm/struct.Config.html>:
    // "the period in clock cycles of a slice can be computed as `(top + 1) * (phase_correct ? 1 : 2) * divider`."
    // We can obtain `clk_hz`, the number of clock cycles in one second, from the system.
    // We want to send a pulse every `SERVO_PWM_PERIOD_MS` milliseconds.
    // That's equivalent to `clk_hz / SERVO_PWM_FREQ_HZ` clock cycles.
    // So `(top + 1) * 2 * divider = clk_hz / SERVO_PWM_FREQ_HZ`.
    // We know that `top <= 65_535`, so `65_536 * 2 * divider <= clk_hz / SERVO_PWM_FREQ_HZ`.
    // Rearranging, `divider <= clk_hz / (131_072 * SERVO_PWM_FREQ_HZ)`.
    // Since integer division floors automatically, the above will work as an equation.
    // Then, once we have `divider`, we can rearrange the original equation:
    // `top = (clk_hz / (SERVO_PWM_FREQ_HZ * 2 * divider)) - 1`.
    let clk_hz: u32 = embassy_rp::clocks::clk_sys_freq();
    let () = log::info!("Clock frequency: {clk_hz:?} Hz");
    let Some(clk_hz) = FixedU32::<Radix>::checked_from_num(clk_hz) else {
        let mut ticker = Ticker::every(Duration::from_secs(1));
        loop {
            let () = log::info!("Clock frequency: {clk_hz:?} Hz");
            let () = log::error!("Clock frequency too large: {clk_hz:#?}");
            let () = ticker.next().await;
        }
    };
    let divider = {
        let denominator = (SERVO_PWM_FREQ_HZ as u32) << 17;
        let Some(denominator) = FixedU32::<Radix>::checked_from_num(denominator) else {
            let mut ticker = Ticker::every(Duration::from_secs(1));
            loop {
                let () = log::info!("Clock frequency: {clk_hz:?} Hz");
                let () = log::error!(
                    "Clock divider intermediate computation too large: {denominator:#?}"
                );
                let () = ticker.next().await;
            }
        };
        (clk_hz / denominator) + FixedU32::<Radix>::from_bits(1)
    };
    let () = log::info!("Clock divider: {divider:?}");
    let top: FixedU32<Radix> = {
        let denominator = (SERVO_PWM_FREQ_HZ as u32 * divider) << 1;
        let Some(denominator) = FixedU32::<Radix>::checked_from_num(denominator) else {
            let mut ticker = Ticker::every(Duration::from_secs(1));
            loop {
                let () = log::info!("Clock frequency: {clk_hz:?} Hz");
                let () = log::info!("Clock divider: {divider:?}");
                let () =
                    log::error!("Clock top intermediate computation too large: {denominator:#?}");
                let () = ticker.next().await;
            }
        };
        (clk_hz / denominator) - FixedU32::<Radix>::ONE
    };
    let () = log::info!("Clock top: {top:?}");
    let Some(top) = top.floor().checked_to_num() else {
        let mut ticker = Ticker::every(Duration::from_secs(1));
        loop {
            let () = log::info!("Clock frequency: {clk_hz:?} Hz");
            let () = log::info!("Clock divider: {divider:?}");
            let () = log::info!("Clock top: {top:?}");
            let () = log::error!("Clock top too large: {top:#?}");
            let () = ticker.next().await;
        }
    };
    let Some(divider) = FixedU16::<Radix>::lossless_try_from(divider) else {
        let mut ticker = Ticker::every(Duration::from_secs(1));
        loop {
            let () = log::info!("Clock frequency: {clk_hz:?} Hz");
            let () = log::info!("Clock divider: {divider:?}");
            let () = log::info!("Clock top: {top:?}");
            let () = log::error!("Clock divider too large: {divider:#?}");
            let () = ticker.next().await;
        }
    };

    let pwm_min = top / 20;
    let pwm_max = top / 10;
    let pwm_ctr = (pwm_min + pwm_max + 1) >> 1;
    let mut cfg = pwm::Config::default();
    cfg.compare_a = pwm_ctr;
    cfg.compare_b = pwm_ctr;
    cfg.divider = divider;
    cfg.enable = true;
    cfg.phase_correct = true;
    cfg.top = top;

    let pwm1 = Pwm::new_output_b(p.PWM_SLICE6, p.PIN_13, cfg.clone());
    let pwm2 = Pwm::new_output_b(p.PWM_SLICE7, p.PIN_15, cfg.clone());

    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        /*
        let theta =
            const { 2_f32 * core::f32::consts::PI } * ((counter as f32) / (WAVE_PERIOD_MS as f32));
        let x = 0.5_f32 * (1_f32 + libm::cosf(theta));
        let y = 0.5_f32 * (1_f32 + libm::sinf(theta));
        log::info!("({x:01.2:?}, {y:01.2:?})");
        */
        let () = log::info!("Hello!");
        let () = ticker.next().await;
    }
}
