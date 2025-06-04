#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

pub mod ik;

use {
    defmt_rtt as _,
    embassy_executor::Spawner,
    embassy_rp::{
        bind_interrupts,
        peripherals::{UART1, USB},
        pwm::{self, Pwm, SetDutyCycle as _},
        uart, usb,
    },
    embassy_time::{Duration, Ticker, Timer},
    fixed::{FixedU16, FixedU32, traits::LosslessTryFrom, types::extra::U4},
    panic_probe as _,
};

bind_interrupts!(struct Irqs {
    UART1_IRQ => uart::InterruptHandler<UART1>;
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

const SERVO_PWM_PERIOD_MS: u16 = 20;
const SERVO_PWM_FREQ_HZ: u16 = 1000 / SERVO_PWM_PERIOD_MS;

const WAVE_PERIOD_MS: u16 = 1000;
const MAIN_LOOP_PERIOD_MS: u16 = SERVO_PWM_PERIOD_MS;

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
    let Some(clk_hz) = FixedU32::<U4>::checked_from_num(clk_hz) else {
        let mut ticker = Ticker::every(Duration::from_secs(1));
        loop {
            let () = log::info!("Clock frequency: {clk_hz:?} Hz");
            let () = log::error!("Clock frequency too large: {clk_hz:#?}");
            let () = ticker.next().await;
        }
    };
    let divider = {
        let denominator = (SERVO_PWM_FREQ_HZ as u32) << 17;
        let Some(denominator) = FixedU32::<U4>::checked_from_num(denominator) else {
            let mut ticker = Ticker::every(Duration::from_secs(1));
            loop {
                let () = log::info!("Clock frequency: {clk_hz:?} Hz");
                let () = log::error!(
                    "Clock divider intermediate computation too large: {denominator:#?}"
                );
                let () = ticker.next().await;
            }
        };
        (clk_hz / denominator) + FixedU32::<U4>::from_bits(1)
    };
    let () = log::info!("Clock divider: {divider:?}");
    let top: FixedU32<U4> = {
        let denominator = (SERVO_PWM_FREQ_HZ as u32 * divider) << 1;
        let Some(denominator) = FixedU32::<U4>::checked_from_num(denominator) else {
            let mut ticker = Ticker::every(Duration::from_secs(1));
            loop {
                let () = log::info!("Clock frequency: {clk_hz:?} Hz");
                let () = log::info!("Clock divider: {divider:?}");
                let () =
                    log::error!("Clock top intermediate computation too large: {denominator:#?}");
                let () = ticker.next().await;
            }
        };
        (clk_hz / denominator) - FixedU32::<U4>::ONE
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
    let Some(divider) = FixedU16::<U4>::lossless_try_from(divider) else {
        let mut ticker = Ticker::every(Duration::from_secs(1));
        loop {
            let () = log::info!("Clock frequency: {clk_hz:?} Hz");
            let () = log::info!("Clock divider: {divider:?}");
            let () = log::info!("Clock top: {top:?}");
            let () = log::error!("Clock divider too large: {divider:#?}");
            let () = ticker.next().await;
        }
    };

    let pwm_min = (top as f32) / 20.0;
    let pwm_max = (top as f32) / 10.0;
    let pwm_ctr = 0.5 * (pwm_min + pwm_max);
    let pwm_rng = pwm_ctr - pwm_min;

    let (pwm_hip, pwm_knee) = Pwm::new_output_ab(p.PWM_SLICE7, p.PIN_14, p.PIN_15, {
        let mut cfg = pwm::Config::default();
        cfg.compare_a = pwm_ctr as _;
        cfg.compare_b = pwm_ctr as _;
        cfg.divider = divider;
        cfg.enable = true;
        cfg.phase_correct = true;
        cfg.top = top;
        cfg
    })
    .split();
    let Some(mut pwm_hip) = pwm_hip else {
        let mut ticker = Ticker::every(Duration::from_secs(1));
        loop {
            let () = log::info!("Clock frequency: {clk_hz:?} Hz");
            let () = log::info!("Clock divider: {divider:?}");
            let () = log::info!("Clock top: {top:?}");
            let () = log::error!("No hip PWM (no fucking clue what this means, though)");
            let () = ticker.next().await;
        }
    };
    let Some(mut pwm_knee) = pwm_knee else {
        let mut ticker = Ticker::every(Duration::from_secs(1));
        loop {
            let () = log::info!("Clock frequency: {clk_hz:?} Hz");
            let () = log::info!("Clock divider: {divider:?}");
            let () = log::info!("Clock top: {top:?}");
            let () = log::error!("No knee PWM (no fucking clue what this means, though)");
            let () = ticker.next().await;
        }
    };

    let leg = ik::Leg {
        length_hip_to_knee: 2.5,
        length_knee_to_foot: 5.6,
    };

    let mut counter: u16 = 0;
    let mut ticker = Ticker::every(Duration::from_millis(MAIN_LOOP_PERIOD_MS as _));
    loop {
        let theta =
            (counter as f32) * const { 2.0 * core::f32::consts::PI / (WAVE_PERIOD_MS as f32) };

        let x = 2.75 + libm::cosf(theta);
        let y = -5.85 - libm::sinf(theta);
        let xy = (x, y);
        let vec_from_knee_to_foot = ik::Cartesian { x, y };

        let servos = leg
            .inverse_kinematics(vec_from_knee_to_foot)
            .map(|ik::Servos { hip, knee }| (hip, knee));

        log::info!("");
        log::info!("Cartesian: {:01.2?}", Result::<_, ()>::Ok(xy));
        log::info!("Servo(IK): {servos:01.2?}");

        if let Ok((hip, knee)) = servos {
            match pwm_hip.set_duty_cycle((pwm_ctr - pwm_rng * hip) as _) {
                Ok(()) => {}
                Err(e) => log::error!("Couldn't set hip duty cycle: {e:?}"),
            }
            match pwm_knee.set_duty_cycle((pwm_ctr + pwm_rng * knee) as _) {
                Ok(()) => {}
                Err(e) => log::error!("Couldn't set knee duty cycle: {e:?}"),
            }
        }

        let () = ticker.next().await;
        counter += MAIN_LOOP_PERIOD_MS;
        if counter >= WAVE_PERIOD_MS {
            counter -= WAVE_PERIOD_MS;
        }
    }
}
