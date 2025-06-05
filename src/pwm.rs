use {
    embassy_rp::{
        Peripheral,
        pwm::{self, Config, Pwm, PwmOutput},
    },
    embassy_sync::once_lock::OnceLock,
    embassy_time::{Duration, Ticker},
    fixed::{FixedU16, FixedU32, traits::LosslessTryFrom, types::extra::U4},
};

// From <https://docs.embassy.dev/embassy-rp/git/rp2040/pwm/struct.Config.html>:
// "the period in clock cycles of a slice can be computed as `(top + 1) * (phase_correct ? 1 : 2) * divider`."
// We can obtain `clock_hz`, the number of clock cycles in one second, from the system.
// We want to send a pulse every `PULSE_PERIOD_MS` milliseconds.
// That's equivalent to `clock_hz / PULSE_FREQ_HZ` clock cycles.
// So `(top + 1) * 2 * divider = clock_hz / PULSE_FREQ_HZ`.
// We know that `top <= 65_535`, so `65_536 * 2 * divider <= clock_hz / PULSE_FREQ_HZ`.
// Rearranging, `divider <= clock_hz / (131_072 * PULSE_FREQ_HZ)`.
// Since integer division floors automatically, the above will work as an equation.
// Then, once we have `divider`, we can rearrange the original equation:
// `top = (clock_hz / (PULSE_FREQ_HZ * 2 * divider)) - 1`.

pub const PULSE_PERIOD_MS: u16 = 20;
pub const PULSE_FREQ_HZ: u16 = 1000 / PULSE_PERIOD_MS;

pub const RADIANS_TO_SERVO: f32 = {
    // These servos have 180-degree travel (+/-90).
    // So we want [-90, +90] to map to [-1, +1].
    // 90 degrees === pi/2 radians.
    // So, given x: rad, we need to multiply by 1: deg/rad.
    // 1: deg/rad = 90/(pi/2) = 180/pi.
    // Then 90 degrees = 1 servo-input, so we can chain:
    // (1: deg/rad)(1: pwm/deg) = (180/pi)(1/90) = 2/pi.
    2.0 / core::f32::consts::PI
};

#[inline]
pub async fn get_or_init<T, F: async FnOnce() -> T>(lock: &OnceLock<T>, f: F) -> &T {
    if let Some(t) = lock.try_get() {
        return t;
    }
    let _: Result<(), T> = lock.init(f().await);
    lock.try_get().unwrap()
}

#[inline]
pub async fn clock_frequency() -> u32 {
    static LOCK: OnceLock<u32> = OnceLock::new();

    *get_or_init(&LOCK, async || {
        let clk_hz: u32 = embassy_rp::clocks::clk_sys_freq();
        let () = log::info!("Clock frequency: {clk_hz:?} Hz");
        clk_hz
    })
    .await
}

#[inline]
pub async fn clock_frequency_fp() -> FixedU32<U4> {
    static LOCK: OnceLock<FixedU32<U4>> = OnceLock::new();

    *get_or_init(&LOCK, async || {
        let clock_hz = clock_frequency().await;
        let Some(clock_hz) = FixedU32::<U4>::checked_from_num(clock_hz) else {
            let mut ticker = Ticker::every(Duration::from_secs(1));
            loop {
                let () = log::error!("Clock frequency too large: {clock_hz:#?}");
                let () = ticker.next().await;
            }
        };
        clock_hz
    })
    .await
}

#[inline]
pub async fn clock_divider_32b() -> FixedU32<U4> {
    static LOCK: OnceLock<FixedU32<U4>> = OnceLock::new();

    *get_or_init(&LOCK, async || {
        let divider = {
            let denominator = (PULSE_FREQ_HZ as u32) << 17;
            let Some(denominator) = FixedU32::<U4>::checked_from_num(denominator) else {
                let mut ticker = Ticker::every(Duration::from_secs(1));
                loop {
                    let () = log::error!(
                        "Clock divider intermediate computation too large: {denominator:#?}"
                    );
                    let () = ticker.next().await;
                }
            };
            (clock_frequency_fp().await / denominator) + FixedU32::<U4>::from_bits(1)
        };
        let () = log::info!("Clock divider: {divider:?}");
        divider
    })
    .await
}

#[inline]
pub async fn clock_top() -> u16 {
    static LOCK: OnceLock<u16> = OnceLock::new();

    *get_or_init(&LOCK, async || {
        let divider = clock_divider_32b().await;
        let denominator = (PULSE_FREQ_HZ as u32 * divider) << 1;
        let Some(denominator) = FixedU32::<U4>::checked_from_num(denominator) else {
            let mut ticker = Ticker::every(Duration::from_secs(1));
            loop {
                let () =
                    log::error!("Clock top intermediate computation too large: {denominator:#?}");
                let () = ticker.next().await;
            }
        };
        let top = (clock_frequency_fp().await / denominator) - FixedU32::<U4>::ONE;
        let () = log::info!("Clock top: {top:?}");
        let Some(top) = top.floor().checked_to_num() else {
            let mut ticker = Ticker::every(Duration::from_secs(1));
            loop {
                let () = log::error!("Clock top too large: {top:#?}");
                let () = ticker.next().await;
            }
        };
        top
    })
    .await
}

#[inline]
pub async fn clock_divider() -> FixedU16<U4> {
    static LOCK: OnceLock<FixedU16<U4>> = OnceLock::new();

    *get_or_init(&LOCK, async || {
        let divider = clock_divider_32b().await;
        let Some(divider) = FixedU16::<U4>::lossless_try_from(divider) else {
            let mut ticker = Ticker::every(Duration::from_secs(1));
            loop {
                let () = log::error!("Clock divider too large: {divider:#?}");
                let () = ticker.next().await;
            }
        };
        divider
    })
    .await
}

#[inline]
pub async fn pulse_min() -> f32 {
    static LOCK: OnceLock<f32> = OnceLock::new();
    *get_or_init(&LOCK, async || (clock_top().await as f32) / 20.0).await
}
#[inline]
pub async fn pulse_max() -> f32 {
    static LOCK: OnceLock<f32> = OnceLock::new();
    *get_or_init(&LOCK, async || (clock_top().await as f32) / 10.0).await
}
#[inline]
pub async fn pulse_center() -> f32 {
    static LOCK: OnceLock<f32> = OnceLock::new();
    *get_or_init(&LOCK, async || {
        0.5 * (pulse_min().await + pulse_max().await)
    })
    .await
}
#[inline]
pub async fn pulse_range_plus_minus() -> f32 {
    static LOCK: OnceLock<f32> = OnceLock::new();
    *get_or_init(&LOCK, async || pulse_center().await - pulse_min().await).await
}

#[inline]
pub async fn init_slice<'d, Slice: pwm::Slice>(
    slice: impl Peripheral<P = Slice> + 'd,
    a: impl Peripheral<P = impl pwm::ChannelAPin<Slice>> + 'd,
    b: impl Peripheral<P = impl pwm::ChannelBPin<Slice>> + 'd,
) -> (PwmOutput<'d>, PwmOutput<'d>) {
    let (a, b) = Pwm::new_output_ab(slice, a, b, {
        let mut cfg = Config::default();
        // let pulse_center = pulse_center().await;
        cfg.compare_a = 0; // pulse_center as _;
        cfg.compare_b = 0; // pulse_center as _;
        cfg.divider = clock_divider().await;
        cfg.enable = true;
        cfg.phase_correct = true;
        cfg.top = clock_top().await;
        cfg
    })
    .split();

    let Some(a) = a else {
        let mut ticker = Ticker::every(Duration::from_secs(1));
        loop {
            let () = log::error!("PWM slice did not allow an A channel");
            let () = ticker.next().await;
        }
    };

    let Some(b) = b else {
        let mut ticker = Ticker::every(Duration::from_secs(1));
        loop {
            let () = log::error!("PWM slice did not allow a B channel");
            let () = ticker.next().await;
        }
    };

    (a, b)
}
