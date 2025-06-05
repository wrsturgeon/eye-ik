use embassy_rp::pwm::{PwmError, PwmOutput, SetDutyCycle};

pub struct Servo<'d> {
    pwm: PwmOutput<'d>,
    pulse_center: f32,
    pulse_min: f32,
    pulse_max: f32,
}

#[derive(Debug)]
pub enum CouldntInitialize {
    PulseCenterOutOfRange(OutOfRange),
    PulseRangeLowerOutOfRange(OutOfRange),
    PulseRangeHigherOutOfRange(OutOfRange),
}

#[derive(Debug)]
pub enum CouldntMove {
    OutOfRange(OutOfRange),
    PwmError(PwmError),
}

#[derive(Debug)]
pub struct OutOfRange {
    pub min: f32,
    pub max: f32,
    pub observed: f32,
}

impl OutOfRange {
    #[inline]
    pub fn check(min: f32, max: f32, observed: f32) -> Result<(), Self> {
        if (min..=max).contains(&observed) {
            Ok(())
        } else {
            Err(Self { min, max, observed })
        }
    }
}

impl<'d> Servo<'d> {
    #[inline]
    pub fn with_center_and_ranges(
        pwm: PwmOutput<'d>,
        pulse_center: f32,
        pulse_range_lower: f32,
        pulse_range_higher: f32,
    ) -> Result<Self, CouldntInitialize> {
        let () = OutOfRange::check(-1.0, 1.0, pulse_center)
            .map_err(CouldntInitialize::PulseCenterOutOfRange)?;
        let () = OutOfRange::check(0.0, 1.0 + pulse_center, pulse_range_lower)
            .map_err(CouldntInitialize::PulseRangeLowerOutOfRange)?;
        let () = OutOfRange::check(0.0, 1.0 - pulse_center, pulse_range_higher)
            .map_err(CouldntInitialize::PulseRangeHigherOutOfRange)?;
        let pulse_min = pulse_center - pulse_range_lower;
        let pulse_max = pulse_center + pulse_range_higher;
        Ok(Self {
            pwm,
            pulse_center,
            pulse_min,
            pulse_max,
        })
    }

    #[inline]
    pub fn go_to(&mut self, position: f32) -> Result<(), CouldntMove> {
        let () = OutOfRange::check(self.pulse_min, self.pulse_max, position)
            .map_err(CouldntMove::OutOfRange)?;
        let duty = self.pulse_center + position;
        self.pwm
            .set_duty_cycle(duty as _)
            .map_err(CouldntMove::PwmError)
    }
}
