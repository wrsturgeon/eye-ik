use {
    crate::{
        ik, pwm,
        servo::{self, Servo},
    },
    core::f32::consts::PI,
    embassy_rp::pwm::PwmOutput,
};

const TWO_PI: f32 = 2.0 * PI;
const NEGATIVE_PI: f32 = -PI;

#[derive(Debug)]
pub enum CouldntInit {
    YawServo(servo::CouldntInitialize),
    HipServo(servo::CouldntInitialize),
    KneeServo(servo::CouldntInitialize),
}

#[derive(Debug)]
pub enum IkError {
    CouldntMoveYaw(servo::CouldntMove),
    CouldntMoveHip(servo::CouldntMove),
    CouldntMoveKnee(servo::CouldntMove),
    Ik2dError(ik::HipToFootError),
}

#[inline]
fn clamp_plus_minus_pi(mut radians: f32) -> f32 {
    while radians >= PI {
        radians -= TWO_PI
    }
    while radians < NEGATIVE_PI {
        radians += TWO_PI
    }
    radians
}

pub struct Leg<'d> {
    yaw: Servo<'d>,
    hip: Servo<'d>,
    knee: Servo<'d>,
    yaw_servo_x: f32,
    yaw_servo_y: f32,
    home_yaw_radians: f32,
}

impl<'d> Leg<'d> {
    #[inline]
    pub fn with_home_yaw(
        home_yaw_radians: f32,
        yaw_pwm: PwmOutput<'d>,
        hip_pwm: PwmOutput<'d>,
        knee_pwm: PwmOutput<'d>,
    ) -> Result<Self, CouldntInit> {
        let home_yaw_radians = clamp_plus_minus_pi(home_yaw_radians);
        Ok(Self {
            yaw: Servo::with_center_and_ranges(
                yaw_pwm,
                0.0,
                const { -PI / 6.0 },
                const { PI / 6.0 },
            )
            .map_err(CouldntInit::YawServo)?,
            hip: Servo::with_center_and_ranges(
                hip_pwm,
                0.0,
                const { -PI / 2.0 },
                const { PI / 2.0 },
            )
            .map_err(CouldntInit::HipServo)?,
            knee: Servo::with_center_and_ranges(
                knee_pwm,
                0.0,
                const { -PI / 4.0 },
                const { PI / 4.0 },
            )
            .map_err(CouldntInit::KneeServo)?,
            yaw_servo_x: libm::cosf(home_yaw_radians) * ik::LENGTH_CENTER_TO_YAW,
            yaw_servo_y: libm::sinf(home_yaw_radians) * ik::LENGTH_CENTER_TO_YAW,
            home_yaw_radians,
        })
    }

    #[inline]
    pub fn ik_to(
        &mut self,
        ik::CartesianDisplacementFromEyeCenterLookingForward {
            x: foot_x,
            y: foot_y,
            z: foot_z,
        }: ik::CartesianDisplacementFromEyeCenterLookingForward,
    ) -> Result<(), IkError> {
        // The (x, y) plane is as if you were looking down over the robot.
        // The z plane is up/down, as if it were jumping.

        let horizontal_displacement_x = foot_x - self.yaw_servo_x;
        let horizontal_displacement_y = foot_y - self.yaw_servo_y;
        let global_yaw = libm::atan2f(horizontal_displacement_x, horizontal_displacement_y); // Already guaranteed to be on [-pi, pi).

        // let hip_servo_x = self.yaw_servo_x + libm::cosf(global_yaw) * LENGTH_YAW_TO_HIP;
        // let hip_servo_y = self.yaw_servo_y + libm::sinf(global_yaw) * LENGTH_YAW_TO_HIP;

        // Update yaw:
        {
            let mut local_yaw = global_yaw - self.home_yaw_radians;
            while local_yaw >= PI {
                local_yaw -= TWO_PI
            }
            while local_yaw < NEGATIVE_PI {
                local_yaw += TWO_PI
            }
            let () = self
                .yaw
                .go_to(pwm::RADIANS_TO_SERVO * local_yaw)
                .map_err(IkError::CouldntMoveYaw)?;
        };

        let distance_hip_to_foot_projected = {
            libm::sqrtf(
                (horizontal_displacement_x * horizontal_displacement_x)
                    + (horizontal_displacement_y * horizontal_displacement_y),
            )
        };

        let hip_to_foot = ik::HipToFootDisplacementIn2dPlane {
            x: distance_hip_to_foot_projected,
            y: foot_z,
        };
        let ik::HipAndKneeAngles { hip, knee } =
            ik::hip_to_foot_2d(hip_to_foot).map_err(IkError::Ik2dError)?;
        let () = self
            .hip
            .go_to(pwm::RADIANS_TO_SERVO * hip)
            .map_err(IkError::CouldntMoveHip)?;
        let () = self
            .knee
            .go_to(pwm::RADIANS_TO_SERVO * knee)
            .map_err(IkError::CouldntMoveKnee)?;
        Ok(())
    }
}
