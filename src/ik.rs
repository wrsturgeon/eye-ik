use core::f32::consts::PI;

const HIP_SERVO_MIN_RADIANS: f32 = -0.5 * PI;
const HIP_SERVO_MAX_RADIANS: f32 = 0.5 * PI;
const KNEE_SERVO_MIN_RADIANS: f32 = -0.5 * PI;
const KNEE_SERVO_MAX_RADIANS: f32 = 0.5 * PI;

pub struct Leg {
    pub length_hip_to_knee: f32,
    pub length_knee_to_foot: f32,
}

pub struct Cartesian {
    pub x: f32,
    pub y: f32,
}

pub struct Servos {
    pub hip: f32,
    pub knee: f32,
}

#[derive(Debug)]
pub enum Error {
    Unreachable(Unreachable),
    ServoOutOfRange(ServoOutOfRange),
}

#[derive(Debug)]
pub struct Unreachable {
    pub length_fully_extended_squared: f32,
    pub magnitude_squared: f32,
}

#[derive(Debug)]
pub enum ServoOutOfRange {
    Hip { radians: f32 },
    Knee { radians: f32 },
}

impl Cartesian {
    #[inline]
    pub fn magnitude_squared(&self) -> f32 {
        (self.x * self.x) + (self.y * self.y)
    }
}

impl Leg {
    #[inline]
    pub fn length_fully_extended(&self) -> f32 {
        self.length_hip_to_knee + self.length_knee_to_foot
    }

    #[inline]
    pub fn inverse_kinematics(&self, vec_hip_to_foot: Cartesian) -> Result<Servos, Error> {
        let magnitude_squared = vec_hip_to_foot.magnitude_squared();
        let Cartesian { x, y } = vec_hip_to_foot;
        let length_hip_to_knee_squared = self.length_hip_to_knee * self.length_hip_to_knee;
        let length_knee_to_foot_squared = self.length_knee_to_foot * self.length_knee_to_foot;

        {
            // Check if this point is even reachable:
            let length_fully_extended_squared = {
                let length_fully_extended = self.length_fully_extended();
                length_fully_extended * length_fully_extended
            };
            if magnitude_squared > length_fully_extended_squared {
                return Err(Error::Unreachable(Unreachable {
                    length_fully_extended_squared,
                    magnitude_squared,
                }));
            }
        }

        let theta_hip = {
            // Law of cosines:
            // L_2^2 = L_1^2 + hypotenuse^2 - 2 L_1 hypotenuse cos(theta_hip_internal)
            // ==> cos(theta_hip_internal) = L_1^2 + hypotenuse^2 - L_2^2 / 2 L_1 hypotenuse
            let theta_hip_internal = {
                let cos_theta_hip_internal = {
                    let magnitude = libm::sqrtf(magnitude_squared);
                    (length_hip_to_knee_squared + magnitude_squared - length_knee_to_foot_squared)
                        * 0.5
                        / (self.length_hip_to_knee * magnitude)
                };
                libm::acosf(cos_theta_hip_internal)
            };

            // Arctangent of the whole enchilada on [-pi, pi):
            let theta_sigma = libm::atan2f(y, x);

            theta_sigma + theta_hip_internal
        };

        let theta_knee = {
            // Law of cosines:
            // hypotenuse^2 = L_1^2 + L_2^2 - 2 L_1 L_2 cos(theta_knee_internal)
            // ==> cos(theta_knee_internal) = L_1^2 + L_2^2 - hypotenuse^2 / 2 L_1 L_2
            let theta_knee_internal = {
                let cos_theta_knee_internal =
                    (length_hip_to_knee_squared + length_knee_to_foot_squared - magnitude_squared)
                        * 0.5
                        / (self.length_hip_to_knee * self.length_knee_to_foot);
                libm::acosf(cos_theta_knee_internal)
            };
            theta_knee_internal - const { 0.5 * PI } + theta_hip
        };

        let hip = if (theta_hip < HIP_SERVO_MIN_RADIANS) || (theta_hip > HIP_SERVO_MAX_RADIANS) {
            return Err(Error::ServoOutOfRange(ServoOutOfRange::Hip {
                radians: theta_hip,
            }));
        } else {
            theta_hip * const { 1.0 / (0.5 * PI) }
        };

        let knee = if (theta_knee < KNEE_SERVO_MIN_RADIANS) || (theta_knee > KNEE_SERVO_MAX_RADIANS) {
            return Err(Error::ServoOutOfRange(ServoOutOfRange::Knee {
                radians: theta_knee,
            }));
        } else {
            theta_knee * const { 1.0 / (0.5 * PI) }
        };

        Ok(Servos { hip, knee })
    }
}
