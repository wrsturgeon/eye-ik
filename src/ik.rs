use {crate::pwm, core::f32::consts::PI};

pub const LENGTH_CENTER_TO_YAW: f32 = 0.900;
pub const LENGTH_YAW_TO_HIP: f32 = 0.574;
pub const LENGTH_HIP_TO_KNEE: f32 = 2.563;
pub const LENGTH_KNEE_TO_FOOT: f32 = 5.467;

pub struct CartesianDisplacementFromEyeCenterLookingForward {
    /// Along the axis formed if the eye were to shoot a laser out of its pupil,
    /// parallel to the ground.
    pub x: f32,
    /// Left/right relative to the eye, parallel to the ground.
    pub y: f32,
    /// Up and down, perpendicular to the ground.
    pub z: f32,
}

pub struct HipToFootDisplacementIn2dPlane {
    pub x: f32,
    pub y: f32,
}

pub struct Angles {
    pub yaw: f32,
    pub hip: f32,
    pub knee: f32,
}

pub struct HipAndKneeAngles {
    pub hip: f32,
    pub knee: f32,
}

#[derive(Debug)]
pub enum HipToFootError {
    Unreachable(Unreachable),
    KneeLock(KneeLock),
}

#[derive(Debug)]
pub struct Unreachable {
    pub reach_from_hip: f32,
    pub distance: f32,
}

#[derive(Debug)]
pub enum AngleOutOfRange {
    Yaw { radians: f32 },
    Hip { radians: f32 },
    Knee { radians: f32 },
}

#[derive(Debug)]
pub enum KneeLock {
    TooClose { hip: f32, knee: f32 },
    TooFar { hip: f32, knee: f32 },
}

impl HipToFootDisplacementIn2dPlane {
    #[inline]
    pub fn magnitude_squared(&self) -> f32 {
        (self.x * self.x) + (self.y * self.y)
    }
}

#[inline]
pub fn hip_to_foot_2d(
    displacement: HipToFootDisplacementIn2dPlane,
) -> Result<HipAndKneeAngles, HipToFootError> {
    const LENGTH_HIP_TO_KNEE_SQUARED: f32 = LENGTH_HIP_TO_KNEE * LENGTH_HIP_TO_KNEE;
    const LENGTH_KNEE_TO_FOOT_SQUARED: f32 = LENGTH_KNEE_TO_FOOT * LENGTH_KNEE_TO_FOOT;
    const REACH_FROM_HIP: f32 = LENGTH_HIP_TO_KNEE + LENGTH_KNEE_TO_FOOT;

    let distance_squared = displacement.magnitude_squared();
    let distance = libm::sqrtf(distance_squared);
    let HipToFootDisplacementIn2dPlane { x, y } = displacement;

    {
        // Check if this point is even reachable:
        if distance > REACH_FROM_HIP {
            return Err(HipToFootError::Unreachable(Unreachable {
                reach_from_hip: REACH_FROM_HIP,
                distance,
            }));
        }
    }

    let hip_radians = {
        // Law of cosines:
        // L_2^2 = L_1^2 + hypotenuse^2 - 2 L_1 hypotenuse cos(hip_internal_radians)
        // ==> cos(hip_internal_radians) = L_1^2 + hypotenuse^2 - L_2^2 / 2 L_1 hypotenuse
        let hip_internal_radians = {
            let cos_hip_internal_radians = {
                (const { LENGTH_HIP_TO_KNEE_SQUARED - LENGTH_KNEE_TO_FOOT_SQUARED }
                    + distance_squared)
                    * const { 0.5 / LENGTH_HIP_TO_KNEE }
                    / distance
            };
            libm::acosf(cos_hip_internal_radians)
        };

        // Arctangent of the whole enchilada on [-pi, pi):
        let sigma_radians = libm::atan2f(y, x);

        sigma_radians + hip_internal_radians
    };

    let knee_radians = {
        // Law of cosines:
        // hypotenuse^2 = L_1^2 + L_2^2 - 2 L_1 L_2 cos(knee_internal_radians)
        // ==> cos(knee_internal_radians) = L_1^2 + L_2^2 - hypotenuse^2 / 2 L_1 L_2
        let knee_internal_radians = {
            let cos_knee_internal_radians =
                (const { LENGTH_HIP_TO_KNEE_SQUARED + LENGTH_KNEE_TO_FOOT_SQUARED }
                    - distance_squared)
                    * 0.5
                    / const { LENGTH_HIP_TO_KNEE * LENGTH_KNEE_TO_FOOT };
            libm::acosf(cos_knee_internal_radians)
        };
        knee_internal_radians - const { 0.5 * PI } + hip_radians
    };

    let hip = hip_radians * pwm::RADIANS_TO_SERVO;
    let knee = -knee_radians * pwm::RADIANS_TO_SERVO;

    // TODO: Knee lock!

    Ok(HipAndKneeAngles { hip, knee })
}
