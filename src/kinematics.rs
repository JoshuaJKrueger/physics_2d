use nalgebra::Vector2;

use crate::types::{MeterPerSec, NewtonMeter, RadianPerSec};

pub struct Kinematics {
    pub vel: Vector2<MeterPerSec>,
    pub angular_vel: RadianPerSec,
    pub torque: NewtonMeter,
}

impl Kinematics {
    pub fn new(vel: Vector2<f64>, angular_vel: RadianPerSec, torque: NewtonMeter) -> Self {
        Kinematics {
            vel,
            angular_vel,
            torque,
        }
    }
}
