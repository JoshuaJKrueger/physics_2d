use nalgebra::Vector2;

use crate::types::{MeterPerSec, NewtonMeter, RadianPerSec};

/// Represents the kinematic properties of an object.
///
/// This structure includes linear velocity, angular velocity, and torque.
pub struct Kinematics {
    /// Linear velocity in 2D space.
    pub vel: Vector2<MeterPerSec>,
    /// Angular velocity representing rotational speed.
    pub angular_vel: RadianPerSec,
    /// Torque acting on the object.
    pub torque: NewtonMeter,
}

impl Kinematics {
    /// Creates a new `Kinematics` instance with the specified parameters.
    ///
    /// # Arguments
    ///
    /// * `vel` - Linear velocity in 2D space.
    /// * `angular_vel` - Angular velocity representing rotational speed.
    /// * `torque` - Torque acting on the object.
    ///
    /// # Returns
    ///
    /// A new `Kinematics` instance.
    pub fn new(vel: Vector2<f64>, angular_vel: RadianPerSec, torque: NewtonMeter) -> Self {
        Kinematics {
            vel,
            angular_vel,
            torque,
        }
    }
}
