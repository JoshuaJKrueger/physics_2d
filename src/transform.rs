use nalgebra::Point2;

use crate::types::Radian;

/// Represents the transformation (position and orientation) of an object in 2D space.

pub struct Transform {
    /// The position of the object.
    pub pos: Point2<f64>,
    // rot: Rotation2<f64>,
    // scale: Vector2<f64>,
    /// The orientation (rotation) of the object in radians.
    pub orientation: Radian,
}

impl Transform {
    /// Creates a new `Transform` with the specified position.
    ///
    /// # Arguments
    ///
    /// * `pos` - The position of the object.
    pub fn new(pos: Point2<f64>) -> Self {
        Transform {
            pos,
            // rot: Rotation2::identity(),
            // scale: Vector2::new(1.0, 1.0),
            orientation: 0.0,
        }
    }
}
