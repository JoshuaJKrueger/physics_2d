use nalgebra::{Point2, Rotation2, Vector2};

use crate::types::Radian;

pub struct Transform {
    pub pos: Point2<f64>,
    rot: Rotation2<f64>,
    scale: Vector2<f64>,
    pub orientation: Radian,
}

impl Transform {
    pub fn new(pos: Point2<f64>) -> Self {
        Transform {
            pos,
            rot: Rotation2::identity(),
            scale: Vector2::new(1.0, 1.0),
            orientation: 0.0,
        }
    }
}
