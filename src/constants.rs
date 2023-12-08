use nalgebra::Vector2;
// use ordered_float::OrderedFloat;

use crate::types::MeterPerSquaredSecond;

pub const GRAVITY: Vector2<MeterPerSquaredSecond> = Vector2::new(0.0, 9.8);
pub const RED: [f32; 4] = [1.0, 0.0, 0.0, 1.0];
pub const GREEN: [f32; 4] = [0.0, 1.0, 0.0, 1.0];
pub const WHITE: [f32; 4] = [1.0, 1.0, 1.0, 1.0];
pub const BLACK: [f32; 4] = [0.0, 0.0, 0.0, 1.0];
pub const ONE_THIRD: f64 = 1.0 / 3.0;
// For positional correction
// pub const PEN_ALLOWANCE: NormalizedCoefficient = OrderedFloat(0.05);
// pub const PERCENT_CORRECTION: NormalizedCoefficient = OrderedFloat(0.4);
