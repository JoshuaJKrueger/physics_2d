use nalgebra::Vector2;

use std::f64::EPSILON;

// nalgebra only supports 3D cross product
pub fn cross_v_v(a: &Vector2<f64>, b: &Vector2<f64>) -> f64 {
    a.x * b.y - a.y * b.x
}

pub fn cross_s_v(a: f64, b: &Vector2<f64>) -> Vector2<f64> {
    Vector2::new(-a * b.y, a * b.x)
}

pub fn equal(a: f64, b: f64) -> bool {
    (a - b).abs() <= EPSILON
}

pub fn bias_gt(a: f64, b: f64) -> bool {
    a >= b * 0.95 + a * 0.01
}
