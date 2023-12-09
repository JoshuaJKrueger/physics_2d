use nalgebra::Vector2;

// use std::f64::EPSILON;

// nalgebra only supports 3D cross product

/// Calculates the 2D cross product of two vectors.
///
/// # Arguments
///
/// * `a` - The first vector.
/// * `b` - The second vector.
///
/// # Returns
///
/// The result of the cross product operation.
pub fn cross_v_v(a: &Vector2<f64>, b: &Vector2<f64>) -> f64 {
    a.x * b.y - a.y * b.x
}

/// Calculates the 2D cross product of a scalar and a vector.
///
/// # Arguments
///
/// * `a` - The scalar value.
/// * `b` - The vector.
///
/// # Returns
///
/// The resulting 2D vector after the cross product operation.
pub fn cross_s_v(a: f64, b: &Vector2<f64>) -> Vector2<f64> {
    Vector2::new(-a * b.y, a * b.x)
}

// pub fn equal(a: f64, b: f64) -> bool {
//     (a - b).abs() <= EPSILON
// }

/// Performs a biased greater-than comparison between two floating-point numbers.
///
/// The bias helps in handling floating-point errors when comparing values that are very close.
///
/// # Arguments
///
/// * `a` - The first floating-point number.
/// * `b` - The second floating-point number.
///
/// # Returns
///
/// `true` if `a` is greater than or equal to a biased value of `b`, otherwise `false`.
pub fn bias_gt(a: f64, b: f64) -> bool {
    a >= b * 0.95 + a * 0.01
}
