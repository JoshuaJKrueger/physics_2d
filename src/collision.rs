use nalgebra::{Point2, Vector2};

pub enum Shape {
    Circle { radius: f64 },
    Rectangle { width: f64, height: f64 },
}

struct AABB
{
    min: Point2<f64>,
    max: Point2<f64>,
}

struct Circle
{
    center: Point2<f64>,
    radius: f64,
}

pub struct Object
{
    pub position: Point2<f64>,
    pub velocity: Vector2<f64>,
    pub angle: f64,
    pub shape: Shape,
    pub mass: f64,
    pub restitution: f64,
    pub inv_mass: f64,
}

struct Manifold<'a>
{
    a: &'a Object,
    b: &'a Object,
    penetration: Option<f64>,
    normal: Option<Vector2<f64>>,
}

fn aabb_aabb(a: &AABB, b: &AABB) -> bool {
    // Check if the two Axis-Aligned Bounding Boxes (AABB) intersect along any axis.
    // If there is separation along any axis, return false; otherwise, return true.
    !(a.max.x < b.min.x || a.min.x > b.max.x || a.max.y < b.min.y || a.min.y > b.max.y)
}

fn circle_circle(a: &Circle, b: &Circle) -> bool {
    // TODO: Compare performance
    // let r = a.radius + b.radius;
    // r *= r;
    // r < (a.x + b.x)^2 + (a.y + b.y)^2
    (b.center - a.center).norm_squared() < (a.radius + b.radius).powi(2)
}

// // TODO: Refactor this function
// fn ResolveCollision(a: &Object, b: &Object) {
//     let rel_vel = b.velocity - a.velocity;
//     // TODO: Get normal
//     let vel_norm = rel_vel.dot(&normal);

//     // Make sure objects are moving towards one another
//     if (vel_norm < 0) {
//         let e = min(a.restitution, b.restitution);
//         let j = -(1 + e) * vel_norm;

//         j /= a.inv_mass + b.inv_mass;

//         let impulse = j * normal;

//         a.velocity -= a.inv_mass * impulse;
//         b.velocity += b.inv_mass * impulse;

//         // TODO: Might be simpler, check if it functions the same
//         // let mass_sum = a.mass + b.mass;
//         // let ratio = a.mass / mass_sum;
//         // a.velocity -= ratio * impulse;
//         // ratio = b.mass / mass_sum;
//         // b.velocity += ratio * impulse;
//     }
// }

// // TODO: Refactor
// fn correct_float_error(a: &Object, b: &Object) {
//     const PERCENT: f64 = 0.2;
//     const JITTER_THESHOLD: f64 = 0.01;
//     let correction = max(penetration_depth - k_slop, 0.0) / (a.inv_mass + b.inv_mass) * PERCENT * n;
//     a.position -= a.inv_mass * correction;
//     b.position += b.inv_mass * correction;
// }