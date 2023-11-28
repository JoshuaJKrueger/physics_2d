use nalgebra::{Point2, Rotation2, Vector2};
use ordered_float::OrderedFloat;

use crate::types::{Meter, SquareMeter, Radian, Kilogram, InvKilogram, KilogramMeterSquared, MeterSquaredPerKilogram, KilogramPerCubicMeter, NormalizedCoefficient, RadianPerSec, NewtonMeter};


// nalgebra only supports 3D cross product
fn cross(a: &Vector2<f64>, b: &Vector2<f64>) -> f64 {
    a.x * b.y - a.y * b.x
}

pub enum Shape {
    Circle { radius: Meter },
    Polygon { vertices: Vec<Point2<f64>> },
}

impl Shape {
    fn calculate_area(&self) -> SquareMeter {
        match self {
            Shape::Circle { radius} => OrderedFloat(std::f64::consts::PI) * radius * radius,
            Shape::Polygon { vertices } => {
                unimplemented!()
            },
        }
    }
}

pub struct Transform {
    pub pos: Point2<f64>,
    rot: Rotation2<f64>,
    scale: Vector2<f64>,
    orientation: Radian,
}

pub struct MassData {
    pub mass: Kilogram,
    pub inv_mass: InvKilogram,
    moment_inertia: KilogramMeterSquared,
    inv_m_inertia: MeterSquaredPerKilogram,
}

pub struct Material {
    density: KilogramPerCubicMeter,
    pub restitution: NormalizedCoefficient,
    pub dynamic_friction: NormalizedCoefficient,
    pub static_friction: NormalizedCoefficient,
}

pub struct Kinematics {
    pub vel: Vector2<f64>,
    pub angular_vel: RadianPerSec,
    torque: NewtonMeter,
}

pub struct Object
{
    pub shape: Shape,
    pub tx: Transform,
    pub mat: Material,
    pub mass_data: MassData,
    pub kinematics: Kinematics,
    pub force: Vector2<f64>,
}

impl Object {
    fn apply_force(&mut self, f: Vector2<f64>) {
        self.force += f;
    }

    fn apply_impulse(&mut self, imp: Vector2<f64>, contact_vec: Vector2<f64>) {
        self.kinematics.vel += self.mass_data.inv_mass * imp;
        self.kinematics.angular_vel += self.mass_data.inv_m_inertia * cross(&contact_vec, &imp);
    }
}
