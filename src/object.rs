use nalgebra::{Point2, Rotation2, Vector2};
use ordered_float::OrderedFloat;
use graphics::{Context, ellipse, line::Line, Transformed};
use opengl_graphics::GlGraphics;
use rand::Rng;
use std::f64::consts::PI;

use crate::types::{Meter, Radian, Kilogram, InvKilogram, KilogramMeterSquared, MeterSquaredPerKilogram, KilogramPerCubicMeter, NormalizedCoefficient, RadianPerSec, NewtonMeter, MeterPerSec};
use crate::constants::WHITE;


// nalgebra only supports 3D cross product
fn cross(a: &Vector2<f64>, b: &Vector2<f64>) -> f64 {
    a.x * b.y - a.y * b.x
}

pub enum Shape {
    Circle { radius: Meter },
    Polygon { vertices: Vec<Point2<f64>> },
}

impl Shape {
    fn calculate_mass_data(&self, density: KilogramPerCubicMeter) -> MassData {
        match self {
            Shape::Circle { radius } => {
                let m = OrderedFloat(PI) * radius * radius * density;
                MassData::new(*m, *(m * radius * radius))
            },
            Shape::Polygon { vertices } => {
                unimplemented!()
            },
        }
    }

    pub fn tag(&self) -> ShapeTag {
        match self {
            Shape::Circle { .. } => ShapeTag::Circle,
            Shape::Polygon { .. } => ShapeTag::Polygon,
        }
    }
}

pub enum ShapeTag {
    Circle,
    Polygon,
}

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

pub struct MassData {
    pub mass: Kilogram,
    pub inv_mass: InvKilogram,
    moment_inertia: KilogramMeterSquared,
    pub inv_m_inertia: MeterSquaredPerKilogram,
}

impl MassData {
    pub fn new(mass: Kilogram, moment_inertia: KilogramMeterSquared) -> Self {
        let inv_mass = if mass != 0.0 { 1.0 / mass } else { 0.0 };
        let inv_m_inertia = if moment_inertia != 0.0 { 1.0 / moment_inertia } else { 0.0 };

        MassData {
            mass,
            inv_mass,
            moment_inertia,
            inv_m_inertia,
        }
    }
}

pub struct Material {
    density: KilogramPerCubicMeter,
    pub restitution: NormalizedCoefficient,
    pub dynamic_friction: NormalizedCoefficient,
    pub static_friction: NormalizedCoefficient,
}

impl Material {
    pub fn new(density: KilogramPerCubicMeter, restitution: NormalizedCoefficient, dynamic_friction: NormalizedCoefficient, static_friction: NormalizedCoefficient) -> Self {
        Material {
            density,
            restitution,
            dynamic_friction,
            static_friction,
        }
    }
}

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
    pub fn new(shape: Shape, tx: Transform, mat: Option<Material>, mass_data: Option<MassData>, kinematics: Option<Kinematics>) -> Self {
        let mat = mat.unwrap_or_else(|| {
            // Generate random material properties
            let density = rand::thread_rng().gen_range(0.1..10.0);
            let restitution = rand::thread_rng().gen_range(0.0..1.0);
            let dynamic_friction = rand::thread_rng().gen_range(0.0..1.0);
            let static_friction = rand::thread_rng().gen_range(0.0..1.0);

            Material::new(density, OrderedFloat(restitution), OrderedFloat(dynamic_friction), OrderedFloat(static_friction),
            )
        });

        let mass_data = mass_data.unwrap_or_else(|| shape.calculate_mass_data(mat.density));

        let kinematics = kinematics.unwrap_or_else(|| {
            // Generate random kinematics properties
            let vel = Vector2::new(rand::thread_rng().gen_range(-10.0..10.0), rand::thread_rng().gen_range(-10.0..10.0));
            let angular_vel = rand::thread_rng().gen_range(-1.0..1.0);
            let torque = rand::thread_rng().gen_range(-1.0..1.0);

            Kinematics::new(vel, angular_vel, torque)
        });

        let force = Vector2::zeros();

        Object {shape, tx, mat, mass_data, kinematics, force}
    }

    fn apply_force(&mut self, f: Vector2<f64>) {
        self.force += f;
    }

    fn apply_impulse(&mut self, imp: Vector2<f64>, contact_vec: Vector2<f64>) {
        self.kinematics.vel += self.mass_data.inv_mass * imp;
        self.kinematics.angular_vel += self.mass_data.inv_m_inertia * cross(&contact_vec, &imp);
    }

    pub fn draw(&self, c: Context, gl: &mut GlGraphics) {
        match &self.shape {
            Shape::Circle { radius } => {
                ellipse::Ellipse::new_border(WHITE, 1.0).draw(ellipse::circle(self.tx.pos.x, self.tx.pos.y, **radius), &c.draw_state, c.transform, gl);
            }
            Shape::Polygon { vertices } => {
                // piston polygon only has a filled version
                for i in 0..vertices.len() {
                    let next_index = (i + 1) % vertices.len();
                    let start = [vertices[i].x, vertices[i].y];
                    let end = [vertices[next_index].x, vertices[next_index].y];
    
                    Line::new(WHITE, 1.0).draw([start[0], start[1], end[0], end[1]], &c.draw_state, c.transform.trans(self.tx.pos.x, self.tx.pos.y), gl);
                }
            }
        }
    }
}
