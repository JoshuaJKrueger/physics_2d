use graphics::Context;
use opengl_graphics::GlGraphics;

use nalgebra::Vector2;

use ordered_float::OrderedFloat;
use rand::Rng;

use crate::custom_math::cross_v_v;
use crate::kinematics::Kinematics;
use crate::mass_data::MassData;
use crate::material::Material;
use crate::shapes::{Shape, Shapes};
use crate::transform::Transform;

pub struct Object {
    pub shape: Shapes,
    pub tx: Transform,
    pub mat: Material,
    pub mass_data: MassData,
    pub kinematics: Kinematics,
    pub force: Vector2<f64>,
}

impl Object {
    pub fn new(
        mut shape: Shapes,
        tx: Transform,
        mat: Option<Material>,
        mass_data: Option<MassData>,
        kinematics: Option<Kinematics>,
    ) -> Self {
        let mat = mat.unwrap_or_else(|| {
            // Generate random material properties
            let density = rand::thread_rng().gen_range(0.1..10.0);
            let restitution = rand::thread_rng().gen_range(0.0..1.0);
            let dynamic_friction = rand::thread_rng().gen_range(0.0..1.0);
            let static_friction = rand::thread_rng().gen_range(0.0..1.0);

            Material::new(
                density,
                OrderedFloat(restitution),
                OrderedFloat(dynamic_friction),
                OrderedFloat(static_friction),
            )
        });

        let mass_data = mass_data.unwrap_or_else(|| shape.calculate_mass_data(mat.density));

        let kinematics = kinematics.unwrap_or_else(|| {
            // Generate random kinematics properties
            let vel = Vector2::new(
                rand::thread_rng().gen_range(-10.0..10.0),
                rand::thread_rng().gen_range(-10.0..10.0),
            );
            let angular_vel = rand::thread_rng().gen_range(-1.0..1.0);
            let torque = rand::thread_rng().gen_range(-1.0..1.0);

            Kinematics::new(vel, angular_vel, torque)
        });

        let force = Vector2::zeros();

        Object {
            shape,
            tx,
            mat,
            mass_data,
            kinematics,
            force,
        }
    }

    // fn apply_force(&mut self, f: &Vector2<f64>) {
    //     self.force += f;
    // }

    pub fn apply_impulse(&mut self, imp: &Vector2<f64>, contact_vec: &Vector2<f64>) {
        self.kinematics.vel += self.mass_data.inv_mass * imp;
        self.kinematics.angular_vel += self.mass_data.inv_m_inertia * cross_v_v(contact_vec, imp);
    }

    pub fn draw(&self, c: Context, gl: &mut GlGraphics) {
        self.shape.draw(c, gl, &self.tx);
    }
}
