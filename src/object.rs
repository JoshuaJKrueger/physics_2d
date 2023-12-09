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

/// Represents a physical object in the simulation with shape, material, and kinematic properties.
pub struct Object {
    /// The shape of the object, such as a circle or polygon.
    pub shape: Shapes,
    /// The transform containing position and orientation information.
    pub tx: Transform,
    /// The material properties of the object affecting its physical behavior.
    pub mat: Material,
    /// The mass and moment of inertia data for the object.
    pub mass_data: MassData,
    /// The kinematic properties of the object, including velocity, angular velocity, and torque.
    pub kinematics: Kinematics,
    /// The force acting on the object.
    pub force: Vector2<f64>,
}

impl Object {
    /// Creates a new object with the specified shape, transform, material, mass data, and kinematics.
    ///
    /// If material, mass data, or kinematics are not provided, random values within certain ranges are generated.
    ///
    /// # Arguments
    ///
    /// * `shape` - The shape of the object.
    /// * `tx` - The transform containing position and orientation information.
    /// * `mat` - Optional material properties of the object. If not provided, random values are generated.
    /// * `mass_data` - Optional mass and moment of inertia data. If not provided, calculated based on the shape and material.
    /// * `kinematics` - Optional kinematic properties of the object. If not provided, random values are generated.
    ///
    /// # Returns
    ///
    /// A new `Object` instance with the specified properties.
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

    /// Applies an impulse to the object at a specific contact point.
    ///
    /// # Arguments
    ///
    /// * `imp` - The impulse vector to apply.
    /// * `contact_vec` - The vector from the object's center to the contact point.
    pub fn apply_impulse(&mut self, imp: &Vector2<f64>, contact_vec: &Vector2<f64>) {
        self.kinematics.vel += self.mass_data.inv_mass * imp;
        self.kinematics.angular_vel += self.mass_data.inv_m_inertia * cross_v_v(contact_vec, imp);
    }

    /// Dispatches the draw call to the specific shape implementation.
    ///
    /// # Arguments
    ///
    /// * `c` - The graphics context.
    /// * `gl` - The OpenGL graphics.
    pub fn draw(&self, c: Context, gl: &mut GlGraphics) {
        self.shape.draw(c, gl, &self.tx);
    }
}
