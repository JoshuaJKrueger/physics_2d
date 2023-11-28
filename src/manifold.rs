use nalgebra::Vector2;
use std::cmp::{min, max};
use std::f64::{EPSILON, INFINITY};
use ordered_float::OrderedFloat;

use crate::object::Object;
use crate::types::{Meter, NormalizedCoefficient};

pub struct Manifold<'a> {
    a: &'a mut Object,
    b: &'a mut Object,
    pub penetration: Meter,
    pub normal: Vector2<f64>,
    pub contacts: [Vector2<f64>; 2],
    pub contact_count: usize,
    mixed_restitution: NormalizedCoefficient,
    mixed_dynamic_friction: NormalizedCoefficient,
    mixed_static_friction: NormalizedCoefficient,
}

// nalgebra doesn't support
fn cross(a: f64, b: Vector2<f64>) -> Vector2<f64> {
    Vector2::new(-a * b.y, a * b.x)
}

impl<'a> Manifold<'a> {
    fn solve() {
        unimplemented!()
    }

    fn initialize(&mut self) {
        self.mixed_restitution = min(self.a.mat.restitution, self.b.mat.restitution);
        self.mixed_dynamic_friction = OrderedFloat((self.a.mat.dynamic_friction * self.b.mat.dynamic_friction).sqrt());
        self.mixed_static_friction = OrderedFloat((self.a.mat.static_friction * self.b.mat.static_friction).sqrt());

        for i in 0..self.contact_count {
            let a_radii = self.contacts[i] - self.a.tx.pos.coords;
            let b_radii = self.contacts[i] - self.b.tx.pos.coords;
            let rel_vel = self.b.kinematics.vel + cross(self.b.kinematics.angular_vel, b_radii) - self.a.kinematics.vel - cross(self.a.kinematics.angular_vel, a_radii);

            // if rel_vel.norm_squared() < (dt * gravity).norm_squared() + EPSILON {
            //     self.mixed_restitution = OrderedFloat(0.0);
            // }
        }
    }

    fn apply_impulse(&mut self) {
        if self.a.mass_data.mass == INFINITY && self.b.mass_data.mass == INFINITY {
            self.infinite_mass_correction();
            return;
        }

        for i in 0..self.contact_count {
            unimplemented!()
        }
    }

    fn positional_correction(&mut self) {
        const PEN_ALLOWANCE: NormalizedCoefficient = OrderedFloat(0.05);
        const PERCENT_CORRECTION: NormalizedCoefficient = OrderedFloat(0.4);
        let correction = *(max(self.penetration - PEN_ALLOWANCE, OrderedFloat(0.0)) / (self.a.mass_data.inv_mass + self.b.mass_data.inv_mass)) * self.normal * *PERCENT_CORRECTION;
        self.a.tx.pos -= correction * self.a.mass_data.inv_mass;
        self.b.tx.pos += correction * self.b.mass_data.inv_mass;
    }

    fn infinite_mass_correction(&mut self) {
        self.a.kinematics.vel = Vector2::zeros();
        self.b.kinematics.vel = Vector2::zeros();
    }
}