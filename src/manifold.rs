use nalgebra::Vector2;
use std::cmp::{min, max};
use std::f64::EPSILON;
use ordered_float::OrderedFloat;
use std::cell::RefCell;
use std::rc::Rc;

use crate::object::{Object, ShapeTag};
use crate::types::{Meter, NormalizedCoefficient};
use crate::constants::{GRAVITY, PEN_ALLOWANCE, PERCENT_CORRECTION};
use crate::collision::{circle_circle, circle_polygon, polygon_polygon};

pub struct Manifold {
    pub a: Rc<RefCell<Object>>,
    pub b: Rc<RefCell<Object>>,
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

impl Manifold {
    pub fn new(a: Rc<RefCell<Object>>, b: Rc<RefCell<Object>>) -> Manifold {
        Manifold {
            a,
            b,
            penetration: OrderedFloat(0.0),
            normal: Vector2::zeros(),
            contacts: [Vector2::zeros(); 2],
            contact_count: 0,
            mixed_restitution: OrderedFloat(0.0),
            mixed_dynamic_friction: OrderedFloat(0.0),
            mixed_static_friction: OrderedFloat(0.0),
        }
    }

    pub fn solve(&mut self) {
        let a = self.a.borrow().shape.tag();
        let b = self.b.borrow().shape.tag();

        match (a, b) {
            (ShapeTag::Circle { .. }, ShapeTag::Circle { .. }) => circle_circle(self),
            (ShapeTag::Polygon { .. }, ShapeTag::Polygon { .. }) => polygon_polygon(self),
            _ => circle_polygon(self, None),
        };
    }

    pub fn initialize(&mut self, dt: f64) {
        self.mixed_restitution = min(self.a.borrow().mat.restitution, self.b.borrow().mat.restitution);
        self.mixed_dynamic_friction = OrderedFloat((self.a.borrow().mat.dynamic_friction * self.b.borrow().mat.dynamic_friction).sqrt());
        self.mixed_static_friction = OrderedFloat((self.a.borrow().mat.static_friction * self.b.borrow().mat.static_friction).sqrt());

        for i in 0..self.contact_count {
            let a_radii = self.contacts[i] - self.a.borrow().tx.pos.coords;
            let b_radii = self.contacts[i] - self.b.borrow().tx.pos.coords;
            let rel_vel = self.b.borrow().kinematics.vel + cross(self.b.borrow().kinematics.angular_vel, b_radii) - self.a.borrow().kinematics.vel - cross(self.a.borrow().kinematics.angular_vel, a_radii);

            if rel_vel.norm_squared() < (dt * GRAVITY).norm_squared() + EPSILON {
                self.mixed_restitution = OrderedFloat(0.0);
            }
        }
    }

    pub fn apply_impulse(&mut self) {
        if self.a.borrow().mass_data.mass.is_infinite() && self.b.borrow().mass_data.mass.is_infinite() {
            self.infinite_mass_correction();
            return;
        }

        for i in 0..self.contact_count {
            unimplemented!()
        }
    }

    pub fn positional_correction(&mut self) {
        let correction = *(max(self.penetration - PEN_ALLOWANCE, OrderedFloat(0.0)) / (self.a.borrow().mass_data.inv_mass + self.b.borrow().mass_data.inv_mass)) * self.normal * *PERCENT_CORRECTION;
        let a_inv_mass = self.a.borrow().mass_data.inv_mass;
        let b_inv_mass = self.b.borrow().mass_data.inv_mass;

        self.a.borrow_mut().tx.pos -= correction * a_inv_mass;
        self.b.borrow_mut().tx.pos += correction * b_inv_mass;
    }

    fn infinite_mass_correction(&mut self) {
        self.a.borrow_mut().kinematics.vel = Vector2::zeros();
        self.b.borrow_mut().kinematics.vel = Vector2::zeros();
    }
}