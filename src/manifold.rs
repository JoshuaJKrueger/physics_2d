use nalgebra::Vector2;
use ordered_float::OrderedFloat;
use std::cell::RefCell;
use std::cmp::{max, min};
use std::f64::EPSILON;
use std::rc::Rc;

use crate::collision::{circle_circle, circle_polygon, polygon_polygon};
use crate::constants::{GRAVITY, PEN_ALLOWANCE, PERCENT_CORRECTION};
use crate::custom_math::{cross_s_v, cross_v_v};
use crate::object::Object;
use crate::shapes::{Shape, ShapeDiscriminant};
use crate::types::{Meter, NormalizedCoefficient};

/// Represents a collision manifold between two objects.
pub struct Manifold {
    /// The first object involved in the collision.
    pub a: Rc<RefCell<Object>>,
    /// The second object involved in the collision.
    pub b: Rc<RefCell<Object>>,
    /// The penetration depth of the collision.
    pub penetration: Meter,
    /// The collision normal pointing from object A to object B.
    pub normal: Vector2<f64>,
    /// Array of contact points in world coordinates.
    pub contacts: [Vector2<f64>; 2],
    /// Number of valid contact points.
    pub contact_count: usize,
    /// Coefficient of restitution for the collision.
    mixed_restitution: NormalizedCoefficient,
    /// Coefficient of dynamic friction for the collision.
    mixed_dynamic_friction: NormalizedCoefficient,
    /// Coefficient of static friction for the collision.
    mixed_static_friction: NormalizedCoefficient,
}

impl Manifold {
    /// Creates a new `Manifold` instance representing a collision between two objects.
    ///
    /// # Arguments
    ///
    /// * `a` - The first object involved in the collision.
    /// * `b` - The second object involved in the collision.
    ///
    /// # Returns
    ///
    /// A new `Manifold` instance.
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

    /// Dispatches collision detection to the appropriate function based on the shape of a and b.
    pub fn solve(&mut self) {
        let a = self.a.borrow().shape.discriminant();
        let b = self.b.borrow().shape.discriminant();

        match (a, b) {
            (ShapeDiscriminant::Circle, ShapeDiscriminant::Circle) => circle_circle(self),
            (ShapeDiscriminant::Polygon, ShapeDiscriminant::Polygon) => polygon_polygon(self),
            (ShapeDiscriminant::Circle, ShapeDiscriminant::Polygon) => circle_polygon(self, true),
            _ => circle_polygon(self, false),
        };
    }

    /// Initializes the manifold properties based on the object materials and contact points.
    ///
    /// # Arguments
    ///
    /// * `dt` - The time step for the simulation.
    pub fn initialize(&mut self, dt: f64) {
        self.mixed_restitution = min(
            self.a.borrow().mat.restitution,
            self.b.borrow().mat.restitution,
        );
        self.mixed_dynamic_friction = OrderedFloat(
            (self.a.borrow().mat.dynamic_friction * self.b.borrow().mat.dynamic_friction).sqrt(),
        );
        self.mixed_static_friction = OrderedFloat(
            (self.a.borrow().mat.static_friction * self.b.borrow().mat.static_friction).sqrt(),
        );

        for i in 0..self.contact_count {
            let a_radii = self.contacts[i] - self.a.borrow().tx.pos.coords;
            let b_radii = self.contacts[i] - self.b.borrow().tx.pos.coords;
            let rel_vel = self.b.borrow().kinematics.vel
                + cross_s_v(self.b.borrow().kinematics.angular_vel, &b_radii)
                - self.a.borrow().kinematics.vel
                - cross_s_v(self.a.borrow().kinematics.angular_vel, &a_radii);

            if rel_vel.norm_squared() < (dt * GRAVITY).norm_squared() + EPSILON {
                self.mixed_restitution = OrderedFloat(0.0);
            }
        }
    }

    /// Applies impulse to resolve the collision.
    pub fn apply_impulse(&mut self) {
        if self.a.borrow().mass_data.mass.is_infinite()
            && self.b.borrow().mass_data.mass.is_infinite()
        {
            self.infinite_mass_correction();
            return;
        }

        for i in 0..self.contact_count {
            let ra = self.contacts[i] - self.a.borrow().tx.pos.coords;
            let rb = self.contacts[i] - self.b.borrow().tx.pos.coords;
            let rv = self.b.borrow().kinematics.vel
                + cross_s_v(self.b.borrow().kinematics.angular_vel, &rb)
                - self.a.borrow().kinematics.vel
                - cross_s_v(self.a.borrow().kinematics.angular_vel, &ra);
            let contact_vel = rv.dot(&self.normal);

            if contact_vel > 0.0 {
                return;
            }

            let ra_cross_n = cross_v_v(&ra, &self.normal);
            let rb_cross_n = cross_v_v(&rb, &self.normal);
            let inv_mass_sum = self.a.borrow().mass_data.inv_mass
                + self.b.borrow().mass_data.inv_mass
                + (ra_cross_n * ra_cross_n) * self.a.borrow().mass_data.inv_m_inertia
                + (rb_cross_n * rb_cross_n) * self.b.borrow().mass_data.inv_m_inertia;
            let mut imp_s = -(1.0 + *self.mixed_restitution) * contact_vel;

            imp_s /= inv_mass_sum;
            imp_s /= self.contact_count as f64;

            let imp = self.normal * imp_s;
            self.a.borrow_mut().apply_impulse(&-imp, &ra);
            self.b.borrow_mut().apply_impulse(&imp, &rb);

            // Fiction resolution
            // TODO
            // rv = self.b.borrow().kinematics.vel + cross_s_v(self.b.borrow().kinematics.angular_vel, &rb) - self.a.borrow().kinematics.vel - cross_s_v(self.a.borrow().kinematics.angular_vel, &ra);

            // let t = (rv - (self.normal * rv.dot(&self.normal))).normalize();
            // let mut tan_mag = -rv.dot(&t);

            // tan_mag /= inv_mass_sum;
            // tan_mag /= self.contact_count as f64;

            // if equal(tan_mag, 0.0) { return; }

            // let tan_imp = if tan_mag.abs() < imp_s * *self.mixed_static_friction { t * tan_mag }
            // else { t * -imp_s * *self.mixed_dynamic_friction };

            // self.a.borrow_mut().apply_impulse(&-tan_imp, &ra);
            // self.b.borrow_mut().apply_impulse(&tan_imp, &rb);
        }
    }

    /// Keeps objects from intersecting
    pub fn positional_correction(&mut self) {
        let correction = *(max(self.penetration - PEN_ALLOWANCE, OrderedFloat(0.0))
            / (self.a.borrow().mass_data.inv_mass + self.b.borrow().mass_data.inv_mass))
            * self.normal
            * *PERCENT_CORRECTION;
        let a_inv_mass = self.a.borrow().mass_data.inv_mass;
        let b_inv_mass = self.b.borrow().mass_data.inv_mass;

        self.a.borrow_mut().tx.pos -= correction * a_inv_mass;
        self.b.borrow_mut().tx.pos += correction * b_inv_mass;
    }

    /// When two objects with infinite mass collide, their velocities are set to zero
    fn infinite_mass_correction(&mut self) {
        self.a.borrow_mut().kinematics.vel = Vector2::zeros();
        self.b.borrow_mut().kinematics.vel = Vector2::zeros();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use nalgebra::Point2;

    use crate::{circle::Circle, kinematics::Kinematics, shapes::Shapes, transform::Transform};

    #[test]
    fn test_apply_impulse() {
        let circle1 = Shapes::Circle(Circle {
            radius: OrderedFloat(4.0),
        });
        let circle2 = Shapes::Circle(Circle {
            radius: OrderedFloat(4.0),
        });
        let tx1 = Transform::new(Point2::new(0.0, 0.0));
        let tx2 = Transform::new(Point2::new(0.0, 0.0));
        let a = RefCell::new(Object::new(
            circle1,
            tx1,
            None,
            None,
            Some(Kinematics::new(Vector2::new(10.0, 0.0), 0.0, 0.0)),
        ));
        let b = RefCell::new(Object::new(
            circle2,
            tx2,
            None,
            None,
            Some(Kinematics::new(Vector2::new(-10.0, 0.0), 0.0, 0.0)),
        ));
        let mut manifold = Manifold::new(Rc::new(a), Rc::new(b));
        let initial_vel_a = manifold.a.borrow().kinematics.vel.clone();
        let initial_vel_b = manifold.b.borrow().kinematics.vel.clone();

        manifold.solve();
        manifold.apply_impulse();

        assert_ne!(manifold.a.borrow().kinematics.vel, initial_vel_a);
        assert_ne!(manifold.b.borrow().kinematics.vel, initial_vel_b);
    }
}
