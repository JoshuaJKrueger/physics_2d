use graphics::Context;
use nalgebra::Vector2;
use opengl_graphics::GlGraphics;
use std::cell::RefCell;
use std::rc::Rc;

use crate::constants::{GRAVITY, GREEN, RED};
use crate::manifold::Manifold;
use crate::object::Object;

/// Represents a physics scene with a collection of objects and contact manifolds.
pub struct Scene {
    /// A collection of objects in the scene, each wrapped in a `RefCell` and reference-counted `Rc`.
    pub objects: Vec<Rc<RefCell<Object>>>,
    /// A collection of contact manifolds representing interactions between objects.
    pub contacts: Vec<Manifold>,
}

impl Scene {
    /// Advances the simulation by a specified time step.
    ///
    /// # Arguments
    ///
    /// * `dt` - The time step.
    pub fn step(&mut self, dt: f64) {
        self.contacts.clear();

        for (i, a) in self.objects.iter().enumerate() {
            for b in self.objects.iter().skip(i + 1) {
                if a.borrow().mass_data.mass.is_infinite()
                    && b.borrow().mass_data.mass.is_infinite()
                {
                    continue;
                }

                let mut m = Manifold::new(Rc::clone(a), Rc::clone(b));
                m.solve();

                if m.contact_count > 0 {
                    self.contacts.push(m);
                }
            }
        }

        for obj in &self.objects {
            integrate_forces(&mut obj.borrow_mut(), dt);
        }

        for contact in &mut self.contacts {
            contact.initialize(dt);
        }

        for _ in 0..10 {
            for contact in &mut self.contacts {
                contact.apply_impulse();
            }
        }

        for obj in &self.objects {
            integrate_velocities(&mut obj.borrow_mut(), dt);
        }

        for contact in &mut self.contacts {
            contact.positional_correction();
        }

        for obj in &mut self.objects {
            obj.borrow_mut().force = Vector2::zeros();
            obj.borrow_mut().kinematics.torque = 0.0;
        }
    }

    /// Renders the scene, including objects and visualizations for contact points and normals.
    ///
    /// # Arguments
    ///
    /// * `c` - The graphics context.
    /// * `gl` - The OpenGL graphics object.
    pub fn render(&self, c: Context, gl: &mut GlGraphics) {
        for object in &self.objects {
            object.borrow().draw(c, gl);
        }

        // Visualize contact points as red points
        for manifold in &self.contacts {
            for contact in &manifold.contacts {
                let square = graphics::rectangle::square(contact.x - 2.5, contact.y - 2.5, 5.0);
                graphics::rectangle(RED, square, c.transform, gl);
            }
        }

        // Visualize contact normals as green lines
        for manifold in &self.contacts {
            for contact in &manifold.contacts {
                let normal = manifold.normal * 50.0;
                let line = graphics::line::Line::new(GREEN, 1.0);
                line.draw(
                    [
                        contact.x,
                        contact.y,
                        contact.x + normal.x,
                        contact.y + normal.y,
                    ],
                    &c.draw_state,
                    c.transform,
                    gl,
                );
            }
        }
    }
}

/// Semi-implicit Euler method for integrating forces over time.
///
/// # Arguments
///
/// * `obj` - The object to integrate forces for.
/// * `dt` - The time step.
fn integrate_forces(obj: &mut Object, dt: f64) {
    if obj.mass_data.mass.is_infinite() {
        return;
    }

    obj.kinematics.vel += (obj.force * obj.mass_data.inv_mass + GRAVITY) * (dt / 2.0);
    obj.kinematics.angular_vel += obj.kinematics.torque * obj.mass_data.inv_m_inertia * (dt / 2.0);
}

/// Semi-implicit Euler method for integrating velocities over time.
///
/// # Arguments
///
/// * `obj` - The object to integrate velocities for.
/// * `dt` - The time step.
fn integrate_velocities(obj: &mut Object, dt: f64) {
    if obj.mass_data.mass.is_infinite() {
        return;
    }

    obj.tx.pos += obj.kinematics.vel * dt;
    obj.tx.orientation += obj.kinematics.angular_vel * dt;
    integrate_forces(obj, dt);
}

#[cfg(test)]
mod tests {
    use super::*;

    use nalgebra::Point2;
    use ordered_float::OrderedFloat;

    use crate::{circle::Circle, shapes::Shapes, transform::Transform};

    #[test]
    fn test_integrate_forces() {
        let circle = Shapes::Circle(Circle {
            radius: OrderedFloat(1.0),
        });
        let tx = Transform::new(Point2::new(0.0, 0.0));
        let mut a = Object::new(circle, tx, None, None, None);
        let initial_vel = a.kinematics.vel.clone();
        let initial_angular_vel = a.kinematics.angular_vel;

        a.force = Vector2::new(1.0, 2.0);
        a.kinematics.torque = 3.0;

        let dt = 0.1;

        integrate_forces(&mut a, dt);

        assert_ne!(a.kinematics.vel, initial_vel);
        assert_ne!(a.kinematics.angular_vel, initial_angular_vel);
    }

    #[test]
    fn test_integrate_velocities() {
        let circle = Shapes::Circle(Circle {
            radius: OrderedFloat(1.0),
        });
        let tx = Transform::new(Point2::new(0.0, 0.0));
        let mut obj = Object::new(circle, tx, None, None, None);
        let initial_pos = obj.tx.pos;
        let initial_orientation = obj.tx.orientation;

        obj.kinematics.vel = Vector2::new(1.0, 2.0);
        obj.kinematics.angular_vel = 3.0;
        obj.force = Vector2::new(4.0, 5.0);
        obj.kinematics.torque = 6.0;

        let dt = 0.1;

        integrate_velocities(&mut obj, dt);

        assert_ne!(obj.tx.pos, initial_pos);
        assert_ne!(obj.tx.orientation, initial_orientation);
    }
}
