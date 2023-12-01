use graphics::Context;
use opengl_graphics::GlGraphics;
use nalgebra::Vector2;
use std::cell::RefCell;
use std::rc::Rc;

use crate::manifold::Manifold;
use crate::object::Object;
use crate::constants::{GRAVITY, GREEN, RED};

// Semi-implicit Euler method
fn integrate_forces(obj: &mut Object, dt: f64) {
    if obj.mass_data.mass.is_infinite() { return; }

    obj.kinematics.vel += (obj.force * obj.mass_data.inv_mass + GRAVITY) * (dt / 2.0);
    obj.kinematics.angular_vel += obj.kinematics.torque * obj.mass_data.inv_m_inertia * (dt / 2.0);
}

fn integrate_velocities(obj: &mut Object, dt: f64) {
    if obj.mass_data.mass.is_infinite() { return; }

    obj.tx.pos += obj.kinematics.vel * dt;
    obj.tx.orientation += obj.kinematics.angular_vel * dt;
    integrate_forces(obj, dt);
}

pub struct Scene {
    pub objects: Vec<Rc<RefCell<Object>>>,
    pub contacts: Vec<Manifold>,
}

impl Scene {
    pub fn step(&mut self, dt: f64) {
        self.contacts.clear();

        for (i, a) in self.objects.iter().enumerate() {
            for b in self.objects.iter().skip(i + 1) {
                if a.borrow().mass_data.mass.is_infinite() && b.borrow().mass_data.mass.is_infinite() { continue; }

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
                line.draw([contact.x, contact.y, contact.x + normal.x, contact.y + normal.y], &c.draw_state, c.transform, gl);
            }
        }
    }
}