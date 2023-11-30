use std::f64::INFINITY;

use graphics::Context;
use opengl_graphics::GlGraphics;
use nalgebra::Vector2;

use crate::manifold::Manifold;
use crate::object::Object;
use crate::constants::GRAVITY;

// Semi-implicit Euler method
fn integrate_forces(obj: &mut Object, dt: f64) {
    if obj.mass_data.mass == INFINITY {
        return;
    }

    obj.kinematics.vel += (obj.force * obj.mass_data.inv_mass + GRAVITY) * (dt / 2.0);
    obj.kinematics.angular_vel += obj.kinematics.torque * obj.mass_data.inv_m_inertia * (dt / 2.0);
}

fn integrate_velocities(obj: &mut Object, dt: f64) {
    if obj.mass_data.mass == INFINITY {
        return;
    }

    obj.tx.pos += obj.kinematics.vel * dt;
    obj.tx.orientation += obj.kinematics.angular_vel * dt;
    integrate_forces(obj, dt);
}

pub struct Scene<'a> {
    pub objects: Vec<Object>,
    pub contacts: Vec<Manifold<'a>>,
}

impl<'a> Scene<'a> {
    pub fn step(&mut self, dt: f64) {
        self.contacts.clear();

        for (i, a) in self.objects.iter_mut().enumerate() {
            // for b in self.objects.iter_mut().skip(i + 1) {
            //     if a.mass_data.mass == INFINITY && b.mass_data.mass == INFINITY {
            //         continue;
            //     }

            //     let mut m = Manifold::new(a, b);
            //     m.solve();

            //     if m.contact_count > 0 {
            //         self.contacts.push(m);
            //     }
            // }
        }

        for obj in &mut self.objects {
            integrate_forces(obj, dt);
        }

        for contact in &mut self.contacts {
            contact.initialize(dt);
        }

        for _ in 0..10 {
            for contact in &mut self.contacts {
                contact.apply_impulse();
            }
        }

        for obj in &mut self.objects {
            integrate_velocities(obj, dt);
        }

        for contact in &mut self.contacts {
            contact.positional_correction();
        }

        for obj in &mut self.objects {
            obj.force = Vector2::zeros();
            obj.kinematics.torque = 0.0;
        }
    }

    pub fn render(&self, c: Context, gl: &mut GlGraphics) {
        for object in &self.objects {
            object.draw(c, gl);
        }

        const RED: [f32; 4] = [1.0, 0.0, 0.0, 1.0];
        const GREEN: [f32; 4] = [0.0, 1.0, 0.0, 1.0];

        // Visualize contact points as red points
        for manifold in &self.contacts {
            for contact in &manifold.contacts {
                let square = graphics::rectangle::square(contact.x - 2.0, contact.y - 2.0, 4.0);
                graphics::rectangle(RED, square, c.transform, gl);
            }
        }

        // Visualize contact normals as green lines
        for manifold in &self.contacts {
            for contact in &manifold.contacts {
                let normal = manifold.normal * 0.75; // Adjust the length of the normal
                let line = graphics::line::Line::new(GREEN, 1.0);
                line.draw([contact.x, contact.y, contact.x + normal.x, contact.y + normal.y], &c.draw_state, c.transform, gl);
            }
        }
    }
}