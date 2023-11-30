use graphics::Context;
use opengl_graphics::GlGraphics;

use crate::manifold::Manifold;
use crate::object::Object;

pub struct Scene<'a> {
    pub objects: Vec<Object>,
    pub contacts: Vec<Manifold<'a>>,
}

impl<'a> Scene<'a> {
    pub fn step(&mut self, dt: f64) {
        // unimplemented!()
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