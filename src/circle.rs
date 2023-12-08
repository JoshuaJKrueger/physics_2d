use graphics::{ellipse, Context};
use opengl_graphics::GlGraphics;

use std::f64::consts::PI;

use crate::constants::WHITE;
use crate::mass_data::MassData;
use crate::shapes::{Shape, ShapeDiscriminant};
use crate::transform::Transform;
use crate::types::{KilogramPerCubicMeter, Meter};

pub struct Circle {
    pub radius: Meter,
}

impl Shape for Circle {
    fn calculate_mass_data(&mut self, density: KilogramPerCubicMeter) -> MassData {
        let m = PI * *self.radius * *self.radius * density;
        MassData::new(m, m * *self.radius * *self.radius)
    }

    fn draw(&self, c: Context, gl: &mut GlGraphics, tx: &Transform) {
        ellipse::Ellipse::new_border(WHITE, 1.0).draw(
            ellipse::circle(tx.pos.x, tx.pos.y, *self.radius),
            &c.draw_state,
            c.transform,
            gl,
        );
    }

    fn discriminant(&self) -> ShapeDiscriminant {
        ShapeDiscriminant::Circle
    }
}
