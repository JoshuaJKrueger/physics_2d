use graphics::Context;
use opengl_graphics::GlGraphics;

use crate::circle::Circle;
use crate::mass_data::MassData;
use crate::polygon::Polygon;
use crate::transform::Transform;
use crate::types::KilogramPerCubicMeter;

pub trait Shape {
    fn calculate_mass_data(&mut self, density: KilogramPerCubicMeter) -> MassData;
    fn draw(&self, c: Context, gl: &mut GlGraphics, tx: &Transform);
    fn discriminant(&self) -> ShapeDiscriminant;
}

pub enum Shapes {
    Circle(Circle),
    Polygon(Polygon),
}

impl Shape for Shapes {
    fn calculate_mass_data(&mut self, density: KilogramPerCubicMeter) -> MassData {
        match self {
            Shapes::Circle(c) => c.calculate_mass_data(density),
            Shapes::Polygon(p) => p.calculate_mass_data(density),
        }
    }

    fn draw(&self, c: Context, gl: &mut GlGraphics, tx: &Transform) {
        match self {
            Shapes::Circle(circ) => circ.draw(c, gl, tx),
            Shapes::Polygon(p) => p.draw(c, gl, tx),
        }
    }

    fn discriminant(&self) -> ShapeDiscriminant {
        match self {
            Shapes::Circle(c) => c.discriminant(),
            Shapes::Polygon(p) => p.discriminant(),
        }
    }
}

pub enum ShapeDiscriminant {
    Circle,
    Polygon,
}
