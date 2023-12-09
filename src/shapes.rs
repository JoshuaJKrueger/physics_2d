use graphics::Context;
use opengl_graphics::GlGraphics;

use crate::circle::Circle;
use crate::mass_data::MassData;
use crate::polygon::Polygon;
use crate::transform::Transform;
use crate::types::KilogramPerCubicMeter;

/// A trait representing a geometric shape with mass properties.
pub trait Shape {
    /// Calculates the mass data of the shape based on the given density.
    ///
    /// # Arguments
    ///
    /// * `density` - The density of the material the shape is composed of.
    fn calculate_mass_data(&mut self, density: KilogramPerCubicMeter) -> MassData;

    /// Draws the shape on the screen.
    ///
    /// # Arguments
    ///
    /// * `c` - The graphics context.
    /// * `gl` - The OpenGL graphics object.
    /// * `tx` - The transformation to apply to the shape.
    fn draw(&self, c: Context, gl: &mut GlGraphics, tx: &Transform);

    /// Retrieves the discriminant of the shape, indicating its type.
    fn discriminant(&self) -> ShapeDiscriminant;
}

/// An enumeration representing different geometric shapes.
pub enum Shapes {
    /// A circle shape.
    Circle(Circle),
    /// A polygon shape.
    Polygon(Polygon),
}

impl Shape for Shapes {
    /// Calculates the mass data of the shape based on the given density.
    fn calculate_mass_data(&mut self, density: KilogramPerCubicMeter) -> MassData {
        match self {
            Shapes::Circle(c) => c.calculate_mass_data(density),
            Shapes::Polygon(p) => p.calculate_mass_data(density),
        }
    }

    /// Draws the shape on the screen.
    fn draw(&self, c: Context, gl: &mut GlGraphics, tx: &Transform) {
        match self {
            Shapes::Circle(circ) => circ.draw(c, gl, tx),
            Shapes::Polygon(p) => p.draw(c, gl, tx),
        }
    }

    /// Retrieves the discriminant of the shape, indicating its type.
    fn discriminant(&self) -> ShapeDiscriminant {
        match self {
            Shapes::Circle(c) => c.discriminant(),
            Shapes::Polygon(p) => p.discriminant(),
        }
    }
}

/// An enumeration representing discriminants for different shape types.
pub enum ShapeDiscriminant {
    /// Indicates a circle shape.
    Circle,
    /// Indicates a polygon shape.
    Polygon,
}
