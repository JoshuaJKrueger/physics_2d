use graphics::{ellipse, Context};
use opengl_graphics::GlGraphics;

use std::f64::consts::PI;

use crate::constants::WHITE;
use crate::mass_data::MassData;
use crate::shapes::{Shape, ShapeDiscriminant};
use crate::transform::Transform;
use crate::types::{KilogramPerCubicMeter, Meter};

/// Represents a circular shape for physics simulations.
pub struct Circle {
    /// The radius of the circle, in meters.
    pub radius: Meter,
}

impl Shape for Circle {
    /// Calculates the mass and moment of inertia of the circle.
    ///
    /// # Parameters
    ///
    /// * `density`: The density of the material, in kilograms per cubic meter.
    ///
    /// # Returns
    ///
    /// A `MassData` struct containing the mass and moment of inertia.
    fn calculate_mass_data(&mut self, density: KilogramPerCubicMeter) -> MassData {
        let m = PI * *self.radius * *self.radius * density;
        MassData::new(m, m * *self.radius * *self.radius)
    }

    /// Draws the circle on the screen.
    ///
    /// # Parameters
    ///
    /// * `c`: The graphics context.
    /// * `gl`: The OpenGL graphics context.
    /// * `tx`: The transformation to apply to the shape.
    fn draw(&self, c: Context, gl: &mut GlGraphics, tx: &Transform) {
        ellipse::Ellipse::new_border(WHITE, 1.0).draw(
            ellipse::circle(tx.pos.x, tx.pos.y, *self.radius),
            &c.draw_state,
            c.transform,
            gl,
        );
    }

    /// Returns the unique identifier for the circle shape.
    ///
    /// # Returns
    ///
    /// A `ShapeDiscriminant::Circle` enum value.
    fn discriminant(&self) -> ShapeDiscriminant {
        ShapeDiscriminant::Circle
    }
}
