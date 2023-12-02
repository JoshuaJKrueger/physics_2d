use graphics::{Context, ellipse, line::Line, Transformed};
use opengl_graphics::GlGraphics;

use nalgebra::{Vector2, Matrix2, Point2};

use std::f64::consts::PI;

use crate::types::{Meter, KilogramPerCubicMeter};
use crate::constants::{WHITE, ONE_THIRD};
use crate::object::{MassData, Transform};
use crate::custom_math::cross_v_v;

pub trait Shape {
    fn calculate_mass_data(&mut self, density: KilogramPerCubicMeter) -> MassData;
    fn draw(&self, c: Context, gl: &mut GlGraphics, tx: &Transform);
    fn discriminant(&self) -> ShapeDiscriminant;
}

pub trait PolygonShape: Shape {
    fn setBoundingBox(&mut self, width: Meter, height: Meter);
    fn getRightMostPoint(&self);
    fn buildHull(&mut self);
    fn compute_norms(&mut self);
}

pub struct Circle { pub radius: Meter, }

impl Shape for Circle {
    fn calculate_mass_data(&mut self, density: KilogramPerCubicMeter) -> MassData {
        let m = PI * *self.radius * *self.radius * density;
        MassData::new(m, m * *self.radius * *self.radius)
    }

    fn draw(&self, c: Context, gl: &mut GlGraphics, tx: &Transform) {
        ellipse::Ellipse::new_border(WHITE, 1.0).draw(ellipse::circle(tx.pos.x, tx.pos.y, *self.radius), &c.draw_state, c.transform, gl);
    }

    fn discriminant(&self) -> ShapeDiscriminant {
        ShapeDiscriminant::Circle
    }
}

pub struct Polygon {
    pub orient: Matrix2<f64>,
    pub vertices: Vec<Point2<f64>>,
    pub normals: Vec<Vector2<f64>>,
}

impl Polygon {
    pub fn new(vertices: Vec<Point2<f64>>, orient: Option<Matrix2<f64>>) -> Self {
        let orient = orient.unwrap_or_else(|| Matrix2::identity());
        let normals = vec![];
        todo!("Build hull");
        
        Self { orient, vertices, normals }
    }
}

impl Shape for Polygon {
    fn calculate_mass_data(&mut self, density: KilogramPerCubicMeter) -> MassData {
        let mut centroid = Vector2::zeros();
        let mut area = 0.0;
        let mut mmi = 0.0;

        // Calculate area and centroid
        for (p1, p2) in self.vertices.iter().zip(self.vertices.iter().cycle().skip(1)) {
            // Get the signed area of the triangle formed by (0, 0), p1, and p2
            let signed_area = cross_v_v(&p1.coords, &p2.coords);
            let tri_area = 0.5 * signed_area;

            area += tri_area;
            // Accumulate the weighted centroid coordinates.
            centroid += tri_area * ONE_THIRD * (p1.coords + p2.coords);

            // Calculate squared integral terms for inertia calculation.
            let int_x_sqrd = p1.coords.x * p1.coords.x + p2.coords.x * p1.coords.x + p2.coords.x * p2.coords.x;
            let int_y_sqrd = p1.coords.y * p1.coords.y + p2.coords.y * p1.coords.y + p2.coords.y * p2.coords.y;

            mmi += (0.25 * ONE_THIRD * signed_area) * (int_x_sqrd + int_y_sqrd)
        }

        centroid *= 1.0 / area;

        // Translate vertices to be centered around the origin
        for vert in &mut self.vertices {
            vert.coords -= centroid;
        }

        MassData::new(density * area, mmi)
    }

    fn draw(&self, c: Context, gl: &mut GlGraphics, tx: &Transform) {
        // piston polygon only has a filled version
        for i in 0..self.vertices.len() {
            let next_index = (i + 1) % self.vertices.len();
            let start = [self.vertices[i].x, self.vertices[i].y];
            let end = [self.vertices[next_index].x, self.vertices[next_index].y];

            Line::new(WHITE, 1.0).draw([start[0], start[1], end[0], end[1]], &c.draw_state, c.transform.trans(tx.pos.x, tx.pos.y), gl);
        }
    }

    fn discriminant(&self) -> ShapeDiscriminant {
        ShapeDiscriminant::Polygon
    }
}

impl PolygonShape for Polygon {
    fn setBoundingBox(&mut self, width: Meter, height: Meter) {
        unimplemented!()
    }

    fn getRightMostPoint(&self) {
        unimplemented!()
    }

    fn buildHull(&mut self) {
        unimplemented!()
    }

    fn compute_norms(&mut self) {
        unimplemented!()
    }

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
            Shapes::Polygon(p) => p.draw(c, gl, tx)
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