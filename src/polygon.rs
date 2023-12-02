use graphics::{Context, line::Line, Transformed};
use opengl_graphics::GlGraphics;

use nalgebra::{Matrix2, Point2, Vector2};

use crate::types::KilogramPerCubicMeter;
use crate::mass_data::MassData;
use crate::transform::Transform;
use crate::shapes::{Shape, ShapeDiscriminant};
use crate::custom_math::cross_v_v;
use crate::constants::{ONE_THIRD, WHITE};

pub struct Polygon {
    pub orient: Matrix2<f64>,
    pub vertices: Vec<Point2<f64>>,
    pub normals: Vec<Vector2<f64>>,
}

impl Polygon {
    pub fn new(vertices: Vec<Point2<f64>>, orient: Option<Matrix2<f64>>) -> Result<Self, &'static str> {
        let orient = orient.unwrap_or_else(|| Matrix2::identity());

        // Ensure enough vertices to make polygon
        if vertices.len() <= 2 { return Err("Not enough vertices"); }

        let right_most = getRightMostPoint(vertices);

        
        Ok(Self { orient, vertices, normals })
    }

    fn setBoundingBox(&mut self) {
        unimplemented!()
    }

    // Finds the vertex in a polygon that has the maximum projection along a given direction.
    fn find_support(&self, dir: &Vector2<f64>) -> Point2<f64> {
        self.vertices.iter().max_by(|v1, v2| {
            v1.coords.dot(dir).partial_cmp(&v2.coords.dot(dir)).unwrap()
        }).map_or(Point2::new(0.0, 0.0), |v| *v)
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

fn getRightMostPoint(verts: &Vec<Point2<f64>) -> Point2<f64> {
    unimplemented!()
}

fn buildHull(&mut self) {
    unimplemented!()
}

fn compute_norms(&mut self) {
    unimplemented!()
}