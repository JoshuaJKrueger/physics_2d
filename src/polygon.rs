use graphics::{line::Line, Context, Transformed};
use opengl_graphics::GlGraphics;

use nalgebra::{Matrix2, Point2, Vector2};
use std::f64::EPSILON;

use crate::constants::{ONE_THIRD, WHITE};
use crate::custom_math::cross_v_v;
use crate::mass_data::MassData;
use crate::shapes::{Shape, ShapeDiscriminant};
use crate::transform::Transform;
use crate::types::KilogramPerCubicMeter;

/// Represents a polygon in the simulation.
pub struct Polygon {
    /// The orientation matrix of the polygon.
    pub orient: Matrix2<f64>,
    /// The vertices of the polygon.
    pub vertices: Vec<Point2<f64>>,
    /// The normals of the polygon.
    pub normals: Vec<Vector2<f64>>,
}

impl Polygon {
    /// Creates a new polygon with the specified vertices and orientation.
    ///
    /// # Arguments
    ///
    /// * `vertices` - The vertices of the polygon.
    /// * `orient` - Optional orientation matrix. If not provided, the identity matrix is used.
    ///
    /// # Returns
    ///
    /// A new `Polygon` instance.
    pub fn new(vertices: Vec<Point2<f64>>, orient: Option<Matrix2<f64>>) -> Self {
        let orient = orient.unwrap_or_else(Matrix2::identity);

        // Ensure enough vertices to make polygon
        if vertices.len() <= 2 {
            todo!("Error")
        }

        // let right_most = get_right_most_vert_idx(&vertices);
        // TODO
        // let hull = build_hull(&vertices, right_most);
        let normals = compute_norms(&vertices);

        Self {
            orient,
            vertices,
            normals,
        }
    }

    // fn set_bounding_box(&mut self) {
    //     unimplemented!()
    // }

    /// Finds the support point of the polygon in the given direction.
    ///
    /// # Arguments
    ///
    /// * `dir` - The direction vector.
    ///
    /// # Returns
    ///
    /// The support point of the polygon in the given direction.
    pub fn find_support(&self, dir: &Vector2<f64>) -> Point2<f64> {
        self.vertices
            .iter()
            .max_by(|v1, v2| v1.coords.dot(dir).partial_cmp(&v2.coords.dot(dir)).unwrap())
            .map_or(Point2::new(0.0, 0.0), |v| *v)
    }
}

impl Shape for Polygon {
    // Adapted from https://code.tutsplus.com/series/how-to-create-a-custom-physics-engine--gamedev-12715
    /// Calculates the mass and moment of inertia data for the polygon.
    ///
    /// # Arguments
    ///
    /// * `density` - The density of the material the polygon is made of.
    ///
    /// # Returns
    ///
    /// A `MassData` structure containing the calculated mass and moment of inertia.
    fn calculate_mass_data(&mut self, density: KilogramPerCubicMeter) -> MassData {
        let mut centroid = Vector2::zeros();
        let mut area = 0.0;
        let mut mmi = 0.0;

        // Calculate area and centroid
        for (p1, p2) in self
            .vertices
            .iter()
            .zip(self.vertices.iter().cycle().skip(1))
        {
            // Get the signed area of the triangle formed by (0, 0), p1, and p2
            let signed_area = cross_v_v(&p1.coords, &p2.coords);
            let tri_area = 0.5 * signed_area;

            area += tri_area;
            // Accumulate the weighted centroid coordinates.
            centroid += tri_area * ONE_THIRD * (p1.coords + p2.coords);

            // Calculate squared integral terms for inertia calculation.
            let int_x_sqrd =
                p1.coords.x * p1.coords.x + p2.coords.x * p1.coords.x + p2.coords.x * p2.coords.x;
            let int_y_sqrd =
                p1.coords.y * p1.coords.y + p2.coords.y * p1.coords.y + p2.coords.y * p2.coords.y;

            mmi += (0.25 * ONE_THIRD * signed_area) * (int_x_sqrd + int_y_sqrd)
        }

        centroid *= 1.0 / area;

        // Translate vertices to be centered around the origin
        for vert in &mut self.vertices {
            vert.coords -= centroid;
        }

        MassData::new(density * -area, -mmi * density)
    }

    /// Draws the polygon using OpenGL graphics.
    ///
    /// # Arguments
    ///
    /// * `c` - The graphics context.
    /// * `gl` - The OpenGL graphics.
    /// * `tx` - The transform information.
    fn draw(&self, c: Context, gl: &mut GlGraphics, tx: &Transform) {
        // piston polygon only has a filled version
        for i in 0..self.vertices.len() {
            let next_index = (i + 1) % self.vertices.len();
            let start = [self.vertices[i].x, self.vertices[i].y];
            let end = [self.vertices[next_index].x, self.vertices[next_index].y];

            Line::new(WHITE, 1.0).draw(
                [start[0], start[1], end[0], end[1]],
                &c.draw_state,
                c.transform.trans(tx.pos.x, tx.pos.y),
                gl,
            );
        }
    }

    /// Returns the discriminant associated with the polygon shape.
    fn discriminant(&self) -> ShapeDiscriminant {
        ShapeDiscriminant::Polygon
    }
}

// fn get_right_most_vert_idx(verts: &[Point2<f64>]) -> usize {
//     let (right_most, _) =
//         verts
//             .iter()
//             .enumerate()
//             .fold((0, verts[0].x), |(right_most, highest_x), (i, vertex)| {
//                 if vertex.x > highest_x {
//                     (i, vertex.x)
//                 } else if vertex.x == highest_x && vertex.y < verts[right_most].y {
//                     (i, highest_x)
//                 } else {
//                     (right_most, highest_x)
//                 }
//             });

//     right_most
// }

// Adapted from https://code.tutsplus.com/series/how-to-create-a-custom-physics-engine--gamedev-12715
// fn build_hull(verts: &[Point2<f64>], right_most_idx: usize) -> Vec<Point2<f64>> {
//     let mut hull = Vec::new();
//     let mut index_hull = right_most_idx;

//     loop {
//         hull.push(verts[index_hull]);

//         let mut next_hull_index = 0;
//         for i in 1..verts.len() {
//             if next_hull_index == index_hull {
//                 next_hull_index = i;
//                 continue;
//             }

//             let e1 = verts[next_hull_index] - verts[hull.len() - 1];
//             let e2 = verts[i] - verts[hull.len() - 1];
//             let c = cross_v_v(&e1, &e2);

//             if c < 0.0 || (c == 0.0 && e2.norm_squared() > e1.norm_squared()) {
//                 next_hull_index = i;
//             }
//         }

//         if next_hull_index == right_most_idx {
//             break;
//         }

//         index_hull = next_hull_index;
//     }

//     hull
// }

/// Computes the normals for the edges of a polygon.
///
/// # Arguments
///
/// * `verts` - The vertices of the polygon.
///
/// # Returns
///
/// A vector containing the computed normals for the edges of the polygon.
fn compute_norms(verts: &[Point2<f64>]) -> Vec<Vector2<f64>> {
    verts
        .iter()
        .zip(verts.iter().cycle().skip(1))
        .map(|(&p1, &p2)| {
            let face = p2 - p1;
            assert!(face.norm_squared() > EPSILON * EPSILON);
            Vector2::new(face.y, -face.x).normalize()
        })
        .collect()
}
