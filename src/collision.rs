use nalgebra::{distance_squared, Point2, Vector2};
use ordered_float::OrderedFloat;
use std::f64::{EPSILON, INFINITY, NEG_INFINITY};

use crate::custom_math::bias_gt;
use crate::manifold::Manifold;
use crate::polygon::Polygon;
use crate::shapes::Shapes;

/// Handles collision between two circles and updates the manifold.
///
/// # Arguments
///
/// * `manifold` - A mutable reference to the collision manifold.
pub fn circle_circle(manifold: &mut Manifold) {
    let a = manifold.a.borrow();
    let b = manifold.b.borrow();

    if let (Shapes::Circle(c1), Shapes::Circle(c2)) = (&a.shape, &b.shape) {
        let norm = b.tx.pos - a.tx.pos;
        let dist_sqr = norm.norm_squared();
        let rad = c1.radius + c2.radius;

        if dist_sqr >= *rad * *rad {
            manifold.contact_count = 0;
            return;
        }

        manifold.contact_count = 1;

        let dist = dist_sqr.sqrt();
        if dist == 0.0 {
            manifold.penetration = c1.radius;
            manifold.normal = Vector2::new(1.0, 0.0);
            manifold.contacts[0] = a.tx.pos.coords;
        } else {
            manifold.penetration = rad - dist;
            manifold.normal = norm / dist;
            manifold.contacts[0] = manifold.normal * *c2.radius + a.tx.pos.coords;
        }
    }
}

/// Handles collision between a circle and a polygon and updates the manifold.
///
/// # Arguments
///
/// * `manifold` - A mutable reference to the collision manifold.
/// * `circle_first` - A boolean indicating whether the circle is the first shape in the collision check.
pub fn circle_polygon(manifold: &mut Manifold, circle_first: bool) {
    let (a, b) = if circle_first {
        (manifold.a.borrow(), manifold.b.borrow())
    } else {
        (manifold.b.borrow(), manifold.a.borrow())
    };

    if let (Shapes::Circle(c), Shapes::Polygon(p)) = (&a.shape, &b.shape) {
        manifold.contact_count = 0;

        // Transform circle to polygon model space
        let cen: Point2<f64> = (p.orient.transpose() * (a.tx.pos - b.tx.pos)).into();

        // Find min penetration edge
        let (face_norm, separation) = p.vertices.iter().enumerate().fold(
            (0, NEG_INFINITY),
            |(face_norm, separation), (i, vertex)| {
                let s = p.normals[i].dot(&(cen - vertex));
                if s > *c.radius {
                    (face_norm, separation)
                } else if s > separation {
                    (i, -s)
                } else {
                    (face_norm, separation)
                }
            },
        );

        if separation > *c.radius {
            return;
        }

        // Get the face's verts
        let v1 = p.vertices[face_norm];
        let v2 = p.vertices[(face_norm + 1) % p.vertices.len()];

        // Check if cen is in polygon
        if separation < EPSILON {
            manifold.contact_count = 1;
            manifold.normal = -(p.orient * p.normals[face_norm]);
            manifold.contacts[0] = manifold.normal * *c.radius + a.tx.pos.coords;
            manifold.penetration = c.radius;
            return;
        }

        // Determine the Voronoi region cen is in
        let d1 = (cen - v1).dot(&(v2 - v1));
        let d2 = (cen - v2).dot(&(v1 - v2));

        manifold.penetration = c.radius - separation;

        // Near v1 or v2
        if d1 <= 0.0 || d2 <= 0.0 {
            let (vertex, dist) = if d1 <= 0.0 {
                (v1, distance_squared(&cen, &v1))
            } else {
                (v2, distance_squared(&cen, &v2))
            };

            if dist > *c.radius * *c.radius {
                return;
            }

            manifold.contact_count = 1;
            let mut norm = vertex - cen;
            norm = p.orient * norm;
            manifold.normal = norm.normalize();
            manifold.contacts[0] = p.orient * vertex.coords + b.tx.pos.coords;
        } else {
            // Near face
            let mut norm = p.normals[face_norm];
            if (cen - v1).dot(&norm) > *c.radius {
                return;
            }

            manifold.contact_count = 1;
            norm = p.orient * norm;
            manifold.normal = -norm;
            manifold.contacts[0] = manifold.normal * *c.radius + a.tx.pos.coords;
        }
    }
}

/// Handles collision between two polygons and updates the manifold.
///
/// # Arguments
///
/// * `manifold` - A mutable reference to the collision manifold.
pub fn polygon_polygon(manifold: &mut Manifold) {
    let a = manifold.a.borrow();
    let b = manifold.b.borrow();

    if let (Shapes::Polygon(p1), Shapes::Polygon(p2)) = (&a.shape, &b.shape) {
        manifold.contact_count = 0;

        let (pen_a, face_a) = find_axis_least_pen(p1, p2, a.tx.pos, b.tx.pos);
        if pen_a >= 0.0 {
            return;
        }
        let (pen_b, face_b) = find_axis_least_pen(p2, p1, a.tx.pos, b.tx.pos);
        if pen_b >= 0.0 {
            return;
        }

        let (mut ref_idx, flip, ref_poly, inc_poly) = if bias_gt(pen_a, pen_b) {
            (face_a, false, p1, p2)
        } else {
            (face_b, true, p2, p1)
        };

        let mut incident_face = [Vector2::zeros(); 2];
        find_incident_face(
            &mut incident_face,
            ref_poly,
            inc_poly,
            ref_idx,
            if flip { a.tx.pos } else { b.tx.pos },
        );

        let mut v1 = ref_poly.vertices[ref_idx];
        ref_idx = if ref_idx + 1 == ref_poly.vertices.len() {
            0
        } else {
            ref_idx + 1
        };
        let mut v2 = ref_poly.vertices[ref_idx];

        v1 = ref_poly.orient * v1
            + if flip {
                b.tx.pos.coords
            } else {
                a.tx.pos.coords
            };
        v2 = ref_poly.orient * v2
            + if flip {
                b.tx.pos.coords
            } else {
                a.tx.pos.coords
            };

        let side_plan_norm = (v2 - v1).normalize();
        let ref_face_norm = Vector2::new(side_plan_norm.y, -side_plan_norm.x);
        let ref_c = ref_face_norm.dot(&v1.coords);
        let neg_side = -side_plan_norm.dot(&v1.coords);
        let pos_side = side_plan_norm.dot(&v2.coords);

        if clip(-side_plan_norm, neg_side, &mut incident_face) < 2
            || clip(side_plan_norm, pos_side, &mut incident_face) < 2
        {
            return;
        }

        manifold.normal = if flip { -ref_face_norm } else { ref_face_norm };

        let mut cp = 0;
        let mut separation = ref_face_norm.dot(&incident_face[0]) - ref_c;

        if separation <= 0.0 {
            manifold.contacts[cp] = incident_face[0];
            manifold.penetration = OrderedFloat(-separation);
            cp += 1;
        } else {
            manifold.penetration = OrderedFloat(0.0);
        }

        separation = ref_face_norm.dot(&incident_face[1]) - ref_c;

        if separation <= 0.0 {
            manifold.contacts[cp] = incident_face[1];
            manifold.penetration += -separation;
            cp += 1;
            manifold.penetration /= cp as f64;
        }

        manifold.contact_count = cp;
    }
}

/// Finds the axis of least penetration between two polygons.
///
/// # Arguments
///
/// * `a` - The first polygon.
/// * `b` - The second polygon.
/// * `a_pos` - The position of the first polygon.
/// * `b_pos` - The position of the second polygon.
///
/// # Returns
///
/// A tuple containing the penetration depth and the index of the least penetrating face.
fn find_axis_least_pen(
    a: &Polygon,
    b: &Polygon,
    a_pos: Point2<f64>,
    b_pos: Point2<f64>,
) -> (f64, usize) {
    let mut best_dist = NEG_INFINITY;
    let mut best_idx = 0;

    for i in 0..a.vertices.len() {
        let n = b.orient.transpose() * (a.orient * a.normals[i]);
        let s = b.find_support(&-n);
        let mut v = a.vertices[i];
        v = a.orient * v + a_pos.coords;
        v -= b_pos.coords;
        v = b.orient.transpose() * v;

        let d = n.dot(&(s - v));

        if d > best_dist {
            best_dist = d;
            best_idx = i;
        }
    }

    (best_dist, best_idx)
}

/// Finds the incident face on the second polygon given the reference and incident polygons.
///
/// # Arguments
///
/// * `v` - A mutable array to store the incident face vertices.
/// * `ref_poly` - The reference polygon.
/// * `inc_poly` - The incident polygon.
/// * `ref_idx` - The index of the reference face.
/// * `inc_pos` - The position of the incident polygon.
fn find_incident_face(
    v: &mut [Vector2<f64>; 2],
    ref_poly: &Polygon,
    inc_poly: &Polygon,
    ref_idx: usize,
    inc_pos: Point2<f64>,
) {
    let mut ref_norm = ref_poly.normals[ref_idx];

    ref_norm = ref_poly.orient * ref_norm;
    ref_norm = inc_poly.orient.transpose() * ref_norm;

    let mut inc_face_idx = 0;
    let mut min_dot = INFINITY;

    for i in 0..inc_poly.vertices.len() {
        let dot = ref_norm.dot(&inc_poly.normals[i]);

        if dot < min_dot {
            min_dot = dot;
            inc_face_idx = i;
        }
    }

    v[0] = inc_poly.orient * inc_poly.vertices[inc_face_idx].coords + inc_pos.coords;
    inc_face_idx = if inc_face_idx + 1 >= inc_poly.vertices.len() {
        0
    } else {
        inc_face_idx + 1
    };
    v[1] = inc_poly.orient * inc_poly.vertices[inc_face_idx].coords + inc_pos.coords;
}

/// Clips a line segment against a plane defined by a normal vector and a constant value.
///
/// # Arguments
///
/// * `n` - The normal vector of the plane.
/// * `c` - The constant value representing the distance from the origin.
/// * `face` - A mutable array containing the line segment vertices to be clipped.
///
/// # Returns
///
/// The number of vertices after clipping.
fn clip(n: Vector2<f64>, c: f64, face: &mut [Vector2<f64>; 2]) -> usize {
    let mut sp = 0;
    let mut out = [face[0], face[1]];
    let d1 = n.dot(&face[0]) - c;
    let d2 = n.dot(&face[1]) - c;

    if d1 <= 0.0 {
        out[sp] = face[0];
        sp += 1;
    }

    if d2 <= 0.0 {
        out[sp] = face[1];
        sp += 1;
    }

    if d1 * d2 < 0.0 {
        let alpha = d1 / (d1 - d2);
        out[sp] = face[0] + alpha * (face[1] - face[0]);
        sp += 1;
    }

    face[0] = out[0];
    face[1] = out[1];

    assert!(sp != 3);

    sp
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::{cell::RefCell, rc::Rc};

    use nalgebra::Matrix2;

    use crate::{circle::Circle, object::Object, transform::Transform};

    #[test]
    fn test_circle_circle_no_collision() {
        let circle1 = Shapes::Circle(Circle {
            radius: OrderedFloat(1.0),
        });
        let circle2 = Shapes::Circle(Circle {
            radius: OrderedFloat(1.0),
        });
        let tx1 = Transform::new(Point2::new(0.0, 0.0));
        let tx2 = Transform::new(Point2::new(5.0, 0.0));
        let a = RefCell::new(Object::new(circle1, tx1, None, None, None));
        let b = RefCell::new(Object::new(circle2, tx2, None, None, None));
        let mut manifold = Manifold::new(Rc::new(a), Rc::new(b));

        circle_circle(&mut manifold);

        assert_eq!(manifold.contact_count, 0);
    }

    #[test]
    fn test_circle_circle_collision() {
        let circle1 = Shapes::Circle(Circle {
            radius: OrderedFloat(2.0),
        });
        let circle2 = Shapes::Circle(Circle {
            radius: OrderedFloat(2.0),
        });
        let tx1 = Transform::new(Point2::new(0.0, 0.0));
        let tx2 = Transform::new(Point2::new(3.0, 0.0));
        let a = RefCell::new(Object::new(circle1, tx1, None, None, None));
        let b = RefCell::new(Object::new(circle2, tx2, None, None, None));
        let mut manifold = Manifold::new(Rc::new(a), Rc::new(b));

        circle_circle(&mut manifold);

        assert_eq!(manifold.contact_count, 1);
    }

    #[test]
    fn test_circle_polygon_no_collision() {
        let circle = Shapes::Circle(Circle {
            radius: OrderedFloat(2.0),
        });
        let polygon = Shapes::Polygon(Polygon {
            vertices: vec![
                Point2::new(0.0, 0.0),
                Point2::new(4.0, 0.0),
                Point2::new(4.0, 4.0),
                Point2::new(0.0, 4.0),
            ],
            normals: vec![
                Vector2::new(0.0, -1.0),
                Vector2::new(1.0, 0.0),
                Vector2::new(0.0, 1.0),
                Vector2::new(-1.0, 0.0),
            ],
            orient: Matrix2::identity(),
        });

        let tx_circle = Transform::new(Point2::new(10.0, 10.0));
        let tx_polygon = Transform::new(Point2::new(0.0, 0.0));

        let a = RefCell::new(Object::new(circle, tx_circle, None, None, None));
        let b = RefCell::new(Object::new(polygon, tx_polygon, None, None, None));

        let mut manifold = Manifold::new(Rc::new(a), Rc::new(b));
        circle_polygon(&mut manifold, true);

        assert_eq!(manifold.contact_count, 0);
    }

    #[test]
    fn test_circle_polygon_collision() {
        let circle = Shapes::Circle(Circle {
            radius: OrderedFloat(4.0),
        });
        let polygon = Shapes::Polygon(Polygon {
            vertices: vec![
                Point2::new(0.0, 0.0),
                Point2::new(4.0, 0.0),
                Point2::new(4.0, 4.0),
                Point2::new(0.0, 4.0),
            ],
            normals: vec![
                Vector2::new(0.0, -1.0),
                Vector2::new(1.0, 0.0),
                Vector2::new(0.0, 1.0),
                Vector2::new(-1.0, 0.0),
            ],
            orient: Matrix2::identity(),
        });

        let tx_circle = Transform::new(Point2::new(0.0, 0.0));
        let tx_polygon = Transform::new(Point2::new(0.0, 0.0));

        let a = RefCell::new(Object::new(circle, tx_circle, None, None, None));
        let b = RefCell::new(Object::new(polygon, tx_polygon, None, None, None));

        let mut manifold = Manifold::new(Rc::new(a), Rc::new(b));
        circle_polygon(&mut manifold, true);

        assert_eq!(manifold.contact_count, 1);
        assert!(manifold.penetration > OrderedFloat(0.0));
    }

    #[test]
    fn test_polygon_polygon_no_collision() {
        let polygon1 = Shapes::Polygon(Polygon {
            vertices: vec![
                Point2::new(0.0, 0.0),
                Point2::new(4.0, 0.0),
                Point2::new(4.0, 4.0),
                Point2::new(0.0, 4.0),
            ],
            normals: vec![
                Vector2::new(0.0, -1.0),
                Vector2::new(1.0, 0.0),
                Vector2::new(0.0, 1.0),
                Vector2::new(-1.0, 0.0),
            ],
            orient: Matrix2::identity(),
        });

        let polygon2 = Shapes::Polygon(Polygon {
            vertices: vec![
                Point2::new(5.0, 5.0),
                Point2::new(9.0, 5.0),
                Point2::new(9.0, 9.0),
                Point2::new(5.0, 9.0),
            ],
            normals: vec![
                Vector2::new(0.0, -1.0),
                Vector2::new(1.0, 0.0),
                Vector2::new(0.0, 1.0),
                Vector2::new(-1.0, 0.0),
            ],
            orient: Matrix2::identity(),
        });

        let tx_polygon1 = Transform::new(Point2::new(0.0, 0.0));
        let tx_polygon2 = Transform::new(Point2::new(10.0, 10.0));

        let a = RefCell::new(Object::new(polygon1, tx_polygon1, None, None, None));
        let b = RefCell::new(Object::new(polygon2, tx_polygon2, None, None, None));

        let mut manifold = Manifold::new(Rc::new(a), Rc::new(b));
        polygon_polygon(&mut manifold);

        assert_eq!(manifold.contact_count, 0);
    }

    #[test]
    fn test_polygon_polygon_collision() {
        let polygon1 = Shapes::Polygon(Polygon {
            vertices: vec![
                Point2::new(0.0, 0.0),
                Point2::new(4.0, 0.0),
                Point2::new(4.0, 4.0),
                Point2::new(0.0, 4.0),
            ],
            normals: vec![
                Vector2::new(0.0, -1.0),
                Vector2::new(1.0, 0.0),
                Vector2::new(0.0, 1.0),
                Vector2::new(-1.0, 0.0),
            ],
            orient: Matrix2::identity(),
        });

        let polygon2 = Shapes::Polygon(Polygon {
            vertices: vec![
                Point2::new(2.0, 2.0),
                Point2::new(6.0, 2.0),
                Point2::new(6.0, 6.0),
                Point2::new(2.0, 6.0),
            ],
            normals: vec![
                Vector2::new(0.0, -1.0),
                Vector2::new(1.0, 0.0),
                Vector2::new(0.0, 1.0),
                Vector2::new(-1.0, 0.0),
            ],
            orient: Matrix2::identity(),
        });

        let tx_polygon1 = Transform::new(Point2::new(0.0, 0.0));
        let tx_polygon2 = Transform::new(Point2::new(0.0, 0.0));

        let a = RefCell::new(Object::new(polygon1, tx_polygon1, None, None, None));
        let b = RefCell::new(Object::new(polygon2, tx_polygon2, None, None, None));

        let mut manifold = Manifold::new(Rc::new(a), Rc::new(b));
        polygon_polygon(&mut manifold);

        assert!(manifold.contact_count > 0);
        assert!(manifold.penetration > OrderedFloat(0.0));
    }
}
