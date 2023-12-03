use nalgebra::{Vector2, distance_squared, Point2};
use ordered_float::OrderedFloat;
use std::f64::{NEG_INFINITY, EPSILON, INFINITY};

use crate::custom_math::bias_gt;
use crate::manifold::Manifold;
use crate::polygon::Polygon;
use crate::shapes::Shapes;

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

pub fn circle_polygon(manifold: &mut Manifold, circle_first: bool) {
    let (a, b) =
    if circle_first { (manifold.a.borrow(), manifold.b.borrow()) }
    else { (manifold.b.borrow(), manifold.a.borrow()) };

    if let (Shapes::Circle(c), Shapes::Polygon(p)) = (&a.shape, &b.shape) {
        manifold.contact_count = 0;

        // Transform circle to polygon model space
        let cen: Point2<f64> = (p.orient.transpose() * (a.tx.pos - b.tx.pos)).into();

        // Find min penetration edge
        let (face_norm, separation) = p.vertices.iter()
            .enumerate()
            .fold((0, NEG_INFINITY), |(face_norm, separation), (i, vertex)| {
                let s = p.normals[i].dot(&(cen - vertex));
                if s > *c.radius { (face_norm, separation) }
                else if s > separation { (i, -s) }
                else { (face_norm, separation) }
            });

        if separation > *c.radius { return; }

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
            let (vertex, dist) = if d1 <= 0.0 { (v1, distance_squared(&cen, &v1)) } else { (v2, distance_squared(&cen, &v2)) };

            if dist > *c.radius * *c.radius { return; }

            manifold.contact_count = 1;
            let mut norm = vertex - cen;
            norm = p.orient * norm;
            manifold.normal = norm.normalize();
            manifold.contacts[0] = p.orient * vertex.coords + b.tx.pos.coords;
        } else { // Near face
            let mut norm = p.normals[face_norm];
            if (cen - v1).dot(&norm) > *c.radius { return; }

            manifold.contact_count = 1;
            norm = p.orient * norm;
            manifold.normal = -norm;
            manifold.contacts[0] = manifold.normal * *c.radius + a.tx.pos.coords;
        }
    }
}

pub fn polygon_polygon(manifold: &mut Manifold) {
    let a = manifold.a.borrow();
    let b = manifold.b.borrow();

    if let (Shapes::Polygon(p1), Shapes::Polygon(p2)) = (&a.shape, &b.shape) {
        manifold.contact_count = 0;

        let (pen_a, face_a) = find_axis_least_pen(p1, p2, a.tx.pos, b.tx.pos);
        if pen_a >= 0.0 { return; }
        let (pen_b, face_b) = find_axis_least_pen(p2, p1, a.tx.pos, b.tx.pos);
        if pen_b >= 0.0 { return; }

        let (mut ref_idx, flip, ref_poly, inc_poly) =
            if bias_gt(pen_a, pen_b) { (face_a, false, p1, p2) }
            else { (face_b, true, p2, p1) };
        
        let mut incident_face = [Vector2::zeros(); 2];
        find_incident_face(&mut incident_face, ref_poly, inc_poly, ref_idx, if flip { a.tx.pos } else { b.tx.pos });

        let mut v1 = ref_poly.vertices[ref_idx];
        ref_idx = if ref_idx + 1 == ref_poly.vertices.len() { 0 } else { ref_idx + 1 };
        let mut v2 = ref_poly.vertices[ref_idx];

        v1 = ref_poly.orient * v1 + if flip { b.tx.pos.coords } else { a.tx.pos.coords };
        v2 = ref_poly.orient * v2 + if flip { b.tx.pos.coords } else { a.tx.pos.coords };

        let side_plan_norm = (v2 - v1).normalize();
        let ref_face_norm = Vector2::new(side_plan_norm.y, -side_plan_norm.x);
        let ref_c = ref_face_norm.dot(&v1.coords);
        let neg_side = -side_plan_norm.dot(&v1.coords);
        let pos_side = side_plan_norm.dot(&v2.coords);

        if clip(-side_plan_norm, neg_side, &mut incident_face) < 2 || clip(side_plan_norm, pos_side, &mut incident_face) < 2 { return; }

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

fn find_axis_least_pen(a: &Polygon, b: &Polygon, a_pos: Point2<f64>, b_pos: Point2<f64>) -> (f64, usize) {
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

fn find_incident_face(v: &mut [Vector2<f64>; 2], ref_poly: &Polygon, inc_poly: &Polygon, ref_idx: usize, inc_pos: Point2<f64>) {
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
    inc_face_idx = if inc_face_idx + 1 >= inc_poly.vertices.len() { 0 } else { inc_face_idx + 1 };
    v[1] = inc_poly.orient * inc_poly.vertices[inc_face_idx].coords + inc_pos.coords;
}

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