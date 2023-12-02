use nalgebra::{Vector2, distance_squared, Point2};
use std::f64::{NEG_INFINITY, EPSILON};

use crate::manifold::Manifold;
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
                else if s > separation { (i, s) }
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
        unimplemented!();
    }
}