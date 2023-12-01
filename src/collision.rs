use nalgebra::Vector2;
use std::f64::{NEG_INFINITY, EPSILON};

use crate::manifold::Manifold;
use crate::object::Shape;

pub fn circle_circle(manifold: &mut Manifold) {
    let a = manifold.a.borrow();
    let b = manifold.b.borrow();

    if let (Shape::Circle { radius: rad_a }, Shape::Circle { radius: rad_b }) = (&a.shape, &b.shape) {
        let norm = b.tx.pos - a.tx.pos;
        let dist_sqr = norm.norm_squared();
        let rad = *rad_a + *rad_b;
    
        if dist_sqr >= *rad * *rad {
            manifold.contact_count = 0;
            return;
        }
    
        manifold.contact_count = 1;
        
        let dist = dist_sqr.sqrt();
        if dist == 0.0 {
            manifold.penetration = *rad_a;
            manifold.normal = Vector2::new(1.0, 0.0);
            manifold.contacts[0] = a.tx.pos.coords;
        } else {
            manifold.penetration = rad - dist;
            manifold.normal = norm / dist;
            manifold.contacts[0] = manifold.normal * **rad_b + a.tx.pos.coords;
        }
    }
}

pub fn circle_polygon(manifold: &mut Manifold, circle_first: bool) {
    let (a, b) =
    if circle_first { (manifold.a.borrow(), manifold.b.borrow()) }
    else { (manifold.b.borrow(), manifold.a.borrow()) };

    if let (Shape::Circle { radius }, Shape::Polygon { vertices, orient, normals  }) = (&a.shape, &b.shape) {
        manifold.contact_count = 0;

        // Transform circle to polygon model space
        let cen = orient.transpose() * (a.tx.pos.coords - b.tx.pos.coords);

        // Find min penetration edge
        let (face_norm, separation) = vertices.iter()
            .enumerate()
            .fold((0, NEG_INFINITY), |(face_norm, separation), (i, vertex)| {
                let s = normals[i].dot(&(cen - vertex.coords));
                if s > **radius { (face_norm, separation) }
                else if s > separation { (i, s) }
                else { (face_norm, separation) }
            });

        if separation > **radius { return; }

        // Get the face's verts
        let v1 = vertices[face_norm].coords;
        let v2 = vertices[(face_norm + 1) % vertices.len()].coords;

        // Check if cen is in polygon
        if separation < EPSILON {
            manifold.contact_count = 1;
            manifold.normal = -(orient * normals[face_norm]);
            manifold.contacts[0] = manifold.normal * **radius + a.tx.pos.coords;
            manifold.penetration = *radius;
            return;
        }

        // Determine the Voronoi region cen is in
        let d1 = (cen - v1).dot(&(v2 - v1));
        let d2 = (cen - v2).dot(&(v1 - v2));

        manifold.penetration = radius - separation;

        // Near v1 or v2
        if d1 <= 0.0 || d2 <= 0.0 {
            let (vertex, dist) = if d1 <= 0.0 { (v1, distsqr(cen, v1)) } else { (v2, distsqr(cen, v2)) };

            if dist > *radius * *radius { return; }

            manifold.contact_count = 1;
            let mut norm = vertex - cen;
            norm = orient * norm;
            manifold.normal = norm.normalize();
            manifold.contacts[0] = orient * vertex + b.tx.pos.coords;
        } else { // Near face
            let mut norm = normals[face_norm];
            if (cen - v1).dot(&norm) > **radius { return; }

            manifold.contact_count = 1;
            norm = orient * norm;
            manifold.normal = -norm;
            manifold.contacts[0] = manifold.normal * **radius + a.tx.pos.coords;
        }
    }
}

pub fn polygon_polygon(manifold: &mut Manifold) {
    let a = manifold.a.borrow();
    let b = manifold.b.borrow();

    if let (Shape::Polygon { vertices: verts_a, orient: orient_a, normals: norms_a }, Shape::Polygon { vertices: verts_b, orient: orient_b, normals: norms_b }) = (&a.shape, &b.shape) {
        unimplemented!();
    }
}