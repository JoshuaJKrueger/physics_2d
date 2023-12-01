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
        let mut cen = orient.transpose() * (a.tx.pos.coords - b.tx.pos.coords);

        // Find min penetration edge
        let mut separation = NEG_INFINITY;
        let mut face_norm = 0;
        for i in 0..vertices.len() {
            let s = normals[i].dot(&(cen - vertices[i].coords));

            if s > **radius { return; }

            if s > separation {
                separation = s;
                face_norm = i;
            }
        }

        // Get the face's verts
        let mut v1 = vertices[face_norm].coords;
        let i = if face_norm + 1 < vertices.len() { face_norm + 1 } else { 0 };
        let mut v2 = vertices[i].coords;

        // Check if cen is in polygon
        if separation < EPSILON
        {
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

        // Near v1
        if d1 <= 0.0 {
            if distsqr(cen, v1) > *radius * *radius { return; }

            manifold.contact_count = 1;
            let mut norm = v1 - cen;
            norm = orient * norm;
            manifold.normal = norm.normalize();
            v1 = orient * v1 + b.tx.pos.coords;
            manifold.contacts[0] = v1;
        } else if d2 <= 0.0 { // Near v2
            if distsqr(cen, v2) > *radius * *radius { return; }

            manifold.contact_count = 1;
            let mut norm = v2 - cen;
            norm = orient * norm;
            manifold.normal = norm.normalize();
            v2 = orient * v2 + b.tx.pos.coords;
            manifold.contacts[0] = v2;
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