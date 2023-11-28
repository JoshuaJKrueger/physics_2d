use nalgebra::Vector2;

use crate::manifold::Manifold;
use crate::object::{Object, Shape};

fn circle_circle(manifold: &mut Manifold, a: &Object, b: &Object) {
    match (&a.shape, &b.shape) {
        (Shape::Circle { radius: rad_a }, Shape::Circle { radius: rad_b }) => {
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
        },
        _ => unreachable!()
    }
}

fn circle_polygon(manifold: &Manifold, a: &Object, b: &Object) {
    match (&a.shape, &b.shape) {
        (Shape::Circle { radius: rad }, Shape::Polygon { vertices: verts }) => {
            unimplemented!();
        },
        (Shape::Polygon { vertices: verts }, Shape::Circle { radius: rad }) => {
            circle_polygon(manifold, b, a)
        },
        _ => unreachable!()
    }
}

fn polygon_polygon(manifold: &Manifold, a: &Object, b: &Object) {
    match (&a.shape, &b.shape) {
        (Shape::Polygon { vertices: verts_a }, Shape::Polygon { vertices: verts_b }) => {
            unimplemented!();
        },
        _ => unreachable!()
    }
}