use nalgebra::Vector2;

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

pub fn circle_polygon(manifold: &Manifold, circle_first: Option<bool>) {

    let (a, b) =
    if circle_first.unwrap_or(true) { (manifold.a.borrow(), manifold.b.borrow()) }
    else { (manifold.b.borrow(), manifold.a.borrow()) };

    match (&a.shape, &b.shape) {
        (Shape::Circle { radius: rad }, Shape::Polygon { vertices: verts }) => {
            unimplemented!();
        },
        (Shape::Polygon { vertices: verts }, Shape::Circle { radius: rad }) => {
            circle_polygon(manifold, Some(false))
        },
        _ => unreachable!()
    }
}

pub fn polygon_polygon(manifold: &Manifold) {
    let a = manifold.a.borrow();
    let b = manifold.b.borrow();

    if let (Shape::Polygon { vertices: verts_a }, Shape::Polygon { vertices: verts_b }) = (&a.shape, &b.shape) {
        unimplemented!();
    }
}