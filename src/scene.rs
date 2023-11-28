use crate::manifold::Manifold;
use crate::object::Object;

pub struct Scene<'a> {
    pub objects: Vec<Object>,
    pub contacts: Vec<Manifold<'a>>,
}

impl<'a> Scene<'a> {
    fn step() {
        unimplemented!()
    }

    fn render() {
        unimplemented!()
    }

    fn clear() {
        unimplemented!()
    }
}