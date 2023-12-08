use crate::types::{KilogramPerCubicMeter, NormalizedCoefficient};

pub struct Material {
    pub density: KilogramPerCubicMeter,
    pub restitution: NormalizedCoefficient,
    pub dynamic_friction: NormalizedCoefficient,
    pub static_friction: NormalizedCoefficient,
}

impl Material {
    pub fn new(
        density: KilogramPerCubicMeter,
        restitution: NormalizedCoefficient,
        dynamic_friction: NormalizedCoefficient,
        static_friction: NormalizedCoefficient,
    ) -> Self {
        Material {
            density,
            restitution,
            dynamic_friction,
            static_friction,
        }
    }
}
