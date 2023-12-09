use crate::types::{KilogramPerCubicMeter, NormalizedCoefficient};

/// Represents material properties of an object affecting its physical behavior.
pub struct Material {
    /// Density of the material in kilograms per cubic meter.
    pub density: KilogramPerCubicMeter,
    /// Coefficient of restitution, representing the elasticity of collisions.
    pub restitution: NormalizedCoefficient,
    /// Coefficient of dynamic friction, determining resistance to motion during contact.
    pub dynamic_friction: NormalizedCoefficient,
    /// Coefficient of static friction, determining resistance to motion before motion begins.
    pub static_friction: NormalizedCoefficient,
}

impl Material {
    /// Creates a new `Material` instance with the specified material properties.
    ///
    /// # Arguments
    ///
    /// * `density` - The density of the material in kilograms per cubic meter.
    /// * `restitution` - The coefficient of restitution representing the elasticity of collisions.
    /// * `dynamic_friction` - The coefficient of dynamic friction determining resistance to motion during contact.
    /// * `static_friction` - The coefficient of static friction determining resistance to motion before motion begins.
    ///
    /// # Returns
    ///
    /// A new `Material` instance with the specified material properties.
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
