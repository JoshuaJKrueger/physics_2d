use crate::types::{InvKilogram, Kilogram, KilogramMeterSquared, MeterSquaredPerKilogram};

/// Represents the mass and moment of inertia data of an object.
/// Stores their inverses as they are used frequently in computations.
pub struct MassData {
    /// Mass of the object in kilograms.
    pub mass: Kilogram,
    /// Inverse mass of the object, calculated as 1 divided by the mass.
    pub inv_mass: InvKilogram,
    // moment_inertia: KilogramMeterSquared,
    /// Inverse moment of inertia of the object, calculated as 1 divided by the moment of inertia.
    pub inv_m_inertia: MeterSquaredPerKilogram,
}

impl MassData {
    /// Creates a new `MassData` instance with the given mass and moment of inertia.
    ///
    /// # Arguments
    ///
    /// * `mass` - The mass of the object in kilograms.
    /// * `moment_inertia` - The moment of inertia of the object in kilogram meter squared.
    ///
    /// # Returns
    ///
    /// A new `MassData` instance with the calculated inverse mass and inverse moment of inertia.
    pub fn new(mass: Kilogram, moment_inertia: KilogramMeterSquared) -> Self {
        let inv_mass = if mass != 0.0 { 1.0 / mass } else { 0.0 };
        let inv_m_inertia = if moment_inertia != 0.0 {
            1.0 / moment_inertia
        } else {
            0.0
        };

        MassData {
            mass,
            inv_mass,
            // moment_inertia,
            inv_m_inertia,
        }
    }
}
