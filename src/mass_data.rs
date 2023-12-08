use crate::types::{InvKilogram, Kilogram, KilogramMeterSquared, MeterSquaredPerKilogram};

pub struct MassData {
    pub mass: Kilogram,
    pub inv_mass: InvKilogram,
    // moment_inertia: KilogramMeterSquared,
    pub inv_m_inertia: MeterSquaredPerKilogram,
}

impl MassData {
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
