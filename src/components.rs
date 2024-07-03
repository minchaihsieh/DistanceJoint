
use bevy::prelude::*;
use bevy::reflect::erased_serde::__private::serde;
use derive_more::From;
use crate::rotation::*;
use crate::utils::*;



#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub enum RigidBody {
    #[default]
    Dynamic,
    Static,
}

impl RigidBody {
    pub fn is_dynamic(&self) -> bool { *self == Self::Dynamic}
    pub fn is_static(&self) -> bool { *self == Self::Static}
}

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct Position(pub Vec3);

impl Position {
    pub fn new(position: Vec3) -> Self { Self(position) }

    pub fn from_xyz(x: f32, y: f32, z: f32) ->Self { Self(Vec3::new(x, y, z))}
}


#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
pub struct PreviousPosition(pub Vec3);


#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
pub struct AccumulatedTranslation(pub Vec3);


#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct LinearVelocity(pub Vec3);

impl LinearVelocity {
    pub const ZERO: LinearVelocity = LinearVelocity(Vec3::ZERO);
}

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct AngularVelocity(pub Vec3);

impl AngularVelocity {
    /// Zero angular velocity.
    pub const ZERO: AngularVelocity = AngularVelocity(Vec3::ZERO);
}

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub(crate) struct PreSolveAngularVelocity(pub Vec3);


#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct Mass(pub f32);
impl Default for Mass {
    fn default() -> Self {
        Self(1.) // Default to 1 kg
    }
}

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
pub struct InverseMass(pub f32);

impl InverseMass {
    pub const ZERO: Self = Self(0.0);

}

#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct Inertia(pub Mat3);
impl Default for Inertia {
    fn default() -> Self {
        Self(Mat3::ZERO)
    }
}

impl Inertia {
    /// Zero angular inertia.
    pub const ZERO: Self = Self(Mat3::ZERO);


    pub fn rotated(&self, rot: &Rotation) -> Self {
        Self(get_rotated_inertia_tensor(self.0, rot.0))
    }


    /// Returns the inverted moment of inertia.

    pub fn inverse(&self) -> InverseInertia {
        InverseInertia(self.0.inverse())
    }


    /// Computes the inertia of a body with the given mass, shifted by the given offset.

    pub fn shifted(&self, mass: f32, offset: Vec3) -> Mat3 {
        if mass > 0.0 && mass.is_finite() {
            let diag = offset.length_squared();
            let diagm = Mat3::from_diagonal(Vec3::splat(diag));
            let offset_outer_product =
                Mat3::from_cols(offset * offset.x, offset * offset.y, offset * offset.z);
            self.0 + (diagm + offset_outer_product) * mass
        } else {
            self.0
        }
    }
}

#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct InverseInertia(pub Mat3);

impl Default for InverseInertia {
    fn default() -> Self {
        InverseInertia(Mat3::ZERO)
    }
}


impl InverseInertia {
    /// Zero inverse angular inertia.

    pub const ZERO: Self = Self(Mat3::ZERO);


    /// Returns the inertia tensor's world-space version that takes the body's orientation into account.

    pub fn rotated(&self, rot: &Rotation) -> Self {
        Self(get_rotated_inertia_tensor(self.0, rot.0))
    }

    /// Returns the original moment of inertia.

    pub fn inverse(&self) -> Inertia {
        Inertia(self.0.inverse())
    }
}

impl From<Inertia> for InverseInertia {
    fn from(inertia: Inertia) -> Self {
        inertia.inverse()
    }
}



#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct CenterOfMass(pub Vec3);

impl CenterOfMass {
    /// A center of mass set at the local origin.
    pub const ZERO: Self = Self(Vec3::ZERO);

    pub fn new(center_of_mass: Vec3) -> Self { Self(center_of_mass) }
}

#[rustfmt::skip]
#[derive(Component, Reflect, Debug, Clone, Copy, Default, Deref, DerefMut, From, PartialEq, PartialOrd, Eq, Ord)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct Dominance(pub i8);

pub(crate) trait TimePrecisionAdjusted {
    /// Returns how much time has advanced since the last update
    /// as [`Scalar`] seconds.
    fn delta_seconds_adjusted(&self) -> f32;
}

impl TimePrecisionAdjusted for Time {
    /// Returns how much time has advanced since the last [`update`](#method.update)
    /// as [`Scalar`] seconds.
    fn delta_seconds_adjusted(&self) -> f32 {
        self.delta_seconds()
    }
}
