
use bevy::prelude::*;
use bevy::reflect::erased_serde::__private::serde;
use derive_more::From;


type PosIntegrationComponents = (
    &'static RigidBody,
    &'static Position,
    &'static mut PreviousPosition,
    &'static mut AccumulatedTranslation,
    &'static mut LinearVelocity,
    //Option<&'static LinearDamping>,
    //Option<&'static GravityScale>,
    //&'static ExternalForce,
    &'static Mass,
    &'static InverseMass,
    //Option<&'static LockedAxes>,
);


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

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub(crate) struct PreSolveAngularVelocity(pub Vec3);

impl AngularVelocity {
    /// Zero angular velocity.
    pub const ZERO: AngularVelocity = AngularVelocity(Vec3::ZERO);
}

#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct Mass(pub f32);
impl Default for Mass {
    fn default() -> Self {
        Self(1.) // Default to 1 kg
    }
}

// 用来模拟不动的物体
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
pub struct InverseMass(pub f32);

impl InverseMass {
    pub const ZERO: Self = Self(0.0);
}

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
