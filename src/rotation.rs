//! Rotation components.

use std::ops::{Add, AddAssign, Sub, SubAssign};

use bevy::prelude::*;


#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct Rotation(pub Quat);

impl Rotation {
    /// Rotates the rotation by a 3D vector.
    pub fn rotate_vec3(&self, vec: Vec3) -> Vec3 {
        self.0 * vec
    }
}


impl Rotation {
    /// Rotates the rotation by a given vector,
    pub fn rotate(&self, vec: Vec3) -> Vec3 {
        self.0 * vec
    }
    pub fn rotate_quat(&mut self, rotation: Quat) {

        self.0 = self.0 * rotation;
        // print!("rot {}, delta{}", self.0, rotation);
    }
    /// Inverts the rotation.
    pub fn inverse(&self) -> Self {
        Self(self.0.inverse())
    }

    pub fn rotate_axis(&mut self, axis: Vec3, angle: f32) {
        self.rotate_quat(Quat::from_axis_angle(axis, angle));
    }
}

impl Add<Self> for Rotation {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Rotation(self.0 + rhs.0)
    }
}

impl AddAssign<Self> for Rotation {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}


impl Sub<Self> for Rotation {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Rotation(self.0 - rhs.0)
    }
}

impl SubAssign<Self> for Rotation {
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}

impl core::ops::Mul<Direction3d> for Rotation {
    type Output = Direction3d;

    fn mul(self, direction: Direction3d) -> Self::Output {
        Direction3d::new_unchecked(self.rotate(*direction))
    }
}

impl core::ops::Mul<Vec3> for Rotation {
    type Output = Vec3;

    fn mul(self, vector: Vec3) -> Self::Output {
        self.rotate(vector)
    }
}

impl core::ops::Mul<Direction3d> for &Rotation {
    type Output = Direction3d;

    fn mul(self, direction: Direction3d) -> Self::Output {
        Direction3d::new_unchecked(self.rotate(*direction))
    }
}

impl core::ops::Mul<Vec3> for &Rotation {
    type Output = Vec3;
    fn mul(self, vector: Vec3) -> Self::Output {
        self.rotate(vector)
    }
}



impl From<Rotation> for Quat {
    fn from(rot: Rotation) -> Self {
        rot.0
    }
}

impl From<Transform> for Rotation {
    fn from(value: Transform) -> Self {
        Self::from(value.rotation)
    }
}

impl From<GlobalTransform> for Rotation {
    fn from(value: GlobalTransform) -> Self {
        Self::from(value.compute_transform().rotation)
    }
}

impl From<&GlobalTransform> for Rotation {
    fn from(value: &GlobalTransform) -> Self {
        Self::from(value.compute_transform().rotation)
    }
}



impl From<Quat> for Rotation {
    fn from(quat: Quat) -> Self {
        Self(Quat::from_xyzw(
            quat.x as f32,
            quat.y as f32,
            quat.z as f32,
            quat.w as f32,
        ))
    }
}

/// The previous rotation of a body. See [`Rotation`].
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct PreviousRotation(pub Rotation);
