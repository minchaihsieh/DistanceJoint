
use bevy::ecs::reflect::ReflectResource;
use bevy::math::Vec3;
use bevy::prelude::{Reflect, Resource};
use bevy::reflect::erased_serde::__private::serde;


#[derive(Reflect, Resource, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Resource)]
pub struct Gravity(pub Vec3);

impl Default for Gravity {
    fn default() -> Self {
        Self(Vec3::Y * -9.81)
    }
}

impl Gravity {
    pub const ZERO: Gravity = Gravity(Vec3::ZERO);
}