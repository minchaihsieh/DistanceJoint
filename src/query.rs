use bevy::ecs::query::QueryData;
use bevy::math::{Mat3, Vec3};
use bevy::prelude::{Entity, Ref};
use crate::components::*;
use crate::rotation::*;
use crate::utils::*;

#[derive(QueryData)]
#[query_data(mutable)]
pub struct RigidBodyQuery {
    pub entity: Entity,
    pub rb: Ref<'static, RigidBody>,
    pub position: &'static mut Position ,
    pub rotation: &'static mut Rotation,
    pub previous_position: &'static mut PreviousPosition,
    pub previous_rotation: &'static mut PreviousRotation,
    pub accumulated_translation: &'static mut AccumulatedTranslation,
    pub linear_velocity: &'static mut LinearVelocity,
    //pub(crate) pre_solve_linear_velocity: &'static mut PreSolveLinearVelocity,
    pub angular_velocity: &'static mut AngularVelocity,
   // pub(crate) pre_solve_angular_velocity: &'static mut PreSolveAngularVelocity,
    pub mass: &'static mut Mass,
    pub inverse_mass: &'static mut InverseMass,
    pub inertia: &'static mut Inertia,
    pub inverse_inertia: &'static mut InverseInertia,
    pub center_of_mass: &'static mut CenterOfMass,
    //pub friction: &'static Friction,
    //pub restitution: &'static Restitution,
    //pub locked_axes: Option<&'static LockedAxes>,
    pub dominance: Option<&'static Dominance>,
}

impl<'w> RigidBodyQueryItem<'w> {
    /// Computes the effective inverse mass, taking into account any translation locking.
    pub fn effective_inv_mass(&self) -> Vec3 {
        let mut inv_mass = Vec3::splat(self.inverse_mass.0);

        inv_mass
    }

    /// Computes the effective world-space inverse inertia tensor, taking into account any rotation locking.
    pub fn effective_world_inv_inertia(&self) -> Mat3 {
        let mut inv_inertia = self.inverse_inertia.rotated(&self.rotation).0;

        inv_inertia
    }

    /// Returns the current position of the body. This is a sum of the [`Position`] and
    /// [`AccumulatedTranslation`] components.
    pub fn current_position(&self) -> Vec3 {
        self.position.0
            + get_pos_translation(
            &self.accumulated_translation,
            &self.previous_rotation,
            &self.rotation,
            &self.center_of_mass,
        )
    }

    /// Returns the [dominance](Dominance) of the body.
    ///
    /// If it isn't specified, the default of `0` is returned for dynamic bodies.
    /// For static and kinematic bodies, `i8::MAX` (`127`) is always returned instead.
    pub fn dominance(&self) -> i8 {
        if !self.rb.is_dynamic() {
            i8::MAX
        } else {
            self.dominance.map_or(0, |dominance| dominance.0)
        }
    }
}