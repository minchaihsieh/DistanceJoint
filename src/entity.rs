
use bevy::prelude::*;
use crate::components::*;
use crate::rotation::{PreviousRotation, Rotation};

pub const DELTA_TIME: f32 = 1. / 60.;

#[derive(Bundle, Default)]
pub struct ParticleBundle {
    pub pos: Position,
    pub prev_pos: PreviousPosition,
    pub rot: Rotation,
    pub prev_rot: PreviousRotation,
    pub mass: Mass,
    pub inverse_mass: InverseMass,
    pub lin_vel: LinearVelocity,
    pub ang_vel: AngularVelocity,
    pub rigid_body: RigidBody,
    pub translation: AccumulatedTranslation,
    pub inertia: Inertia,
    pub inverse_inertia: InverseInertia,
    pub center_of_mass: CenterOfMass,

}

impl ParticleBundle {
    pub fn new_with_ang_vel_and_vel_and_pos (pos: Vec3, vel: Vec3, ang_vel: Vec3, mass: f32, rigid_body: RigidBody) -> Self {
        //let delta_time = time.delta_seconds_adjusted();
        Self {
            ang_vel: AngularVelocity(ang_vel),
            ..Self::new_with_pos_and_vel(pos, vel, mass, rigid_body)
        }
    }
    pub fn new_with_pos_and_vel(pos: Vec3, vel: Vec3, mass: f32, rigid_body: RigidBody) -> Self {
        //let delta_time = time.delta_seconds_adjusted();
        Self {
            pos: Position(pos),
            prev_pos: PreviousPosition(pos - vel * DELTA_TIME),
            lin_vel: LinearVelocity(vel),
            mass: Mass(mass),
            inverse_mass:  InverseMass(if rigid_body == RigidBody::Dynamic {1. / mass} else { 0.0}) ,
            inertia: Inertia(Mat3::from_cols(Vec3::new(1.0 / 3.0, 0.0, 0.0), Vec3::new(0.0,1.0 / 3.0, 0.0), Vec3::new(0.0,0.0,1.0 / 3.0))),
            inverse_inertia: InverseInertia(Mat3::from_cols(Vec3::new(3.0, 0.0, 0.0), Vec3::new(0.0 , 3.0, 0.0), Vec3::new(0.0, 0.0, 3.0))),
            rigid_body,
            center_of_mass: CenterOfMass(pos),
            ..Default::default()
        }
    }
}