
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
    pub lin_vel: LinearVelocity,
    pub ang_vel: AngularVelocity,
    pub rigid_body: RigidBody,
}

impl ParticleBundle {
    pub fn new_with_ang_vel_and_vel_and_pos (pos: Vec3, vel: Vec3, ang_vel: Vec3, rigid_body: RigidBody) -> Self {
        //let delta_time = time.delta_seconds_adjusted();
        Self {
            ang_vel: AngularVelocity(ang_vel),
            ..Self::new_with_pos_and_vel(pos, vel, rigid_body)
        }
    }
    pub fn new_with_pos_and_vel(pos: Vec3, vel: Vec3, rigid_body: RigidBody) -> Self {
        //let delta_time = time.delta_seconds_adjusted();
        Self {
            pos: Position(pos),
            prev_pos: PreviousPosition(pos - vel * DELTA_TIME),
            lin_vel: LinearVelocity(vel),
            rigid_body,
            ..Default::default()
        }
    }
}