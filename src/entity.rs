
use bevy::prelude::*;
use crate::components::*;
pub const DELTA_TIME: f32 = 1. / 60.;

#[derive(Bundle, Default)]
pub struct ParticleBundle {
    pub pos: Position,
    pub prev_pos: PreviousPosition,
    pub mass: Mass,
}

impl ParticleBundle {
    pub fn new_with_pos_and_vel(pos: Vec3, vel: Vec3) -> Self {

        Self {
            pos: Position(pos),
            prev_pos: PreviousPosition(pos - vel * DELTA_TIME),
            ..Default::default()
        }
    }
}