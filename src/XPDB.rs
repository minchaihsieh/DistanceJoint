use bevy::prelude::*;
use crate::components::*;
use crate::resource::*;
pub struct XPDBPlugin;
pub const DELTA_TIME: f32 = 1. / 60.;
impl Plugin for XPDBPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, integrate_pos);
        app.add_systems(Update, position_to_transform);

    }
}


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


// 计算仅受外力作用的物体位置与线速度


fn integrate_pos(mut query: Query<(&mut Position, &mut PreviousPosition, &Mass)>) {
    for (mut pos, mut prev_pos, mass) in query.iter_mut() {
        let gravity = Vec3::new(0., -9.81, 0.);
        let gravitation_force = mass.0 * gravity;
        let external_forces = gravitation_force;
        let velocity = (pos.0 - prev_pos.0) / DELTA_TIME + DELTA_TIME * external_forces / mass.0;
        prev_pos.0 = pos.0;
        pos.0 = pos.0 + velocity * DELTA_TIME;
    }
}


type PosToTransformComponents = (
    &'static mut Transform,
    &'static Position,
    //&'static Rotation,
    Option<&'static Parent>,
);

type PosToTransformFilter = (With<RigidBody>, Changed<Position>);

pub fn position_to_transform(mut query: Query<(&mut Transform, &Position)>) {
    for (mut transform, pos) in query.iter_mut() {
        transform.translation = pos.0;
    }
}



