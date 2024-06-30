use bevy::prelude::*;
use crate::components::*;
use crate::resource::*;
use crate::DistanceJoint::*;
use crate::rotation::{PreviousRotation, Rotation};
use crate::Step;

pub struct XPDBPlugin;
pub const DELTA_TIME: f32 = 1. / 60.;
impl Plugin for XPDBPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Gravity>();
        app.add_systems(Update, collect_collision_pair.in_set(Step::CollectCollisionPairs).before(Step::Integrate));
        app.add_systems(Update, integrate_pos.in_set(Step::Integrate));
        app.add_systems(Update, integrate_rot.in_set(Step::Integrate));
        app.add_systems(Update, solve_pos.in_set(Step::SolvePositions).after(Step::Integrate));
        app.add_systems(Update, update_lin_vel.in_set(Step::UpdateVelocities).after(Step::SolvePositions));
        app.add_systems(Update, solve_lin_vel.in_set(Step::SolveVelocities).after(Step::UpdateVelocities));
        app.add_systems(Update, position_to_transform.after(Step::SolveVelocities));
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
fn collect_collision_pair() {}

fn integrate_pos(mut query: Query<(&mut Position, &mut PreviousPosition, &mut LinearVelocity, &Mass, &RigidBody)>, gravity: Res<Gravity>, time: Res<Time>) {
    let delta_seconds = time.delta_seconds_adjusted();

    for (mut pos, mut prev_pos, mut lin_vel, mass, rigid_body) in query.iter_mut() {
        if rigid_body.is_static() {
            continue;
        }
        prev_pos.0 = pos.0;
        let gravitation_force = mass.0 * gravity.0;
        let external_forces = gravitation_force;
        lin_vel.0 += DELTA_TIME * external_forces / mass.0;
        pos.0 += lin_vel.0 * DELTA_TIME;
    }
}

fn integrate_rot(mut query: Query<(&mut Rotation, &mut PreviousRotation, &mut AngularVelocity)>) {
    for (mut rot, mut prev_rot, mut ang_vel) in query.iter_mut() {
        prev_rot.0 = *rot;
        let q = Quat::from_vec4(ang_vel.0.extend(0.0)) * rot.0;
        let effective_dq = DELTA_TIME * 0.5 * q.xyz()
            .extend(DELTA_TIME * 0.5 * q.w);
        // avoid triggering bevy's change detection unnecessarily
        let delta = Quat::from_vec4(effective_dq);
        if delta != Quat::from_xyzw(0.0, 0.0, 0.0, 0.0) {
            rot.0 = (rot.0 + delta).normalize();
        }
        println!("rot {}, delta{}", rot.0, delta);
    }
}


fn solve_pos(mut query: Query<(&mut DistanceJoint)>,
mut bodies: Query<(&mut Position, &mut PreviousPosition, &mut LinearVelocity, &Mass, &RigidBody)>) {
    for(mut joint) in query.iter_mut(){
        if let Ok([mut body1, mut body2]) = bodies.get_many_mut(joint.entities()){
            // 使用质心位置会有问题！
            let mass1 = body1.3.0;
            let mass2 = body2.3.0;
            let direction = body1.0.0 - body2.0.0;
            let distance =  direction.length();
            let err = distance - joint.rest_length;
            if body1.4.is_static() {
                body2.0.0 += err * (direction / distance);
            } else if body2.4.is_static() {
                body1.0.0 -= err * direction / distance;
            } else {
                body1.0.0 -= err * direction / distance * (mass1 / mass1 + mass2);
                body2.0.0 += err * direction / distance * (mass2 / mass1 + mass2);
            }
        }

    }
}
// TODO: SolvePositions
fn update_lin_vel(mut query: Query<(&Position, &PreviousPosition, &mut LinearVelocity, &RigidBody)>, time: Res<Time>) {
    let delta_seconds = time.delta_seconds_adjusted();
    for (pos, prev_pos, mut lin_vel, rigid_body) in query.iter_mut() {
        if rigid_body.is_static() {
            continue;
        }
        lin_vel.0 = (pos.0 - prev_pos.0) / DELTA_TIME;

    }
}
fn solve_lin_vel() {
    // TODO: SolveVelocity
}

type PosToTransformComponents = (
    &'static mut Transform,
    &'static Position,
    //&'static Rotation,
    Option<&'static Parent>,
);

type PosToTransformFilter = (With<RigidBody>, Changed<Position>);

pub fn position_to_transform(mut query: Query<(&mut Transform, &Position, &Rotation)>) {
    for (mut transform, pos, rot) in query.iter_mut() {
        transform.translation = pos.0;
        transform.rotation = rot.0;
    }
}



