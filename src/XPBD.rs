use bevy::prelude::*;
use bevy::utils::petgraph::matrix_graph::Zero;
use crate::components::*;
use crate::resource::*;
use crate::DistanceJoint::*;
use crate::rotation::{PreviousRotation, Rotation};
use crate::Step;
use crate::query::*;
use crate::utils::*;

pub struct XPBDPlugin;
impl Plugin for XPBDPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Gravity>();
        app.add_systems(Update, collect_collision_pair.in_set(Step::CollectCollisionPairs).before(Step::Integrate));
        app.add_systems(Update, integrate_pos.in_set(Step::Integrate));
        app.add_systems(Update, integrate_rot.in_set(Step::Integrate));
        app.add_systems(Update, solve_pos.in_set(Step::SolvePositions).after(Step::Integrate));
        app.add_systems(Update, update_lin_vel.in_set(Step::UpdateVelocities).after(Step::SolvePositions));
        app.add_systems(Update, update_ang_vel.in_set(Step::UpdateVelocities).after(Step::SolvePositions));
        app.add_systems(Update, solve_lin_vel.in_set(Step::SolveVelocities).after(Step::UpdateVelocities));
        app.add_systems(Update, apply_translation.in_set(Step::ApplyTranslation));
        app.add_systems(Update, position_to_transform.after(Step::UpdateVelocities));
    }
}


pub fn compute_generalized_inverse_mass(
    body: &RigidBodyQueryItem,
    r: Vec3,
    n: Vec3,
) -> f32 {
    if body.rb.is_dynamic() {
        let inverse_inertia = body.effective_world_inv_inertia();

        let r_cross_n = r.cross(n); // Compute the cross product only once

        // The line below is equivalent to Eq (2) because the component-wise multiplication of a transposed vector and another vector is equal to the dot product of the two vectors.
        // a^T * b = a • b
        body.inverse_mass.0 + r_cross_n.dot(inverse_inertia * r_cross_n)
    } else {
        // Static and kinematic bodies are a special case, where 0.0 can be thought of as infinite mass.
        0.0
    }
}


// 计算仅受外力作用的物体位置与线速度
fn collect_collision_pair() {}

fn integrate_pos(mut query: Query<(&mut Position, &mut PreviousPosition, &mut LinearVelocity, &Mass, &RigidBody, &mut AccumulatedTranslation, &InverseMass)>, gravity: Res<Gravity>, time: Res<Time>) {
    let delta_seconds = time.delta_seconds();

    for (pos, mut prev_pos, mut lin_vel, mass, rigid_body, mut translation, inv_mass) in query.iter_mut() {
        if rigid_body.is_static() {
            continue;
        }

        prev_pos.0 = pos.0;
        let effective_mass = Vec3::splat(mass.0);
        let effective_inv_mass = Vec3::splat(inv_mass.0);
        let gravitation_force = effective_mass * gravity.0;
        let external_forces = gravitation_force;
        lin_vel.0 += delta_seconds * external_forces * effective_inv_mass;
        translation.0 += lin_vel.0 * delta_seconds;

        // println!("prev pos {}, lin_vel {}", prev_pos.0, lin_vel.0);
    }
}


fn integrate_rot(mut query: Query<(&mut Rotation, &mut PreviousRotation, &mut AngularVelocity, &RigidBody, &Inertia, &InverseInertia)>, time: Res<Time>) {
    let delta_seconds = time.delta_seconds();

    for (mut rot, mut prev_rot, mut ang_vel, rigid_body, inertia, inv_inertia) in query.iter_mut() {
        // prev_rot.0 = *rot;
        // if ang_vel.0 == Vec3::ZERO {
        //     continue;
        // }
        // rot.rotate_axis(ang_vel.0.normalize(), ang_vel.length() * delta_seconds);
        prev_rot.0 = *rot;

        if rigid_body.is_static() {
            continue;
        }


        // Apply damping and external torque
        if rigid_body.is_dynamic() {
            // Apply damping
            // if let Some(damping) = ang_damping {
            //     // avoid triggering bevy's change detection unnecessarily
            //     if ang_vel.0 != Vector::ZERO && damping.0 != 0.0 {
            //         ang_vel.0 *= 1.0 / (1.0 + delta_secs * damping.0);
            //     }
            // }

            let effective_inertia = inertia.rotated(&rot).0;
            let effective_inv_inertia = inv_inertia.rotated(&rot).0;

            // Apply external torque
            let delta_ang_vel = delta_seconds
                * effective_inv_inertia
                * (Vec3::ZERO
                - ang_vel.0.cross(effective_inertia * ang_vel.0));
            // avoid triggering bevy's change detection unnecessarily
            if delta_ang_vel != Vec3::ZERO {
                ang_vel.0 += delta_ang_vel;
            }
        }

        let q = Quat::from_vec4(ang_vel.0.extend(0.0)) * rot.0;
        let effective_dq = delta_seconds * 0.5 * q.xyz()
            .extend(delta_seconds * 0.5 * q.w);
        // avoid triggering bevy's change detection unnecessarily
        let delta = Quat::from_vec4(effective_dq);
        if delta != Quat::from_xyzw(0.0, 0.0, 0.0, 0.0) {
            rot.0 = (rot.0 + delta).normalize();
        }
    }
}


fn solve_pos(mut query: Query<&mut DistanceJoint>,
             mut bodies: Query<RigidBodyQuery, With<Mass>>,
             time: Res<Time>) {
    let delta_seconds = time.delta_seconds();
    if !delta_seconds.is_zero() {
        for mut joint in query.iter_mut() {
            joint.clear_lagrange_multipliers();

            if let Ok(mut bodies) = bodies.get_many_mut(joint.entities()) {
                if let Ok(mut bodies) = bodies
                    .iter_mut()
                    .collect::<Vec<&mut RigidBodyQueryItem>>()
                    .try_into() {
                    let [body1, body2] = bodies;
                    let world_r1 = body1.rotation.rotate(joint.local_anchor1);
                    let world_r2 = body2.rotation.rotate(joint.local_anchor2);
                    let p1 = body1.current_position() + world_r1;
                    let p2 = body2.current_position() + world_r2;

                    let pos_offset = p2 - p1;
                    let dis = pos_offset.length();

                    let (dir, distance) = (-pos_offset / dis, dis - joint.rest_length);
                    if distance.abs() < f32::EPSILON {
                        continue;
                    }

                    let w1 = compute_generalized_inverse_mass(body1, world_r1, dir);
                    let w2 = compute_generalized_inverse_mass(body2, world_r2, dir);
                    let w = [w1, w2];

                    let gradients = [dir, -dir];
                    println!("w {} gradient {} distance{} dt {}", w2, dir, distance, delta_seconds);
                    // Compute Lagrange multiplier update, essentially the signed magnitude of the correction
                    let delta_lagrange = joint.compute_lagrange_update(
                        joint.lagrange,
                        distance,
                        &gradients,
                        &w,
                        joint.compliance,
                        delta_seconds,
                    );
                    joint.lagrange += delta_lagrange;

                    // Apply positional correction (method from PositionConstraint)
                    joint.apply_positional_correction(body1, body2, delta_lagrange, dir, world_r1, world_r2);

                    // Return constraint force
                    joint.force = joint.compute_force(joint.lagrange, dir, delta_seconds);
                }
            }

            // 不考虑旋转的质点
            // let mass1 = body1.3.0;
            // let mass2 = body2.3.0;
            // let direction = body1.0.0  - body2.0.0;
            // let distance = direction.length();
            // let err = distance - joint.rest_length;
            // if body1.4.is_static() {
            //     body2.0.0 += err * (direction / distance);
            // } else if body2.4.is_static() {
            //     body1.0.0 -= err * direction / distance;
            // } else {
            //     body1.0.0 -= err * direction / distance * (mass1 / mass1 + mass2);
            //     body2.0.0 += err * direction / distance * (mass2 / mass1 + mass2);
            // }
        }
    }
}
fn update_ang_vel(mut query: Query<(&RigidBody, &Rotation, &PreviousRotation, &mut AngularVelocity, &mut PreSolveAngularVelocity,)>, time: Res<Time>) {
    let delta_seconds = time.delta_seconds();

    for (rb, rot, prev_rot, mut ang_vel, mut pre_solve_ang_vel) in &mut query {
        // Static bodies have no velocity
        if rb.is_static() && ang_vel.0 != Vec3::ZERO {
            ang_vel.0 = Vec3::ZERO;
        }

        pre_solve_ang_vel.0 = ang_vel.0;

        if rb.is_dynamic() {
            let delta_rot = rot.mul_quat(prev_rot.inverse().0);
            let mut new_ang_vel = 2.0 * delta_rot.xyz() / delta_seconds;
            if delta_rot.w < 0.0 {
                new_ang_vel = -new_ang_vel;
            }
            // avoid triggering bevy's change detection unnecessarily
            if new_ang_vel != ang_vel.0 && new_ang_vel.is_finite() {
                ang_vel.0 = new_ang_vel;
            }
        }
    }
}
fn update_lin_vel(mut query: Query<(&Position, &PreviousPosition, &AccumulatedTranslation, &mut LinearVelocity, &RigidBody)>, time: Res<Time>) {
    let delta_seconds = time.delta_seconds();
    for (pos, prev_pos, translation, mut lin_vel, rigid_body) in query.iter_mut() {
        if rigid_body.is_static() {
            continue;
        }
        //println!("old lin {} pos {}, translation{}", lin_vel.0, pos.0, translation.0);
        // v = (x - x_prev) / h
        let new_lin_vel = (pos.0 - prev_pos.0 + translation.0) / delta_seconds;
        // avoid triggering bevy's change detection unnecessarily
        if new_lin_vel != lin_vel.0 && new_lin_vel.is_finite() {
            lin_vel.0 = new_lin_vel;
        }
        //println!("new lin {}", lin_vel.0);
    }
}
fn solve_lin_vel() {
    // TODO: SolveVelocity
}


fn apply_translation(
    mut bodies: Query<
        (
            &RigidBody,
            &mut Position,
            &Rotation,
            &PreviousRotation,
            &mut AccumulatedTranslation,
            &CenterOfMass,
        ),
        Changed<AccumulatedTranslation>,
    >,
) {
    for (rb, mut pos, rot, prev_rot, mut translation, center_of_mass) in &mut bodies {
        if rb.is_static() {
            continue;
        }

        // We must also account for the translation caused by rotations around the center of mass,
        // as it may be offset from `Position`.
        pos.0 += get_pos_translation(&translation, prev_rot, rot, center_of_mass);
        translation.0 = Vec3::ZERO;
    }
}


pub fn position_to_transform(mut query: Query<(&mut Transform, &Position, &Rotation)>) {
    for (mut transform, pos, rot) in query.iter_mut() {
        transform.translation = pos.0;
        transform.rotation = rot.0;
    }
}



