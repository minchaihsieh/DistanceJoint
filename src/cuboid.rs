use bevy::prelude::*;
use crate::components::*;
pub struct CuboidPlugin;
use crate::entity::*;
use crate::DistanceJoint::*;
use crate::resource::Gravity;

impl Plugin for CuboidPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_cuboid);
    }
}

fn spawn_cuboid(mut commands: Commands,
                mut meshes: ResMut<Assets<Mesh>>,
                mut materials: ResMut<Assets<StandardMaterial>>,
                ) {
    let dynamic_cub = commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::default()),
            material: materials.add(Color::SILVER),
            transform: Transform::from_xyz(-2.0, -0.5, 0.0),
            ..default()
        },
        )
    ).insert(ParticleBundle::new_with_ang_vel_and_vel_and_pos(
        Vec3::new(-2.0, -0.5, 0.0),
        Vec3::new(0., 0., 0.),
        Vec3::new(0.0 ,0.0, 0.0),
        2.0,
        RigidBody::Dynamic),).id();

    let static_cub = commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::default()),
            material: materials.add(Color::SILVER),
            transform: Transform::from_xyz(0.0, 0.0, 0.0),
            ..default()
        },
    )
    ).insert(ParticleBundle::new_with_pos_and_vel(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0., 0., 0.),
        1.0,
        RigidBody::Static)).id();

    commands.spawn(
        DistanceJoint::new(static_cub, dynamic_cub)
            .with_local_anchor_2(0.5 * Vec3::ONE)
            .with_rest_length(1.5)
            .with_compliance(1.0 / 400.0),
    );

}
