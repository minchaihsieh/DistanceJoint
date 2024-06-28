use bevy::prelude::*;
use crate::components::*;
pub struct CuboidPlugin;
use crate::entity::*;

impl Plugin for CuboidPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_cuboid);
    }
}

fn spawn_cuboid(mut commands: Commands,
                mut meshes: ResMut<Assets<Mesh>>,
                mut materials: ResMut<Assets<StandardMaterial>>) {
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::default()),
            material: materials.add(Color::SILVER),
            transform: Transform::from_xyz(0.0, 0.5, 0.5),
            ..default()
        },
        RigidBody::Dynamic,
        )
    ).insert(ParticleBundle::new_with_pos_and_vel(
        Vec3::new(0.0, 0.5, 0.5),
        Vec3::new(0., 0., 0.),));
}
