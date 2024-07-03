use bevy::prelude::*;
use crate::components::*;
pub struct CuboidPlugin;
use crate::entity::*;
use crate::DistanceJoint::*;

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
            material: materials.add(Color::rgb(0.8, 0.7, 0.6)),
            transform: Transform::from_xyz(-2.0, -0.5, 0.0),
            ..default()
        },
        )
    ).insert(ParticleBundle::new_with_ang_vel_and_vel_and_pos(
        // 位置
        Vec3::new(-2.0, -0.5, 0.0),
        // 初始线速度
        Vec3::new(0., 0., 0.),
        // 初始角速度
        Vec3::new(0.0 ,0.0, 0.0),
        // 质量
        2.0,
        // 刚体状态 静态/动态
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
        // 位置
        Vec3::new(0.0, 0.0, 0.0),
        // 初始线速度
        Vec3::new(0., 0., 0.),
        // 质量
        1.0,
        // 刚体状态 静态/动态
        RigidBody::Static)).id();

    commands.spawn(
        DistanceJoint::new(static_cub, dynamic_cub)
            // 连接点位置
            .with_local_anchor_2(0.5 * Vec3::ONE)
            // 长度
            .with_rest_length(1.5)
            // 柔度
            .with_compliance(1.0 / 400.0),
    );
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 2_000_000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });

}
