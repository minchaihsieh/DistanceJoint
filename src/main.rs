
mod debug;
mod cuboid;
mod camera;
mod XPBD;
mod components;
mod resource;
mod entity;
mod DistanceJoint;
mod rotation;
mod query;
mod utils;

use bevy::prelude::*;
use crate::camera::CameraPlugin;
use crate::cuboid::CuboidPlugin;
use crate::XPBD::XPBDPlugin;
use bevy::diagnostic::LogDiagnosticsPlugin;
use bevy::diagnostic::FrameTimeDiagnosticsPlugin;

fn main() {
    App::new()

        .insert_resource(ClearColor(Color::default()))
        // .insert_resource(AmbientLight {
        //     color: Color::default(),
        //     brightness: 0.75,
        // })
        .add_plugins(DefaultPlugins)
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_plugins(LogDiagnosticsPlugin::default())

        //.add_plugins(DebugPlugin)
        .add_plugins(CameraPlugin)
        .add_plugins(CuboidPlugin)
        .add_plugins(XPBDPlugin)
        //.insert_resource(Gravity(Vec3::ZERO))
        .run();
}

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Step {
    CollectCollisionPairs,
    Integrate,
    SolvePositions,
    UpdateVelocities,
    SolveVelocities,
    ApplyTranslation,
}

// #[derive(Component)]
// struct Rotatable {
//     speed: f32,
// }
//
// fn main() {
//     App::new()
//         .add_plugins(DefaultPlugins)
//         .add_systems(Startup, setup)
//         .add_systems(Update, rotate_cube)
//         .run();
// }
//
// fn setup(
//     mut commands: Commands,
//     mut meshes: ResMut<Assets<Mesh>>,
//     mut materials: ResMut<Assets<StandardMaterial>>,
// ) {
//     // Spawn a cube to rotate.
//     commands.spawn((
//         PbrBundle {
//             mesh: meshes.add(Cuboid::default()),
//             material: materials.add(Color::WHITE),
//             transform: Transform::from_translation(Vec3::ZERO),
//             ..default()
//         },
//         Rotatable { speed: 2.0 * PI },
//     ));
//
//     // Spawn a camera looking at the entities to show what's happening in this example.
//     commands.spawn(Camera3dBundle {
//         transform: Transform::from_xyz(0.0, 10.0, 20.0).looking_at(Vec3::ZERO, Vec3::Y),
//         ..default()
//     });
//
//     // Add a light source so we can see clearly.
//     commands.spawn(DirectionalLightBundle {
//         transform: Transform::from_xyz(3.0, 3.0, 3.0).looking_at(Vec3::ZERO, Vec3::Y),
//         ..default()
//     });
// }
//
// // This system will rotate any entity in the scene with a Rotatable component around its y-axis.
// fn rotate_cube(mut cubes: Query<(&mut Transform, &Rotatable)>, timer: Res<Time>) {
//     for (mut transform, cube) in &mut cubes {
//         // The speed is first multiplied by TAU which is a full rotation (360deg) in radians,
//         // and then multiplied by delta_seconds which is the time that passed last frame.
//         // In other words. Speed is equal to the amount of rotations per second.
//         transform.rotate_axis(Vec3::new(1.0,1.0,1.0).normalize(), cube.speed  * timer.delta_seconds());
//     }
// }

//
// use bevy::prelude::*;
// use bevy_xpbd_3d::{math::*, prelude::*};
// use crate::components::RigidBody;
// fn main() {
//     App::new()
//         .add_plugins((
//             DefaultPlugins,
//             PhysicsPlugins::default(),
//             PhysicsDebugPlugin::default(),
//         ))
//         .register_type::<RigidBody>()
//         .add_plugins(CuboidPlugin)
//         .add_systems(Startup, setup)
//         .run();
// }
//
// fn setup(
//     mut commands: Commands,
//     mut meshes: ResMut<Assets<Mesh>>,
//     mut materials: ResMut<Assets<StandardMaterial>>,
// ) {
//     let cube_mesh = meshes.add(Cuboid::default());
//     let cube_material = materials.add(Color::rgb(0.8, 0.7, 0.6));
//
//     // Spawn a static cube and a dynamic cube that is connected to it by a distance joint.
//     // let static_cube = commands
//     //     .spawn((
//     //         PbrBundle {
//     //             mesh: cube_mesh.clone(),
//     //             material: cube_material.clone(),
//     //             ..default()
//     //         },
//     //         RigidBody::Static,
//     //         Collider::cuboid(1., 1., 1.),
//     //     ))
//     //     .id();
//     // let dynamic_cube = commands
//     //     .spawn((
//     //         PbrBundle {
//     //             mesh: cube_mesh,
//     //             material: cube_material,
//     //             transform: Transform::from_xyz(-2.0, -0.5, 0.0),
//     //             ..default()
//     //         },
//     //         RigidBody::Dynamic,
//     //         Collider::cuboid(1., 1., 1.),
//     //         MassPropertiesBundle::new_computed(&Collider::cuboid(1.0, 1.0, 1.0), 1.0),
//     //     ))
//     //     .id();
//     //
//     // // Add a distance joint to keep the cubes at a certain distance from each other.
//     // commands.spawn(
//     //     DistanceJoint::new(static_cube, dynamic_cube)
//     //         .with_local_anchor_2(0.5 * Vector::ONE)
//     //         .with_rest_length(1.5)
//     //         .with_compliance(1.0 / 400.0),
//     // );
//
//     // Light
//     commands.spawn(PointLightBundle {
//         point_light: PointLight {
//             intensity: 2_000_000.0,
//             shadows_enabled: true,
//             ..default()
//         },
//         transform: Transform::from_xyz(4.0, 8.0, 4.0),
//         ..default()
//     });
//
//     // Camera
//     commands.spawn(Camera3dBundle {
//         transform: Transform::from_xyz(0.0, 0.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
//         ..default()
//     });
// }





