
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
        .add_plugins(DefaultPlugins)
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_plugins(LogDiagnosticsPlugin::default())
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



