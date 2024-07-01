use bevy::prelude::*;
use crate::components::*;
use crate::rotation::*;
pub(crate) fn get_rotated_inertia_tensor(inertia_tensor: Mat3, rot: Quat) -> Mat3 {
    let rot_mat3 = Mat3::from_quat(rot);
    (rot_mat3 * inertia_tensor) * rot_mat3.transpose()
}


pub(crate) fn get_pos_translation(
    com_translation: &AccumulatedTranslation,
    previous_rotation: &Rotation,
    rotation: &Rotation,
    center_of_mass: &CenterOfMass,
) -> Vec3 {
    com_translation.0 + previous_rotation.rotate(center_of_mass.0)
        - rotation.rotate(center_of_mass.0)
}

