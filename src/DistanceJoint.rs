use bevy::prelude::*;
use bevy::reflect::erased_serde::__private::serde;
use crate::query::*;
use crate::rotation::*;

#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct DistanceJoint {
    /// First entity constrained by the joint.
    pub entity1: Entity,
    /// Second entity constrained by the joint.
    pub entity2: Entity,
    /// Attachment point on the first body.
    pub local_anchor1: Vec3,
    /// Attachment point on the second body.
    pub local_anchor2: Vec3,
    /// The distance the attached bodies will be kept relative to each other.
    pub rest_length: f32,
    /// The extents of the allowed relative translation between the attached bodies.
    //pub length_limits: Option<DistanceLimit>,
    /// Linear damping applied by the joint.
    pub damping_linear: f32,
    /// Angular damping applied by the joint.
    pub damping_angular: f32,
    /// Lagrange multiplier for the positional correction.
    pub lagrange: f32,
    /// The joint's compliance, the inverse of stiffness, has the unit meters / Newton.
    pub compliance: f32,
    /// The force exerted by the joint.
    pub force: Vec3,
}

impl DistanceJoint {
    pub fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            local_anchor1: Vec3::ZERO,
            local_anchor2: Vec3::ZERO,
            rest_length: 0.0,
            damping_linear: 0.0,
            damping_angular: 0.0,
            lagrange: 0.0,
            compliance: 0.0,
            force: Vec3::ZERO,
        }
    }
    pub fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }
    pub fn with_compliance(self, compliance: f32) -> Self {
        Self { compliance, ..self }
    }

    pub fn with_local_anchor_1(self, anchor: Vec3) -> Self {
        Self {
            local_anchor1: anchor,
            ..self
        }
    }

    pub fn with_local_anchor_2(self, anchor: Vec3) -> Self {
        Self {
            local_anchor2: anchor,
            ..self
        }
    }

    pub fn with_linear_velocity_damping(self, damping: f32) -> Self {
        Self {
            damping_linear: damping,
            ..self
        }
    }

    pub fn with_angular_velocity_damping(self, damping: f32) -> Self {
        Self {
            damping_angular: damping,
            ..self
        }
    }

    pub fn local_anchor_1(&self) -> Vec3 {
        self.local_anchor1
    }

    pub fn local_anchor_2(&self) -> Vec3 {
        self.local_anchor2
    }

    pub fn damping_linear(&self) -> f32 {
        self.damping_linear
    }

    pub fn damping_angular(&self) -> f32 {
        self.damping_angular
    }
    pub fn clear_lagrange_multipliers(&mut self) {
        self.lagrange = 0.0;
    }




    /// Constrains the distance the bodies with no constraint on their rotation.
    ///
    /// Returns the force exerted by this constraint.
    pub fn compute_lagrange_update(
        &self,
        lagrange: f32,
        c: f32,
        gradients: &[Vec3],
        inverse_masses: &[f32],
        compliance: f32,
        dt: f32,
    ) -> f32 {
        // Compute the sum of all inverse masses multiplied by the squared lengths of the corresponding gradients.
        let w_sum = inverse_masses
            .iter()
            .enumerate()
            .fold(0.0, |acc, (i, w)| acc + *w * gradients[i].length_squared());

        // Avoid division by zero
        if w_sum <= f32::EPSILON {
            return 0.0;
        }

        // tilde_a = a/h^2
        let tilde_compliance = compliance / dt.powi(2);

        (-c - tilde_compliance * lagrange) / (w_sum + tilde_compliance)
    }
    fn get_delta_rot(rot: Rotation, inverse_inertia: Mat3, r: Vec3, p: Vec3) -> Rotation {
        // Equation 8/9
        Rotation(Quat::from_vec4(0.5 * (inverse_inertia * r.cross(p)).extend(0.0)) * rot.0)
    }
    pub fn apply_positional_correction(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        delta_lagrange: f32,
        direction: Vec3,
        r1: Vec3,
        r2: Vec3,
    ) -> Vec3 {
        if delta_lagrange.abs() <= f32::EPSILON {
            return Vec3::ZERO;
        }

        // Compute positional impulse
        let p = delta_lagrange * direction;
        let rot1 = *body1.rotation;
        let rot2 = *body2.rotation;

        let inv_mass1 = body1.effective_inv_mass();
        let inv_mass2 = body2.effective_inv_mass();
        let inv_inertia1 = body1.effective_world_inv_inertia();
        let inv_inertia2 = body2.effective_world_inv_inertia();

        // Apply positional and rotational updates
        if body1.rb.is_dynamic() && body1.dominance() <= body2.dominance() {
            body1.accumulated_translation.0 += p * inv_mass1;
            *body1.rotation += Self::get_delta_rot(rot1, inv_inertia1, r1, p);


            {
                // In 3D, subtracting quaternions like above can result in unnormalized rotations,
                // which causes stability issues (see #235) and panics when trying to rotate unit vectors.
                // TODO: It would be nice to avoid normalization if possible.
                //       Maybe the math above can be done in a way that keeps rotations normalized?
                body1.rotation.0 = body1.rotation.0.normalize();
            }
        }
        if body2.rb.is_dynamic() && body2.dominance() <= body1.dominance() {
            body2.accumulated_translation.0 -= p * inv_mass2;
            *body2.rotation -= Self::get_delta_rot(rot2, inv_inertia2, r2, p);

            {
                // See comments for `body1` above.
                body2.rotation.0 = body2.rotation.0.normalize();
            }
        }

        p
    }

    /// Sets the minimum and maximum distances between the attached bodies.
    // pub fn with_limits(self, min: f32, max: f32) -> Self {
    //     Self {
    //         length_limits: Some(DistanceLimit::new(min, max)),
    //         ..self
    //     }
    // }

    /// Sets the joint's rest length, or distance the bodies will be kept at.
    pub fn with_rest_length(self, rest_length: f32) -> Self {
        Self {
            rest_length,
            ..self
        }
    }
    pub fn compute_force(&self, lagrange: f32, direction: Vec3, dt: f32) -> Vec3 {
        lagrange * direction / dt.powi(2)
    }
}