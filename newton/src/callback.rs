use crate::body::Body;
use crate::NewtonConfig;

use std::time::Duration;

/// Implementation of the `ffi::NewtonApplyForceAndTorque` callback
pub trait ForceAndTorque<V> {
    fn force_and_torque(body: Body<V>, step: Duration);
}

/// Force and torque callback where gravity is applied to the body.
pub enum Gravity {}

/// Don't apply any forces to the body
pub enum DoNothing {}

impl<V> ForceAndTorque<V> for Gravity
where
    V: NewtonConfig,
{
    // TODO use generic to multiply mass
    fn force_and_torque(body: Body<V>, _: Duration) {
        //body.set_force(V::GRAVITY)
    }
}

impl<V> ForceAndTorque<V> for DoNothing {
    fn force_and_torque(_: Body<V>, _: Duration) {}
}
