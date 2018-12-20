use crate::body::NewtonBody;
use crate::NewtonConfig;

/// Implementation of the `ffi::NewtonApplyForceAndTorque` callback
pub trait ForceAndTorque<V> {
    fn force_and_torque(body: NewtonBody<V>);
}

/// Force and torque callback where gravity is applied to the body.
pub enum Gravity {}

/// Don't apply any forces to the body
pub enum DoNothing {}

impl<V> ForceAndTorque<V> for Gravity
where
    V: NewtonConfig,
{
    fn force_and_torque(body: NewtonBody<V>) {
        body.set_force(V::GRAVITY)
    }
}

impl<V> ForceAndTorque<V> for DoNothing {
    fn force_and_torque(_: NewtonBody<V>) {}
}
