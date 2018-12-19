use crate::body::NewtonBody;
use crate::NewtonConfig;

/// Implementation of the `ffi::NewtonApplyForceAndTorque` callback
pub trait ForceAndTorque<V> {
    fn force_and_torque(body: NewtonBody<V>);
}

/// Force and torque callback where gravity is applied to the body.
pub enum Gravity {}

impl<V> ForceAndTorque<V> for Gravity
where
    V: NewtonConfig,
{
    fn force_and_torque(body: NewtonBody<V>) {
        body.set_force(V::GRAVITY)
    }
}
