pub extern crate ffi;

#[cfg(feature = "sandbox")]
pub mod sandbox;

pub mod body;
pub mod collision;
pub mod joint;
pub mod types;
pub mod world;

use self::body::NewtonBody;

use std::fmt::Debug;

pub trait Types {
    type Vector: Copy;
    type Matrix: Copy;
    type Quaternion: Copy;
}

/*
pub trait Application: Sized + Debug {
    type Types;

    //fn force_and_torque(body: &mut NewtonBody<Self>) {}
}
*/

// callbacks

pub trait ForceAndTorque<T> {
    fn force_and_torque(body: &mut NewtonBody<T>);
}
