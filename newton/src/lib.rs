pub mod body;
pub mod collision;
#[cfg(feature = "sandbox")]
pub mod sandbox;
pub mod world;

use self::body::NewtonBody;

use std::fmt::Debug;

pub trait Types {
    type Vector: Copy;
    type Matrix: Copy;
    type Quaternion: Copy;
}

pub trait Application: Sized + Debug {
    type Types;

    fn force_and_torque(body: &mut NewtonBody<Self>) {}
}
