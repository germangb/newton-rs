pub mod body;
pub mod collision;
#[cfg(feature = "sandbox2")]
pub mod sandbox;
pub mod world;

use std::fmt::Debug;

pub trait Types {
    type Vector: Copy;
    type Matrix: Copy;
    type Quaternion: Copy;
}

pub trait Application: Debug {
    type Types;
}
