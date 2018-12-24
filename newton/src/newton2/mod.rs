pub mod world;
pub mod body;
pub mod collision;
//pub mod cell;

use std::fmt::Debug;

pub trait Types {
    type Vector: Copy;
    type Matrix: Copy;
    type Quaternion: Copy;
}

pub trait Application: Debug {
    type Types;
}
