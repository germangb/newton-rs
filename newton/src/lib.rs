mod pointer;

#[cfg(feature = "sandbox")]
pub mod sandbox;

pub mod body;
pub mod collision;
pub mod constraint;
pub mod mesh;
pub mod prelude;
pub mod world;

pub unsafe trait NewtonApp: std::fmt::Debug {
    type Vector: Copy;
    type Matrix: Copy;
    type Quaternion: Copy;
}
