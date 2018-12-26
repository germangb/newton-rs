pub extern crate ffi;

#[cfg(feature = "sandbox")]
pub mod sandbox;

pub mod body;
pub mod collision;
pub mod joint;
pub mod types;
pub mod world;

use std::fmt::Debug;
use std::time::Duration;

type Shared<T> = std::sync::Arc<T>;
type Weak<T> = std::sync::Weak<T>;

pub trait Types {
    type Vector: Copy;
    type Matrix: Copy;
    type Quaternion: Copy;
}
