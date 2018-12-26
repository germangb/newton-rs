pub extern crate ffi;

#[cfg(feature = "sandbox")]
pub mod sandbox;

pub mod body;
pub mod collision;
pub mod joint;
pub mod types;
pub mod world;

use crate::body::NewtonBody;

use std::fmt::Debug;
use std::os::raw;
use std::time::Duration;

type Shared<T> = std::rc::Rc<T>;
type Weak<T> = std::rc::Weak<T>;

pub trait Types {
    type Vector: Copy;
    type Matrix: Copy;
    type Quaternion: Copy;
    type UserData;
}
