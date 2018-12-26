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

type Shared<T> = std::sync::Arc<T>;
type Weak<T> = std::sync::Weak<T>;

type Lock<T> = std::sync::RwLock<T>;
type Locked<'a, T> = std::sync::RwLockReadGuard<'a, T>;
type LockedMut<'a, T> = std::sync::RwLockWriteGuard<'a, T>;

pub trait Types {
    type Vector: Copy + Sync + Send;
    type Matrix: Copy + Sync + Send;
    type Quaternion: Copy + Sync + Send;
    type UserData: Sync + Send;
}
