pub extern crate ffi;

#[cfg(feature = "sandbox")]
pub mod sandbox;

pub mod body;
pub mod collision;
pub mod joint;
pub mod types;
pub mod world;

use std::fmt::Debug;
use std::os::raw;
use std::time::Duration;

pub trait Types {
    type Vector: Copy + Sync + Send;
    type Matrix: Copy + Sync + Send;
    type Quaternion: Copy + Sync + Send;
    type UserData: Sync + Send;
}

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug)]
pub enum Error {
    AlreadyLocked,
    #[cfg(feature = "sync")]
    WouldBlock,
    #[cfg(feature = "sync")]
    Poisoned,
}

#[cfg(feature = "sync")]
type Shared<T> = std::sync::Arc<T>;
#[cfg(feature = "sync")]
type Weak<T> = std::sync::Weak<T>;

#[cfg(feature = "sync")]
#[derive(Debug)]
struct Lock<T>(std::sync::RwLock<T>);
#[cfg(feature = "sync")]
type Locked<'a, T> = std::sync::RwLockReadGuard<'a, T>;
#[cfg(feature = "sync")]
type LockedMut<'a, T> = std::sync::RwLockWriteGuard<'a, T>;

#[cfg(feature = "sync")]
impl<T> Lock<T> {
    fn new(inner: T) -> Self {
        Lock(std::sync::RwLock::new(inner))
    }

    fn read(&self) -> Locked<T> {
        self.0.read().unwrap()
    }

    fn write(&self) -> LockedMut<T> {
        self.0.write().unwrap()
    }
}

#[cfg(not(feature = "sync"))]
type Shared<T> = std::rc::Rc<T>;
#[cfg(not(feature = "sync"))]
type Weak<T> = std::rc::Weak<T>;

#[cfg(not(feature = "sync"))]
#[derive(Debug)]
struct Lock<T>(std::cell::RefCell<T>);
#[cfg(not(feature = "sync"))]
type Locked<'a, T> = std::cell::Ref<'a, T>;
#[cfg(not(feature = "sync"))]
type LockedMut<'a, T> = std::cell::RefMut<'a, T>;

#[cfg(not(feature = "sync"))]
impl<T> Lock<T> {
    fn new(inner: T) -> Self {
        Lock(std::cell::RefCell::new(inner))
    }

    fn read(&self) -> Locked<T> {
        self.0.borrow()
    }

    fn write(&self) -> LockedMut<T> {
        self.0.borrow_mut()
    }
}
