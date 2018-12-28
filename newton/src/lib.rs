pub extern crate ffi;

#[cfg(feature = "sandbox")]
pub mod sandbox;

pub mod body;
pub mod collision;
pub mod joint;
pub mod types;
pub mod world;

use std::{error, fmt, os::raw, time::Duration};

#[cfg(feature = "sync")]
pub unsafe trait Types: Clone {
    type Vector: Copy + Sync + Send;
    type Matrix: Copy + Sync + Send;
    type Quaternion: Copy + Sync + Send;
}

#[cfg(not(feature = "sync"))]
pub unsafe trait Types: Clone {
    type Vector: Copy;
    type Matrix: Copy;
    type Quaternion: Copy;
}

#[derive(Debug)]
pub enum Error {
    AlreadyLocked {
        owner: Option<&'static str>,
    },
    #[cfg(feature = "sync")]
    WouldBlock {
        owner: Option<&'static str>,
    },
    #[cfg(feature = "sync")]
    Poisoned,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Error::AlreadyLocked { .. } => write!(f, "AlreadyLocked"),
            #[cfg(feature = "sync")]
            Error::WouldBlock { .. } => write!(f, "WouldBlock"),
            #[cfg(feature = "sync")]
            Error::Poisoned => write!(f, "Poisoned"),
        }
    }
}

impl error::Error for Error {
    fn description(&self) -> &str {
        match self {
            Error::AlreadyLocked { .. } => "The world is still locked",
            #[cfg(feature = "sync")]
            Error::WouldBlock { .. } => "Trying to aquire this lock would block the current thread",
            #[cfg(feature = "sync")]
            Error::Poisoned => "A thread likely panicked while still holding the lock",
        }
    }
}

pub type Result<T> = std::result::Result<T, Error>;

#[cfg(feature = "sync")]
#[derive(Debug)]
struct Lock<T>(std::sync::RwLock<T>);

#[cfg(feature = "sync")]
type Shared<T> = std::sync::Arc<T>;

#[cfg(feature = "sync")]
type Weak<T> = std::sync::Weak<T>;

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
        self.try_read().unwrap()
    }

    fn try_read(&self) -> Result<Locked<T>> {
        use std::sync::TryLockError;

        match self.0.try_read() {
            Ok(lock) => Ok(lock),
            Err(TryLockError::WouldBlock) => Err(Error::WouldBlock { owner: None }),

            // TODO handle poisoning
            _ => unimplemented!(),
        }
    }

    fn write(&self) -> LockedMut<T> {
        self.try_write().unwrap()
    }

    fn try_write(&self) -> Result<LockedMut<T>> {
        use std::sync::TryLockError;

        match self.0.try_write() {
            Ok(lock) => Ok(lock),
            Err(TryLockError::WouldBlock) => Err(Error::WouldBlock { owner: None }),

            // TODO handle poisoning
            _ => unimplemented!(),
        }
    }
}

#[cfg(not(feature = "sync"))]
#[derive(Debug)]
struct Lock<T>(std::cell::RefCell<T>);

#[cfg(not(feature = "sync"))]
type Shared<T> = std::rc::Rc<T>;

#[cfg(not(feature = "sync"))]
type Weak<T> = std::rc::Weak<T>;

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
        self.try_read().unwrap()
    }

    fn try_read(&self) -> Result<Locked<T>> {
        match self.0.borrow() {
            Ok(lock) => Ok(lock),
            Err(_) => Err(Error::AlreadyLocked),
        }
    }

    fn write(&self) -> LockerMut<T> {
        self.try_write().unwrap()
    }

    fn try_write(&self) -> Result<LockedMut<T>> {
        match self.0.borrow_mut() {
            Ok(lock) => Ok(lock),
            Err(_) => Err(Error::AlreadyLocked),
        }
    }
}
