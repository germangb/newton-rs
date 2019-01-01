use super::Result;

use failure::Fail;

use std::cell::Cell;
use std::marker::PhantomData;
use std::sync::atomic::{AtomicUsize, Ordering};

#[derive(Debug, Fail)]
pub enum LockError {
    /// Can't get a mutable lock.
    #[fail(display = "Cannot acquire a lock: {:?}", writer)]
    AlreadyLocked {
        writer: Option<&'static str>,
        readers: usize,
    },

    /// Can't get a mutable lock. Trying to acquire it would block the current thread
    #[fail(display = "The current thread would block: {:?}", writer)]
    WouldBlock {
        writer: Option<&'static str>,
        readers: usize,
    },

    /// The inner body/collision has been dropped and destroyed
    #[fail(display = "The inner body has been explicitly destroyed elsewhere")]
    Destroyed,

    /// A thread has panic! while holding a mutable lock
    #[fail(display = "Mutable lock is poisoned")]
    Poisoned,
}

pub type Shared<T> = std::sync::Arc<T>;
pub type Weak<T> = std::sync::Weak<T>;

pub type Locked<'a, T> = std::sync::RwLockReadGuard<'a, T>;
pub type LockedMut<'a, T> = std::sync::RwLockWriteGuard<'a, T>;

/// Wraps a `RwLock` + some additional information for error reporting
#[derive(Debug)]
pub struct Lock<T> {
    inner: std::sync::RwLock<T>,

    /// Debug name of the owner of the write lock
    writer: Cell<Option<&'static str>>,
    /// Number of readers with a Read lock
    readers: AtomicUsize,
}

impl<T> Lock<T> {
    pub fn new(inner: T) -> Self {
        Lock {
            inner: std::sync::RwLock::new(inner),
            writer: Cell::new(None),
            readers: AtomicUsize::new(0),
        }
    }

    pub fn read(&self) -> Locked<T> {
        self.try_read().unwrap()
    }

    pub fn try_read(&self) -> Result<Locked<T>> {
        use std::sync::TryLockError;

        match self.inner.try_read() {
            Ok(lock) => Ok(lock),
            Err(TryLockError::WouldBlock) => Err(LockError::WouldBlock {
                writer: self.writer.get(),
                readers: self.readers.load(Ordering::Relaxed),
            }
            .into()),

            // TODO handle poisoning
            _ => unimplemented!(),
        }
    }

    pub fn write(&self, writer: Option<&'static str>) -> LockedMut<T> {
        self.try_write(writer).unwrap()
    }

    pub fn try_write(&self, writer: Option<&'static str>) -> Result<LockedMut<T>> {
        use std::sync::TryLockError;

        match self.inner.try_write() {
            Ok(lock) => {
                self.writer.set(writer);
                Ok(lock)
            }
            Err(TryLockError::WouldBlock) => Err(LockError::WouldBlock {
                writer: self.writer.get(),
                readers: self.readers.load(Ordering::Relaxed),
            }
            .into()),
            Err(TryLockError::Poisoned(e)) => {
                // TODO return poisoned guard
                unimplemented!()
            }

            // TODO handle poisoning
            _ => unimplemented!(),
        }
    }
}
