use super::Result;

use failure::Fail;

use std::cell::Cell;
use std::marker::PhantomData;
use std::ops::{Deref, DerefMut};
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

#[derive(Debug)]
pub struct Locked<'a, T>(std::sync::RwLockReadGuard<'a, T>);

#[derive(Debug)]
pub struct LockedMut<'a, T>(std::sync::RwLockWriteGuard<'a, T>);

impl<'a, T> Deref for Locked<'a, T> {
    type Target = T;
    #[inline]
    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'a, T> Deref for LockedMut<'a, T> {
    type Target = T;
    #[inline]
    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'a, T> DerefMut for LockedMut<'a, T> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.deref_mut()
    }
}

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
        let lock = self.inner.read().unwrap();
        Locked(lock)
    }

    pub fn try_read(&self) -> Result<Locked<T>> {
        use std::sync::TryLockError;

        match self.inner.try_read() {
            Ok(lock) => Ok(Locked(lock)),
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
        let lock = self.inner.write().unwrap();
        self.writer.set(writer);
        LockedMut(lock)
    }

    pub fn try_write(&self, writer: Option<&'static str>) -> Result<LockedMut<T>> {
        use std::sync::TryLockError;

        match self.inner.try_write() {
            Ok(lock) => {
                self.writer.set(writer);
                Ok(LockedMut(lock))
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
