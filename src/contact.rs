use super::body::WeakBody;

use std::marker::PhantomData;
use std::ops::{Deref, DerefMut};

#[derive(Debug, Clone)]
pub struct Contact<T>(WeakBody<T>, WeakBody<T>);

/// Container of collisions
#[derive(Debug)]
pub struct Contacts<T>(Vec<Contact<T>>);

impl<T> Contacts<T> {
    pub fn new() -> Self {
        Self::with_capacity(0)
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Contacts(Vec::with_capacity(capacity))
    }
}

impl<T> Deref for Contacts<T> {
    type Target = Vec<Contact<T>>;
    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T> DerefMut for Contacts<T> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
