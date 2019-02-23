use crate::ffi;
use crate::newton::Newton;

use super::Body;
use std::marker::PhantomData;

/// Iterator over all the bodies in a NewtonWorld.
#[derive(Debug)]
pub struct Bodies<'a> {
    pub(crate) newton: *const ffi::NewtonWorld,
    pub(crate) next: *const ffi::NewtonBody,
    pub(crate) _phantom: PhantomData<&'a ()>,
}

impl<'a> Iterator for Bodies<'a> {
    type Item = Body<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let current = self.next;
        if current.is_null() {
            None
        } else {
            unsafe {
                self.next = ffi::NewtonWorldGetNextBody(self.newton, current);
                Some(Body::from_raw(current, false))
            }
        }
    }
}
