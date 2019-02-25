use std::marker::PhantomData;

use crate::ffi;
use crate::joint::Constraint;
use crate::newton::Newton;

#[derive(Debug)]
pub struct Contacts<'a> {
    _phantom: PhantomData<&'a ()>,
}

#[derive(Debug)]
pub struct Joints<'a> {
    pub(crate) joint: *const ffi::NewtonJoint,
    pub(crate) body: *const ffi::NewtonBody,
    pub(crate) _phantom: PhantomData<&'a ()>,
}

impl<'a> Iterator for Joints<'a> {
    type Item = Constraint<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let current = self.joint;
        if current.is_null() {
            None
        } else {
            unsafe {
                let next = ffi::NewtonBodyGetNextJoint(self.body, current);
                self.joint = next;
                Some(Constraint::from_raw(current, false))
            }
        }
    }
}
