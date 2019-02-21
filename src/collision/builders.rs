use crate::ffi;
use crate::Handle;

use super::{Compound, NewtonCollision, Scene, Tree};

pub struct CompoundBuilder<'a, 'b> {
    pub(super) compound: &'b Compound<'a>,
}

pub struct SceneBuilder<'a, 'b> {
    pub(super) scene: &'b Scene<'a>,
}

pub struct TreeBuilder<'a, 'b> {
    pub(super) tree: &'b Tree<'a>,
}

impl<'a, 'b> CompoundBuilder<'a, 'b> {
    /// Adds a collision to the compound
    pub fn add<C>(&self, col: &C) -> Handle
    where
        C: NewtonCollision,
    {
        let comp = self.compound.raw;
        let sub = col.as_raw();
        unsafe {
            let handle = ffi::NewtonCompoundCollisionAddSubCollision(comp, sub);
            Handle::Pointer(handle as _)
        }
    }

    /// Removes a collision from the compound
    pub fn remove(&self, handle: Handle) {
        let comp = self.compound.raw;
        match handle {
            Handle::Pointer(ptr) => unsafe {
                ffi::NewtonCompoundCollisionRemoveSubCollision(comp, ptr as _);
            },
            Handle::Index(idx) => unsafe {
                let ptr = ffi::NewtonCompoundCollisionGetNodeByIndex(comp, idx as _);
                if !ptr.is_null() {
                    ffi::NewtonCompoundCollisionRemoveSubCollisionByIndex(comp, idx as _);
                } else {
                    //TODO error?
                }
            },
        }
    }
}

impl<'a, 'b> SceneBuilder<'a, 'b> {
    /// Adds a collision to the scene
    pub fn add<C>(&self, col: &C) -> Handle
    where
        C: NewtonCollision,
    {
        let comp = self.scene.raw;
        let sub = col.as_raw();
        unsafe {
            let handle = ffi::NewtonSceneCollisionAddSubCollision(comp, sub);
            Handle::Pointer(handle as _)
        }
    }

    /// Removes a collision from the scene.
    pub fn remove(&self, handle: Handle) {
        let comp = self.scene.raw;
        match handle {
            Handle::Pointer(ptr) => unsafe {
                ffi::NewtonSceneCollisionRemoveSubCollision(comp, ptr as _);
            },
            Handle::Index(idx) => unsafe {
                let ptr = ffi::NewtonSceneCollisionGetNodeByIndex(comp, idx as _);
                if !ptr.is_null() {
                    ffi::NewtonSceneCollisionRemoveSubCollisionByIndex(comp, idx as _);
                } else {
                    //TODO error?
                }
            },
        }
    }
}

impl<'a, 'b> Drop for CompoundBuilder<'a, 'b> {
    fn drop(&mut self) {
        unsafe {
            ffi::NewtonCompoundCollisionEndAddRemove(self.compound.raw);
        }
    }
}

impl<'a, 'b> Drop for SceneBuilder<'a, 'b> {
    fn drop(&mut self) {
        unsafe {
            ffi::NewtonSceneCollisionEndAddRemove(self.scene.raw);
        }
    }
}
