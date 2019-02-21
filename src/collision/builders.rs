use crate::ffi;
use crate::Handle;

use super::{CompoundCollision, NewtonCollision, SceneCollision, TreeCollision};

pub struct CompoundBuilder<'a, 'b> {
    pub(super) compound: &'b CompoundCollision<'a>,
}

pub struct SceneBuilder<'a, 'b> {
    pub(super) scene: &'b SceneCollision<'a>,
}

pub struct TreeBuilder<'a, 'b> {
    pub(super) tree: &'b TreeCollision<'a>,
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
            Handle::from_raw(handle)
        }
    }

    /// Removes a collision from the compound
    pub fn remove(&self, handle: Handle) {
        let comp = self.compound.raw;
        unsafe {
            ffi::NewtonCompoundCollisionRemoveSubCollision(comp, handle.as_raw());
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
            Handle::from_raw(handle)
        }
    }

    /// Removes a collision from the scene.
    pub fn remove(&self, handle: Handle) {
        let comp = self.scene.raw;
        unsafe {
            ffi::NewtonSceneCollisionRemoveSubCollision(comp, handle.as_raw());
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
