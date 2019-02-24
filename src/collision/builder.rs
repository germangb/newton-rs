use std::cell::Cell;

use crate::ffi;
use crate::{Handle, HandleInner, Vec3};

use super::{Compound, ConvexShape, NewtonCollision, Scene, Tree};

/// Type to add/remove collisions from a compound.
pub struct CompoundBuilder<'a, 'b> {
    pub(super) compound: &'b Compound<'a>,
}

/// Add/remove collisions from a scene collision.
pub struct SceneBuilder<'a, 'b> {
    pub(super) scene: &'b Scene<'a>,
}

/// Type to define the geometry of a tree collision.
pub struct TreeBuilder<'a, 'b> {
    pub(super) optimize: bool,
    pub(super) tree: &'b Tree<'a>,
}

impl<'a, 'b> CompoundBuilder<'a, 'b> {
    /// Adds a collision to the compound
    pub fn add<C>(&self, col: &C) -> Handle
        where C: ConvexShape
    {
        let comp = self.compound.raw;
        let sub = col.as_raw();
        unsafe {
            let handle = ffi::NewtonCompoundCollisionAddSubCollision(comp, sub);
            Handle::from_ptr(handle as _)
        }
    }

    /// Removes a collision from the compound
    pub fn remove(&self, handle: Handle) {
        let comp = self.compound.raw;
        match handle.inner() {
            HandleInner::Pointer(ptr) => unsafe {
                ffi::NewtonCompoundCollisionRemoveSubCollision(comp, ptr as _);
            },
            HandleInner::Index(idx) => unsafe {
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
        where C: NewtonCollision
    {
        let comp = self.scene.raw;
        let sub = col.as_raw();
        unsafe {
            let handle = ffi::NewtonSceneCollisionAddSubCollision(comp, sub);
            Handle::from_ptr(handle as _)
        }
    }

    /// Removes a collision from the scene.
    pub fn remove(&self, handle: Handle) {
        let comp = self.scene.raw;
        match handle.inner() {
            HandleInner::Pointer(ptr) => unsafe {
                ffi::NewtonSceneCollisionRemoveSubCollision(comp, ptr as _);
            },
            HandleInner::Index(idx) => unsafe {
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

impl<'a, 'b> TreeBuilder<'a, 'b> {
    // TODO is an IntoIterator<Item=Vec3> better?
    // Adds a single triangle to the collision tree
    pub fn add<I: IntoIterator<Item = Vec3>>(&self, verts: I, attr: i32) {
        unsafe {
            let face: Vec<Vec3> = verts.into_iter().collect();
            ffi::NewtonTreeCollisionAddFace(self.tree.raw,
                                            face.len() as _,
                                            face.as_ptr() as _,
                                            12,
                                            attr);
        }
    }

    /// Optimizes the mesh and finishes the build process.
    pub fn optimize(mut self) {
        self.optimize = true;
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

impl<'a, 'b> Drop for TreeBuilder<'a, 'b> {
    fn drop(&mut self) {
        unsafe {
            let optimize = if self.optimize { 1 } else { 0 };
            ffi::NewtonTreeCollisionEndBuild(self.tree.raw, optimize);
        }
    }
}
