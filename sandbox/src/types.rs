use newton_dynamics::traits::NewtonData;

#[derive(Debug, Clone, Copy)]
pub enum SandboxCollisionData {
    Box {
        dx: f32,
        dy: f32,
        dz: f32,
    },
    Capsule {
        radius: f32,
        height: f32,
    },
    Cylinder {
        radius: f32,
        height: f32,
    },
    Sphere {
        rx: f32,
        ry: f32,
        rz: f32,
    },
    Cone {
        radius: f32,
        height: f32,
    }
}

pub enum SandboxData {}

