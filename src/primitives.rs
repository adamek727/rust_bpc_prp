#[derive(Copy, Clone)]
pub struct Pose {
    x: f32,
    y: f32,
    z: f32,
}

impl Pose {

    pub fn new(x: f32, y: f32, z:f32) -> Pose {
        Pose { x, y, z }
    }

    pub fn x(&self) -> f32 { self.x }
    pub fn y(&self) -> f32 { self.y }
    pub fn z(&self) -> f32 { self.z }
}