use std::io::SeekFrom::Start;

#[derive(Copy, Clone)]
pub enum RobotStateMachine {
    Initialization,
    Calibration,
    LineFollowing,
    Fail,
}

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

pub struct RobotPhysicalConstrains {
    no_of_sensors: usize,
    sensor_poses: Vec<Pose>,
    chassis_base: f32,
    wheel_radius: f32,
    max_acceleration_microsteps_per_sec: f32,
    max_microsteps_per_sec :f32,
}

impl RobotPhysicalConstrains {
    pub fn new(no_of_sensors: usize,
               sensor_poses: Vec<Pose>,
               chassis_base: f32,
               wheel_radius: f32,
               max_acceleration_microsteps_per_sec: f32,
               max_microsteps_per_sec: f32) -> RobotPhysicalConstrains {
        RobotPhysicalConstrains { no_of_sensors, sensor_poses, chassis_base, wheel_radius, max_acceleration_microsteps_per_sec, max_microsteps_per_sec}
    }

    pub fn no_of_sensors(&self) -> usize { self.no_of_sensors }
    pub fn chassis_base(&self) -> f32 { self.chassis_base }
    pub fn wheel_radius(&self) -> f32 { self.wheel_radius }
    pub fn sensor_pose_for_index(&self, sensor_index: usize) -> Pose {
        self.sensor_poses[sensor_index]
    }
    pub fn max_acceleration_microsteps_per_sec(&self) -> f32 { self.max_acceleration_microsteps_per_sec }
    pub fn max_microsteps_per_sec(&self) -> f32 { self.max_microsteps_per_sec }
}

pub struct RobotStats {
    sensor_values: Vec<u16>,
    left_wheel_microsteps_per_sec: f32,
    right_wheel_microsteps_per_sec: f32,
    state: RobotStateMachine,
}

impl RobotStats {

    pub fn new(no_of_sensors: usize) -> RobotStats {
        RobotStats {
            sensor_values: vec![0; no_of_sensors],
            left_wheel_microsteps_per_sec: 0.0,
            right_wheel_microsteps_per_sec: 0.0,
            state: RobotStateMachine::Initialization,
        }
    }

    pub fn sensor_values_for_index(&self, sensor_index: usize) -> u16 { self.sensor_values[sensor_index] }
    pub fn left_wheel_microsteps_per_sec(&self) -> f32 { self.left_wheel_microsteps_per_sec }
    pub fn right_wheel_microsteps_per_sec(&self) -> f32 { self.right_wheel_microsteps_per_sec }
    pub fn state(&self) -> RobotStateMachine { self.state }

    pub fn set_value_for_index(&mut self, sensor_index: usize, val: u16) { self.sensor_values[sensor_index] = val;}
    pub fn set_left_wheel_microsteps_per_sec(&mut self, microsteps: f32) {self.left_wheel_microsteps_per_sec = microsteps;}
    pub fn set_right_wheel_microsteps_per_sec(&mut self, microsteps: f32) {self.right_wheel_microsteps_per_sec = microsteps;}
    pub fn set_state(&mut self, state: RobotStateMachine) {self.state = state}
}