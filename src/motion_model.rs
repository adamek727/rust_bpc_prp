use crate::primitives::Pose;
use crate::robot::RobotPhysicalConstrains;

#[derive(Clone, Copy)]
pub struct WheelSpeeds {
    left_wheel_speed: f32,
    right_wheel_speed: f32,
}

impl WheelSpeeds {

    pub fn new(left_speed: f32, right_speed: f32) -> WheelSpeeds {
        WheelSpeeds {
            left_wheel_speed: left_speed,
            right_wheel_speed: right_speed
        }
    }

    pub fn left_wheel_speed(&self) -> f32 { self.left_wheel_speed }
    pub fn right_wheel_speed(&self) -> f32 { self.right_wheel_speed }
}

#[derive(Clone, Copy)]
pub struct MotionParameters {
    linear_velocity: f32,
    angular_velocity: f32,
}

impl MotionParameters {

    pub fn new(linerar_velocity: f32, angular_velocity: f32) -> MotionParameters {
        MotionParameters {
            linear_velocity: linerar_velocity,
            angular_velocity,
        }
    }

    pub fn linear_velocity(&self) -> f32 { self.linear_velocity }
    pub fn angular_velocity(&self) -> f32 { self.angular_velocity }
}


pub struct MotionModel {
    wheel_perimeter: f32,
    robot_constrains: RobotPhysicalConstrains,
    robot_pose: Pose,
    robot_orientation: f32,
}

impl MotionModel {

    pub fn new(robot_constrains: RobotPhysicalConstrains, init_pose: Pose, robot_orientation: f32) -> MotionModel {
        let wheel_radius = robot_constrains.wheel_radius();
        MotionModel {
            robot_constrains,
            wheel_perimeter: 2.0 * std::f32::consts::PI * wheel_radius,
            robot_pose: init_pose,
            robot_orientation,
        }
    }

    pub fn get_pose(&self) -> Pose { self.robot_pose }
    pub fn get_orientation(&self) -> f32 { self.robot_orientation }

    pub fn get_wheel_speeds(&self, required: MotionParameters) -> WheelSpeeds {

        WheelSpeeds {
            left_wheel_speed: (2.0 * required.linear_velocity() - required.angular_velocity() * self.robot_constrains.chassis_base()) / (2.0 * self.robot_constrains.wheel_radius()),
            right_wheel_speed: (2.0 * required.linear_velocity() + required.angular_velocity() * self.robot_constrains.chassis_base()) / (2.0 * self.robot_constrains.wheel_radius()),
        }
    }


    pub fn wheel_speed_to_microsteps_with_saturation(&self, wheel_ang_speeds: WheelSpeeds) -> WheelSpeeds {

        let left_microsteps = wheel_ang_speeds.left_wheel_speed / (2.0 * std::f32::consts::PI) * self.robot_constrains.microsteps_per_rotation() as f32;
        let right_microsteps = wheel_ang_speeds.right_wheel_speed / (2.0 * std::f32::consts::PI) * self.robot_constrains.microsteps_per_rotation() as f32;

        let mut saturation = 1.0;
        if left_microsteps > self.robot_constrains.max_microsteps_per_sec() {
            saturation = self.robot_constrains.max_microsteps_per_sec() / left_microsteps;
        }
        if right_microsteps > self.robot_constrains.max_microsteps_per_sec() {
            saturation = saturation.min(self.robot_constrains.max_microsteps_per_sec() / right_microsteps);
        }

        WheelSpeeds {
            left_wheel_speed: left_microsteps * saturation,
            right_wheel_speed: right_microsteps * saturation,
        }
    }

    pub fn microsteps_to_wheel_speed(&self, wheel_microsteps_speeds: WheelSpeeds) -> WheelSpeeds {
        WheelSpeeds {
            left_wheel_speed: wheel_microsteps_speeds.left_wheel_speed() / self.robot_constrains.microsteps_per_rotation() as f32 * (2.0 * std::f32::consts::PI),
            right_wheel_speed: wheel_microsteps_speeds.right_wheel_speed() / self.robot_constrains.microsteps_per_rotation() as f32 * (2.0 * std::f32::consts::PI),
        }
    }

    pub fn wheel_speed_to_robot_motion_parameters(&self, wheel_speed: WheelSpeeds) -> MotionParameters {

        MotionParameters {
            linear_velocity: self.robot_constrains.wheel_radius() / 2.0 * (wheel_speed.right_wheel_speed + wheel_speed.left_wheel_speed),
            angular_velocity: self.robot_constrains.wheel_radius() / self.robot_constrains.chassis_base() * (wheel_speed.right_wheel_speed - wheel_speed.left_wheel_speed),
        }
    }

    pub fn integrate_robot_motion(&mut self, motion_params: MotionParameters, delta_t: f32) {
        let dx = motion_params.linear_velocity * self.robot_orientation.cos() * delta_t;
        let dy = motion_params.linear_velocity * self.robot_orientation.sin() * delta_t;
        let d_fi = motion_params.angular_velocity * delta_t;

        self.robot_pose = Pose::new(self.robot_pose.x() + dx, self.robot_pose.y() + dy, self.robot_pose.z());
        self.robot_orientation += d_fi;
    }
}

