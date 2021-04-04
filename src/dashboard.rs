use termion;
use crate::primitives::Pose;
use crate::motion_model::{WheelSpeeds, MotionParameters};


pub struct Dashboard {
    pose: Pose,
    wheel_speed_required: WheelSpeeds,
    wheel_speed_actual: WheelSpeeds,
    motion_params_required: MotionParameters,
    motion_params_actual: MotionParameters,
}

impl Dashboard {

    pub fn new() -> Dashboard {
        Dashboard {
            pose: Pose::new(0.0, 0.0, 0.0),
            wheel_speed_required: WheelSpeeds::new(0.0, 0.0),
            wheel_speed_actual: WheelSpeeds::new(0.0, 0.0),
            motion_params_required: MotionParameters::new(0.0, 0.0),
            motion_params_actual: MotionParameters::new(0.0, 0.0),
        }
    }

    pub fn render(&self) {

        print!("{}{}", termion::clear::All, termion::cursor::Goto(1, 1));
        println!("{}", str::repeat("* ", 20));

        println!(" - Robot's Position - ");
        println!();
        println!("Pose -> x:\t{}\ty:\t{}", self.pose.x(), self.pose.y());

        println!();
        println!(" - Motion Params - ");
        println!();
        println!("Required -> lin vel:\t{:.2}\tang. vel:\t{:.2}", self.motion_params_required.linear_velocity(), self.motion_params_required.angular_velocity());
        println!("Actual   -> lin vel:\t{:.2}\tang. vel:\t{:.2}", self.motion_params_actual.linear_velocity(), self.motion_params_actual.angular_velocity());
        println!("Ratio    -> lin vel:\t{:.2}\tang. vel:\t{:.2}",
                 self.motion_params_actual.linear_velocity() / self.motion_params_required.linear_velocity(),
                 self.motion_params_actual.angular_velocity() / self.motion_params_required.angular_velocity());

        println!();
        println!(" - Wheel Speed - ");
        println!();
        println!("Required -> l:\t{:.2}\tr:\t{:.2}", self.wheel_speed_required.left_wheel_speed(), self.wheel_speed_required.right_wheel_speed());
        println!("Actual   -> l:\t{:.2}\tr:\t{:.2}", self.wheel_speed_actual.left_wheel_speed(), self.wheel_speed_actual.right_wheel_speed());
        println!("Ratio    -> l:\t{:.2}\tr:\t{:.2}",
                 self.wheel_speed_actual.left_wheel_speed()/self.wheel_speed_required.left_wheel_speed(),
                 self.wheel_speed_actual.right_wheel_speed()/self.wheel_speed_required.right_wheel_speed())

    }

    pub fn set_pose(&mut self, pose: Pose) { self.pose = pose; }
    pub fn set_wheel_speed_required(&mut self, wheel_speed_required: WheelSpeeds) { self.wheel_speed_required = wheel_speed_required; }
    pub fn set_wheel_speed_actual(&mut self, wheel_speed_required: WheelSpeeds) { self.wheel_speed_actual = wheel_speed_required; }
    pub fn set_motion_params_required(&mut self, motion_params_required: MotionParameters) { self.motion_params_required = motion_params_required; }
    pub fn set_motion_params_actual(&mut self, motion_params_actual: MotionParameters) { self.motion_params_actual = motion_params_actual; }
}