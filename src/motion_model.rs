pub struct WheelSpeeds {
    left_wheel_speed: f32,
    right_wheel_speed: f32,
}

impl WheelSpeeds {
    pub fn left_wheel_speed(&self) -> f32 { self.left_wheel_speed }
    pub fn right_wheel_speed(&self) -> f32 { self.right_wheel_speed }
}


pub struct MotionModel {
    chassis_base: f32,
    wheel_radius: f32,
    wheel_perimeter: f32
}

impl MotionModel {

    pub fn new(chassis_base: f32, wheel_radius: f32,) -> MotionModel {
        MotionModel {
            chassis_base,
            wheel_radius,
            wheel_perimeter: 2.0 * std::f32::consts::PI * wheel_radius
        }
    }

    pub fn get_wheel_speeds(&self, required_velocity: f32, required_angular_speed: f32) -> WheelSpeeds {

        WheelSpeeds {
            left_wheel_speed: (2.0 * required_velocity - required_angular_speed * self.chassis_base) / (2.0 * self.wheel_radius),
            right_wheel_speed: (2.0 * required_velocity + required_angular_speed * self.chassis_base) / (2.0 * self.wheel_radius),
        }
    }

    pub fn wheell_speed_to_microsteps(&self, wheel_ang_speed: f32) -> f32 {

        let steps_per_rotation = 200;
        let microsteps_per_step = 32;
        let microsteps_per_rotation: f32 = (steps_per_rotation * microsteps_per_step) as f32;

        wheel_ang_speed / (2.0*std::f32::consts::PI) * microsteps_per_rotation
    }
}

