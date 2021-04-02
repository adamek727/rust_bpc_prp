use std::time::SystemTime;
use std::alloc::System;
use std::cmp::{max, min};

pub struct MotorRampGenerator {
    required_microsteps_per_sec: f32,
    actual_microsteps_per_sec: f32,
    last_evaluation_time: u128,
    max_acceleraton_in_microsteps: f32,
    max_speed_in_microsteps: f32,
}

impl MotorRampGenerator {

    pub fn new(max_acceleraton_in_microsteps: f32, max_speed_in_microsteps: f32) -> MotorRampGenerator {

        MotorRampGenerator{
            required_microsteps_per_sec: 0.0,
            actual_microsteps_per_sec: 0.0,
            last_evaluation_time: 0,
            max_acceleraton_in_microsteps,
            max_speed_in_microsteps,
        }
    }

    pub fn set_required_microsteps_per_sec(&mut self, microsteps: f32) {
        self.required_microsteps_per_sec = microsteps;
    }

    pub fn actual_microsteps_per_sec(&mut self) -> f32 {
        let time = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).expect("Unable to get system time").as_millis();
        if self.last_evaluation_time == 0 {
            self.last_evaluation_time = time;
            0.0
        } else {
            let dt = (time - self.last_evaluation_time) as f32 / 1000.0;
            self.last_evaluation_time = time;

            let delta_speed = (self.required_microsteps_per_sec - self.actual_microsteps_per_sec);
            let acceleration = delta_speed.min(self.max_acceleraton_in_microsteps * dt)
                                               .max(-self.max_acceleraton_in_microsteps * dt);

            self.actual_microsteps_per_sec = (self.actual_microsteps_per_sec + acceleration).min(self.max_speed_in_microsteps).max(-self.max_speed_in_microsteps);
            self.actual_microsteps_per_sec
        }
    }
}