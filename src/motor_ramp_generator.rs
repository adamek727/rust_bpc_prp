use std::time::SystemTime;
use std::alloc::System;
use std::cmp::{max, min};

use crate::motion_model;
use crate::motion_model::WheelSpeeds;
use crate::motor_ramp_generator::RampGeneratiorSide::left;

pub enum RampGeneratiorSide {
    left,
    right,
}

pub struct MotorRampGenerator {
    required_microsteps_per_sec: f32,
    actual_microsteps_per_sec: f32,
    last_evaluation_time: u128,
    max_acceleraton_in_microsteps: f32,
    max_speed_in_microsteps: f32,
    ramp_ratio: f32,
    side: RampGeneratiorSide,
}

impl MotorRampGenerator {

    pub fn new(max_acceleraton_in_microsteps: f32, max_speed_in_microsteps: f32, side: RampGeneratiorSide) -> MotorRampGenerator {

        MotorRampGenerator{
            required_microsteps_per_sec: 0.0,
            actual_microsteps_per_sec: 0.0,
            last_evaluation_time: 0,
            max_acceleraton_in_microsteps,
            max_speed_in_microsteps,
            ramp_ratio: 0.0,
            side,
        }
    }

    pub fn set_required_microsteps_per_sec(&mut self, microsteps: &WheelSpeeds) {

        let faster_wheel_speed = microsteps.right_wheel_speed().abs().max(microsteps.left_wheel_speed().abs());
        let reduce_speed_ratio = (self.max_speed_in_microsteps / faster_wheel_speed).min(1.0);

        match &self.side {
            RampGeneratiorSide::left => {
                let left_to_right_ratio = (microsteps.left_wheel_speed() / microsteps.right_wheel_speed()).abs();
                if left_to_right_ratio < 1.0 {
                    self.ramp_ratio = left_to_right_ratio;
                } else {
                    self.ramp_ratio = 1.0;
                }
                self.required_microsteps_per_sec = microsteps.left_wheel_speed() * reduce_speed_ratio;
            }
            RampGeneratiorSide::right=> {
                self.required_microsteps_per_sec = microsteps.right_wheel_speed();
                let right_to_left_ratio = (microsteps.right_wheel_speed() / microsteps.left_wheel_speed()).abs();
                if right_to_left_ratio < 1.0 {
                    self.ramp_ratio = right_to_left_ratio;
                } else {
                    self.ramp_ratio = 1.0;
                }
                self.required_microsteps_per_sec = microsteps.right_wheel_speed() * reduce_speed_ratio;
            }
        }
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
                                               .max(-self.max_acceleraton_in_microsteps * dt)  * self.ramp_ratio;

            self.actual_microsteps_per_sec = (self.actual_microsteps_per_sec + acceleration).min(self.max_speed_in_microsteps).max(-self.max_speed_in_microsteps);

            // println!("{} {:.2}, {:.2}, {:.2}", dt, delta_speed, acceleration, self.actual_microsteps_per_sec);
            self.actual_microsteps_per_sec
        }
    }
}