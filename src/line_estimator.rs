use crate::robot::{RobotPhysicalConstrains, RobotStats};
use crate::sensor_model::SensorModel;

pub struct LineEstimator {
    robot_constrains: RobotPhysicalConstrains,
}

impl LineEstimator {

    const RIGHT_SENSOR_INDEX: usize = 0;
    const MIDDLE_SENSOR_INDEX: usize = 1;
    const LEFT_SENSOR_INDEX: usize = 2;
    const FORWARD_SENSOR_INDEX: usize = 3;

    pub fn new(robot_constrains: RobotPhysicalConstrains) -> LineEstimator {
        LineEstimator {
            robot_constrains,
        }
    }

    pub fn get_distance_from_line(&self, sensor_models: &Vec<SensorModel>, robot_stats: RobotStats) -> f32 {

        let left_sensor_val = sensor_models[LineEstimator::LEFT_SENSOR_INDEX].normalize_measured_value(robot_stats.sensor_value_for_index(LineEstimator::LEFT_SENSOR_INDEX));
        let right_sensor_val = sensor_models[LineEstimator::RIGHT_SENSOR_INDEX].normalize_measured_value(robot_stats.sensor_value_for_index(LineEstimator::RIGHT_SENSOR_INDEX));
        let middle_sensor_val = sensor_models[LineEstimator::MIDDLE_SENSOR_INDEX].normalize_measured_value(robot_stats.sensor_value_for_index(LineEstimator::MIDDLE_SENSOR_INDEX));

        let left_sens_dist = left_sensor_val * self.robot_constrains.sensor_pose_for_index(LineEstimator::LEFT_SENSOR_INDEX).y();
        let right_sens_dist = right_sensor_val * self.robot_constrains.sensor_pose_for_index(LineEstimator::RIGHT_SENSOR_INDEX).y();

        let dist = -(left_sens_dist + right_sens_dist);
        //println!("{}", dist);
        dist
    }

    pub fn forward_sensor_val(&self, sensor_models: &Vec<SensorModel>, robot_stats: RobotStats) -> f32 {

        let mid_sensor_val = sensor_models[LineEstimator::FORWARD_SENSOR_INDEX].normalize_measured_value(robot_stats.sensor_value_for_index(LineEstimator::FORWARD_SENSOR_INDEX));
        mid_sensor_val
    }
}