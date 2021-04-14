use crate::robot::{RobotPhysicalConstrains, RobotStats};
use crate::sensor_model::SensorModel;

pub struct LineEstimator {
    robot_constrains: RobotPhysicalConstrains,
}

impl LineEstimator {

    pub fn new(robot_constrains: RobotPhysicalConstrains) -> LineEstimator {
        LineEstimator {
            robot_constrains,
        }
    }

    pub fn get_distance_from_line(&self, sensor_models: &Vec<SensorModel>, robot_stats: RobotStats) -> f32 {

        let left_sensor_val = sensor_models[1].normalize_measured_value(robot_stats.sensor_value_for_index(1));
        let right_sensor_val = sensor_models[0].normalize_measured_value(robot_stats.sensor_value_for_index(0));

        let left_sens_dist = left_sensor_val * self.robot_constrains.sensor_pose_for_index(1).y();
        let right_sens_dist = right_sensor_val * self.robot_constrains.sensor_pose_for_index(0).y();

        let dist = -(left_sens_dist + right_sens_dist);
        // let dist = sensor_models[0].normalize_measured_value(robot_stats.sensor_value_for_index(0)) - sensor_models[1].normalize_measured_value(robot_stats.sensor_value_for_index(1));
        println!("{}", dist);
        dist
    }
}