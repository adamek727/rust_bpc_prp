use termion;
use crate::primitives::Pose;
use crate::motion_model::{WheelSpeeds, MotionParameters};
use crate::sensor_model::SensorModel;

pub struct Dashboard {
    pose: Pose,
    robot_orientation: f32,
    wheel_speed_required: WheelSpeeds,
    wheel_speed_actual: WheelSpeeds,
    motion_params_required: MotionParameters,
    motion_params_actual: MotionParameters,
    sensor_values: Vec<u16>,
    dist_from_line: f32,

    ros_publisher_wheel_speed_left: rosrust::Publisher<rosrust_msg::std_msgs::Float32>,
    ros_publisher_wheel_speed_right: rosrust::Publisher<rosrust_msg::std_msgs::Float32>,
    ros_publisher_required_wheel_speed_left: rosrust::Publisher<rosrust_msg::std_msgs::Float32>,
    ros_publisher_required_wheel_speed_right: rosrust::Publisher<rosrust_msg::std_msgs::Float32>,

    num_of_sensors: usize,
    ros_publishers_sensor_values: Vec<rosrust::Publisher<rosrust_msg::std_msgs::UInt16>>,

    ros_publishers_distance_from_line: rosrust::Publisher<rosrust_msg::std_msgs::Float32>,
}

impl Dashboard {

    pub fn new(num_of_sensors: usize) -> Dashboard {

        let left_wheel_speed_topic: String = String::from("/bpc_prp/robot/left_wheel_speed");
        let right_wheel_speed_topic: String = String::from("/bpc_prp/robot/right_wheel_speed");
        let left_wheel_required_speed_topic: String = String::from("/bpc_prp/robot/required_left_wheel_speed");
        let right_wheel_required_speed_topic: String = String::from("/bpc_prp/robot/required_right_wheel_speed");

        let mut sensor_value_publishers: Vec<rosrust::Publisher<rosrust_msg::std_msgs::UInt16>> = vec![];
        for i in 0..num_of_sensors {
            let topic = String::from(format!("/bpc_prp/robot/sensor_{}_value", i));
            sensor_value_publishers.push(rosrust::publish(&topic, 0).unwrap());
        }

        let distance_from_line_topic: String = String::from("/bpc_prp/robot/distance_from_line");

        Dashboard {
            pose: Pose::new(0.0, 0.0, 0.0),
            robot_orientation: 0.0,
            wheel_speed_required: WheelSpeeds::new(0.0, 0.0),
            wheel_speed_actual: WheelSpeeds::new(0.0, 0.0),
            motion_params_required: MotionParameters::new(0.0, 0.0),
            motion_params_actual: MotionParameters::new(0.0, 0.0),
            sensor_values: vec![0; num_of_sensors],
            dist_from_line: 0.0,

            ros_publisher_wheel_speed_left: rosrust::publish(&left_wheel_speed_topic, 0).unwrap(),
            ros_publisher_wheel_speed_right: rosrust::publish(&right_wheel_speed_topic, 0).unwrap(),
            ros_publisher_required_wheel_speed_left: rosrust::publish(&left_wheel_required_speed_topic, 0).unwrap(),
            ros_publisher_required_wheel_speed_right: rosrust::publish(&right_wheel_required_speed_topic, 0).unwrap(),

            num_of_sensors,
            ros_publishers_sensor_values: sensor_value_publishers,

            ros_publishers_distance_from_line: rosrust::publish(&distance_from_line_topic, 0).unwrap(),
        }
    }

    pub fn render(&self) {
        //self.handle_terminal_printout();
        self.handle_ros_publishers();
    }

    pub fn set_pose(&mut self, pose: Pose) { self.pose = pose; }
    pub fn set_orientation(&mut self, orientation: f32) { self.robot_orientation = orientation; }
    pub fn set_wheel_speed_required(&mut self, wheel_speed_required: WheelSpeeds) { self.wheel_speed_required = wheel_speed_required; }
    pub fn set_wheel_speed_actual(&mut self, wheel_speed_required: WheelSpeeds) { self.wheel_speed_actual = wheel_speed_required; }
    pub fn set_motion_params_required(&mut self, motion_params_required: MotionParameters) { self.motion_params_required = motion_params_required; }
    pub fn set_motion_params_actual(&mut self, motion_params_actual: MotionParameters) { self.motion_params_actual = motion_params_actual; }
    pub fn set_sensor_values(&mut self, sensor_values: Vec<u16>) { self.sensor_values = sensor_values; }
    pub fn set_dist_from_line(&mut self, dist_from_line: f32) { self.dist_from_line = dist_from_line; }

    fn handle_terminal_printout(&self) {
        print!("{}{}", termion::clear::All, termion::cursor::Goto(1, 1));
        println!("{}", str::repeat("* ", 20));

        println!(" - Robot's Position - ");
        println!();
        println!("Pose -> x:\t{}\ty:\t{}", self.pose.x(), self.pose.y());
        println!("Orient -> {}", self.robot_orientation);

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

    fn handle_ros_publishers(&self) {
        let mut left_wheel_speed_msg = rosrust_msg::std_msgs::Float32::default();
        left_wheel_speed_msg.data = self.wheel_speed_actual.left_wheel_speed();
        self.ros_publisher_wheel_speed_left.send(left_wheel_speed_msg).unwrap();

        let mut right_wheel_speed_msg = rosrust_msg::std_msgs::Float32::default();
        right_wheel_speed_msg.data = self.wheel_speed_actual.right_wheel_speed();
        self.ros_publisher_wheel_speed_right.send(right_wheel_speed_msg).unwrap();

        let mut required_left_wheel_speed_msg = rosrust_msg::std_msgs::Float32::default();
        required_left_wheel_speed_msg.data = self.wheel_speed_required.left_wheel_speed();
        self.ros_publisher_required_wheel_speed_left.send(required_left_wheel_speed_msg).unwrap();

        let mut required_right_wheel_speed_msg = rosrust_msg::std_msgs::Float32::default();
        required_right_wheel_speed_msg.data = self.wheel_speed_required.right_wheel_speed();
        self.ros_publisher_required_wheel_speed_right.send(required_right_wheel_speed_msg).unwrap();

        for i in 0..self.num_of_sensors {
            let mut sensor_value_msg = rosrust_msg::std_msgs::UInt16::default();
            sensor_value_msg.data = self.sensor_values[i];
            self.ros_publishers_sensor_values[i].send(sensor_value_msg);
        }

        let mut dist_from_line_msg = rosrust_msg::std_msgs::Float32::default();
        dist_from_line_msg.data = self.dist_from_line;
        self.ros_publishers_distance_from_line.send(dist_from_line_msg).unwrap();
    }
}