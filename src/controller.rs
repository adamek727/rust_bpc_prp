use std::time::{Duration, SystemTime};
use std::net::UdpSocket;
use std::thread;
use std::thread::sleep;
use std::sync::mpsc::{channel, Sender, Receiver};
use timer;
use chrono;

use crate::nmea;
use crate::message_factory;
use crate::robot;
use crate::robot::{RobotStateMachine};
use crate::primitives::Pose;
use crate::motion_model::{MotionModel, WheelSpeeds, MotionParameters};
use crate::motor_ramp_generator;
use crate::motor_ramp_generator::{MotorRampGenerator, RampGeneratiorSide};
use crate::dashboard::Dashboard;
use termion::event::MouseButton::WheelDown;
use crate::nmea::SimulatorResponseType;
use crate::line_estimator::LineEstimator;
use crate::sensor_model::SensorModel;

pub struct Controller {
    sensor_models: Vec<SensorModel>,
    remote_ip: String,
    tx_port: String,
    rx_port: String,
    robot_stats: robot::RobotStats,
    robot_constrains: robot::RobotPhysicalConstrains,
    is_running: bool,
}


impl Controller {

    pub fn new(ip: String, tx_prt: String, rx_prt: String, robot_constrains: robot::RobotPhysicalConstrains) -> Controller{
        Controller{
            sensor_models: std::iter::repeat_with(|| SensorModel::new())
                .take(robot_constrains.no_of_sensors())
                .collect::<Vec<_>>(),
            remote_ip: ip,
            tx_port: tx_prt,
            rx_port: rx_prt,
            robot_stats: robot::RobotStats::new (robot_constrains.no_of_sensors()),
            robot_constrains,
            is_running: true,
        }
    }

    pub fn run(&mut self) {

        // self.robot_stats.set_state(RobotStateMachine::Custom);

        let mut dashboard = Dashboard::new(self.robot_constrains.no_of_sensors().clone());
        let mut motion_model = MotionModel::new(
            self.robot_constrains.clone(),
            Pose::new(0.0, 0.0, 0.0),
            0.0,
        );
        let mut left_wheel_ramp_gen = MotorRampGenerator::new(
            self.robot_constrains.max_acceleration_microsteps_per_sec(),
            self.robot_constrains.max_microsteps_per_sec(),
            RampGeneratiorSide::left,
        );
        let mut right_wheel_ramp_gen = MotorRampGenerator::new(
            self.robot_constrains.max_acceleration_microsteps_per_sec(),
            self.robot_constrains.max_microsteps_per_sec(),
            RampGeneratiorSide::right,
        );

        let line_estimator = LineEstimator::new(self.robot_constrains.clone());

        let (tx_data_from_simulator, rx_data_from_simulator) = channel();
        let (tx_data_to_simulator, rx_data_to_simulator) = channel();

        self.spawn_simulator_data_transmitter(rx_data_to_simulator);
        self.spawn_simulator_data_receiver(tx_data_from_simulator.clone());


        self.reset_simulation(&tx_data_to_simulator);
        self.spawn_sensor_values_requesting(tx_data_to_simulator.clone());
        sleep(Duration::from_millis(100));


        let mut time = Controller::get_time();
        loop { // Main Loop

            self.read_incoming_messages(&rx_data_from_simulator);

            let mut required_motion = MotionParameters::new(0.0, 0.0);
            let calibration_rot_speed = 0.2;
            let calibration_rot_range = 3.14 / 8.0;
            match self.robot_stats.state() {
                RobotStateMachine::Initialization => {
                    self.robot_stats.set_state(RobotStateMachine::CalibrationRotateLeft);
                },
                RobotStateMachine::CalibrationRotateLeft => {
                    required_motion = MotionParameters::new(0.0, calibration_rot_speed);
                    if motion_model.get_orientation() > calibration_rot_range {
                        self.robot_stats.set_state(RobotStateMachine::CalibrationRotateRight);
                    }
                    for ind in 0..self.robot_stats.number_of_sensors() {
                        let sensor_val = self.robot_stats.sensor_value_for_index(ind);
                        self.sensor_models[ind].on_new_calibration_value(sensor_val);
                    }
                },
                RobotStateMachine::CalibrationRotateRight => {
                    required_motion = MotionParameters::new(0.0, -calibration_rot_speed);
                    if motion_model.get_orientation() < -calibration_rot_range {
                        self.robot_stats.set_state(RobotStateMachine::CalibrationCenter);
                    }
                    for ind in 0..self.robot_stats.number_of_sensors() {
                        let sensor_val = self.robot_stats.sensor_value_for_index(ind);
                        self.sensor_models[ind].on_new_calibration_value(sensor_val);
                    }
                },
                RobotStateMachine::CalibrationCenter => {
                    required_motion = MotionParameters::new(0.0, calibration_rot_speed/4.0);
                    if motion_model.get_orientation() > 0.0 {
                        self.robot_stats.set_state(RobotStateMachine::LineFollowing);
                    }
                    for ind in 0..self.robot_stats.number_of_sensors() {
                        let sensor_val = self.robot_stats.sensor_value_for_index(ind);
                        self.sensor_models[ind].on_new_calibration_value(sensor_val);
                    }
                },
                RobotStateMachine::LineFollowing => {

                    let distance_from_line = line_estimator.get_distance_from_line(&self.sensor_models, self.robot_stats.clone());
                    dashboard.set_dist_from_line(distance_from_line);
                    required_motion = MotionParameters::new(0.2, -distance_from_line*100.0);

                },
                RobotStateMachine::Fail => {
                    required_motion = MotionParameters::new(0.0, 0.0);
                },
                RobotStateMachine::Custom => {
                    required_motion = MotionParameters::new(0.01, 0.0);
                },
            }



            let wheel_speeds = motion_model.get_wheel_speeds( required_motion );
            let wheel_speeds_in_microsteps = motion_model.wheel_speed_to_microsteps_with_saturation(wheel_speeds);
            left_wheel_ramp_gen.set_required_microsteps_per_sec(&wheel_speeds_in_microsteps);
            right_wheel_ramp_gen.set_required_microsteps_per_sec(&wheel_speeds_in_microsteps);

            let actual_wheel_speed_in_microsteps = WheelSpeeds::new(
                left_wheel_ramp_gen.actual_microsteps_per_sec(),
                right_wheel_ramp_gen.actual_microsteps_per_sec(),
            );
            let actual_motion_params = motion_model.wheel_speed_to_robot_motion_parameters(
                motion_model.microsteps_to_wheel_speed(
                    actual_wheel_speed_in_microsteps
                )
            );

            let dt = (Controller::get_time() - time) as f32 / 1000.0;
            motion_model.integrate_robot_motion(actual_motion_params, dt);

            tx_data_to_simulator.send(message_factory::get_left_wheel_speed_request(actual_wheel_speed_in_microsteps.left_wheel_speed()));
            tx_data_to_simulator.send(message_factory::get_right_wheel_speed_request(actual_wheel_speed_in_microsteps.right_wheel_speed()));

            dashboard.set_pose(motion_model.get_pose());
            dashboard.set_orientation(motion_model.get_orientation());
            dashboard.set_wheel_speed_required(wheel_speeds_in_microsteps);
            dashboard.set_wheel_speed_actual(actual_wheel_speed_in_microsteps);
            dashboard.set_motion_params_required(required_motion);
            dashboard.set_motion_params_actual(actual_motion_params);
            dashboard.set_sensor_values(self.robot_stats.sensor_values());
            dashboard.render();

            time = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).expect("Unable to get system time").as_millis();
            sleep(Duration::from_millis(8));
        }
    }

    fn read_incoming_messages(&mut self, receiver_from_simulator: &Receiver<SimulatorResponseType>) {
        loop {  // Readout all new messages

            match receiver_from_simulator.recv_timeout(Duration::from_millis(0)) {
                Ok(response) => {
                    match response {
                        nmea::SimulatorResponseType::Reset { ok } => {
                            println!("Reset: {}", ok)
                        }
                        nmea::SimulatorResponseType::PingPong => {
                            println!("PingPong")
                        }
                        nmea::SimulatorResponseType::LeftOdometry { microsteps } => {
                            // println!("Left Odometry: {}", microsteps)
                        }
                        nmea::SimulatorResponseType::RightOdometry { microsteps } => {
                            // println!("Right Odometry: {}", microsteps)
                        }
                        nmea::SimulatorResponseType::Ok => {
                            // println!("Ok")
                        }
                        nmea::SimulatorResponseType::Sensor { index, value } => {
                            // println!("Sensor {} value: {}", index, value);
                            self.robot_stats.set_value_for_index(index, value);
                        }
                        nmea::SimulatorResponseType::Invalid => {
                            // println!("Invalid response!");
                            self.robot_stats.set_state(RobotStateMachine::Fail);
                        }
                    }
                }
                _ => { break; }
            }
        }
    }

    fn request_data_from_simulator(&self, channel_tx: Receiver<String>) {


    }


    fn spawn_simulator_data_transmitter(&self , channel_rx: Receiver<String>) {

        let remote_ip = self.remote_ip.clone();
        let port = self.tx_port.clone();
        thread::spawn(move || {
            let tx_socket = UdpSocket::bind("0.0.0.0:0").expect("Unable to open tx socket");
            loop {
                match channel_rx.recv_timeout(Duration::from_millis(0)) {
                    Ok(msg) => {
                        let datagram: Vec<u8> = msg.as_bytes().to_vec();
                        // println!{"Sending: {}", msg};
                        tx_socket.send_to(&datagram, format!("{}:{}", remote_ip, port)).expect("Error when sending message to simulator");
                    }
                    _ => {}
                }
                sleep(Duration::from_millis(1));
            }
        });
    }

    fn spawn_simulator_data_receiver(&self, channel_tx: Sender<nmea::SimulatorResponseType>) {

        let copy_rx_port = self.rx_port.clone();
        thread::spawn(move || {
            let rx_socket = UdpSocket::bind(format!("127.0.0.1:{}", copy_rx_port)).expect("Unable to open rx socket");
            loop {
                let mut rx_buf:Vec<u8> = vec![0; 1024];
                let result = rx_socket.recv(&mut rx_buf).expect("Error when receiving datagram");
                if result > 0 {
                    let incoming_msg = String::from_utf8(rx_buf).expect("Unable to convert datagram to string");
                    let parsed = nmea::parse_nmea_message(&incoming_msg);
                    channel_tx.send(parsed);
                }
                sleep(Duration::from_millis(1));
            }
        });
    }

    fn spawn_sensor_values_requesting(&self, sensor_value_request_channel: Sender<String>) {
        let no_of_sensors = self.robot_constrains.no_of_sensors();
        thread::spawn(move || {
            loop {
                for i in 0..no_of_sensors {
                    sensor_value_request_channel.send(message_factory::get_sensor_value_request(i as u16));
                }
                sleep(Duration::from_millis(10));
            }
        });
    }

    fn reset_simulation(&self, tx_channel: &Sender<String>) {
        tx_channel.send(message_factory::get_reset_message());
        sleep(Duration::from_millis(1000));
    }

    fn get_time() -> u128 {
        SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).expect("Unable to get system time").as_millis()
    }
}