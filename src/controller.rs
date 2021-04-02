use std::time::Duration;
use std::net::UdpSocket;
use std::thread;
use std::thread::sleep;
use std::sync::mpsc::{channel, Sender, Receiver};
use timer;
use chrono;

use crate::nmea;
use crate::message_factory;
use crate::robot;
use crate::robot::RobotStateMachine;
use crate::motion_model::MotionModel;
use crate::motor_ramp_generator;
use crate::motor_ramp_generator::MotorRampGenerator;

pub struct Controller {
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
            remote_ip: ip,
            tx_port: tx_prt,
            rx_port: rx_prt,
            robot_stats: robot::RobotStats::new (robot_constrains.no_of_sensors()),
            robot_constrains,
            is_running: true,
        }
    }

    pub fn run(&mut self) {

        let motion_model = MotionModel::new(self.robot_constrains.chassis_base(), self.robot_constrains.wheel_radius());
        let mut left_wheel_ramp_gen = MotorRampGenerator::new(
            self.robot_constrains.max_acceleration_microsteps_per_sec(),
                self.robot_constrains.max_microsteps_per_sec()
        );
        let mut right_wheel_ramp_gen = MotorRampGenerator::new(
            self.robot_constrains.max_acceleration_microsteps_per_sec(),
            self.robot_constrains.max_microsteps_per_sec()
        );

        let (tx_data_from_simulator, rx_data_from_simulator) = channel();
        let (tx_data_to_simulator, rx_data_to_simulator) = channel();

        self.spawn_simulator_data_transmitter(rx_data_to_simulator);
        self.spawn_simulator_data_receiver(tx_data_from_simulator.clone());

        self.reset_simulation(&tx_data_to_simulator);

        let sensor_value_request_channel = tx_data_to_simulator.clone();
        let sensor_value_request_timer = timer::Timer::new();
        let no_of_sensors = self.robot_constrains.no_of_sensors();
        let guard = sensor_value_request_timer.schedule_repeating(chrono::Duration::milliseconds(10), move || {
            for i in 0..no_of_sensors {
                sensor_value_request_channel.send(message_factory::get_sensor_value_request(i as u16));
            }
        });


        // let set_wheel_speed_channel = tx_data_to_simulator.clone();
        // let set_wheel_speed_timer = timer::Timer::new();
        // let guard = set_wheel_speed_timer.schedule_repeating(chrono::Duration::milliseconds(100), move || {
        //     set_wheel_speed_channel.send(message_factory::get_left_wheel_speed_request(self.robot_stats.left_wheel_microsteps_per_sec() as f32));
        //     set_wheel_speed_channel.send(message_factory::get_right_wheel_speed_request(100.0));
        // });

        loop { // Main Loop

            loop {  // Readout all new messages

                match rx_data_from_simulator.recv_timeout(Duration::from_millis(0)) {
                    Ok(response) => {
                        match response {
                            nmea::SimulatorResponseType::Reset { ok } => { println!("Reset: {}", ok) }
                            nmea::SimulatorResponseType::PingPong => { println!("PingPong") }
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
                                // println!("Sensor {} value: {}", index, value) ;
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

            let wheel_speeds = motion_model.get_wheel_speeds(0.0, 10.5);
            left_wheel_ramp_gen.set_required_microsteps_per_sec(motion_model.wheell_speed_to_microsteps(wheel_speeds.left_wheel_speed()));
            right_wheel_ramp_gen.set_required_microsteps_per_sec( motion_model.wheell_speed_to_microsteps(wheel_speeds.right_wheel_speed()));

            tx_data_to_simulator.send(message_factory::get_left_wheel_speed_request(left_wheel_ramp_gen.actual_microsteps_per_sec()));
            tx_data_to_simulator.send(message_factory::get_right_wheel_speed_request(right_wheel_ramp_gen.actual_microsteps_per_sec()));
            sleep(Duration::from_millis(10));
        }
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
                    // println!("{}", incoming_msg);
                    channel_tx.send(parsed);
                }
                sleep(Duration::from_millis(1));
            }
        });
    }

    fn reset_simulation(&self, tx_channel: &Sender<String>) {
        tx_channel.send(message_factory::get_reset_message());
        sleep(Duration::from_millis(1000));
    }
}