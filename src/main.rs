use std::net::UdpSocket;
use std::thread::sleep;
use std::time::Duration;

mod controller;
mod nmea;
mod message_factory;
mod robot;
mod motion_model;
mod motor_ramp_generator;
mod dashboard;
mod primitives;
mod line_estimator;
mod sensor_model;

fn main() {

    rosrust::init("BPC_PRP_CONTROLLER");

    let ip = String::from("127.0.0.1");
    let tx_port = String::from("8080");
    let rx_port = String::from("8081");

    let robot_constrains = robot::RobotPhysicalConstrains::new(
        2,
        vec![
            primitives::Pose::new(0.09, -0.01, 0.01),
            primitives::Pose::new(0.09, 0.01, 0.01)],
        0.16,
        0.04,
        4000.0,
        4000.0,
        200,
        32,
        2.0,
    );

    let mut controller = controller::Controller::new(ip, tx_port, rx_port, robot_constrains);
    controller.run();
}
