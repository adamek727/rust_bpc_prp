use std::time::Duration;
use std::net::UdpSocket;
use std::thread;
use std::thread::sleep;
use std::sync::mpsc::{channel, Sender, Receiver};
extern crate timer;
extern crate chrono;

mod nmea;
mod message_factory;

pub struct Controller {
    remote_ip: String,
    tx_port: String,
    rx_port: String,
    robot_structure: RobotStructure,
    is_running: bool,
}

struct RobotStructure {
    no_of_sensors: u16,
    sensors_val: Vec<u16>,
}

impl Controller {

    pub fn new(ip: String, tx_prt: String, rx_prt: String, no_of_sens: u16) -> Controller{
        Controller{
            remote_ip: ip,
            tx_port: tx_prt,
            rx_port: rx_prt,
            robot_structure: RobotStructure {
                no_of_sensors: no_of_sens,
                sensors_val: vec![0; no_of_sens as usize],
            },
            is_running: true,
        }
    }

    pub fn control_loop(self) {

        let (tx_data_from_simulator, rx_data_from_simulator) = channel();
        let (tx_data_to_simulator, rx_data_to_simulator) = channel();

        self.spawn_simulator_data_transmitter(rx_data_to_simulator);
        self.spawn_simulator_data_receiver(tx_data_from_simulator.clone());

        self.reset_simulation(&tx_data_to_simulator);

        let sensor_value_request_channel = tx_data_to_simulator.clone();
        let sensor_value_request_timer = timer::Timer::new();
        let guard = sensor_value_request_timer.schedule_repeating(chrono::Duration::milliseconds(10), move || {
            sensor_value_request_channel.send(message_factory::get_sensor_value_request(0));
        });


        let set_wheel_speed_channel = tx_data_to_simulator.clone();
        let set_wheel_speed_timer = timer::Timer::new();
        let guard = set_wheel_speed_timer.schedule_repeating(chrono::Duration::milliseconds(100), move || {
            set_wheel_speed_channel.send(message_factory::get_left_wheel_speed_request(100.0));
            set_wheel_speed_channel.send(message_factory::get_right_wheel_speed_request(100.0));
        });

        loop {
            loop {
                match rx_data_from_simulator.recv_timeout(Duration::from_millis(0)) {
                    Ok(response) => {
                        match response {
                            nmea::SimulatorResponseType::Reset { ok } => { println!("Reset: {}", ok) }
                            nmea::SimulatorResponseType::PingPong => { println!("PingPong") }
                            nmea::SimulatorResponseType::LeftOdometry { microsteps } => { println!("Left Odometry: {}", microsteps) }
                            nmea::SimulatorResponseType::RightOdometry { microsteps } => { println!("Right Odometry: {}", microsteps) }
                            nmea::SimulatorResponseType::Ok => { println!("Ok") }
                            nmea::SimulatorResponseType::Sensor { index, value } => { println!("Sensor {} value: {}", index, value) }
                            nmea::SimulatorResponseType::Invalid => { println! {"Invalid response!"} }
                        }
                    }
                    _ => { break; }
                }
            }
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

    fn reset_simulation(&self, tx_channel: &Sender<String>) {
        tx_channel.send(message_factory::get_reset_message());
        sleep(Duration::from_millis(1000));
    }
}