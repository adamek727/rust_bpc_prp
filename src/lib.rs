use std::time::Duration;
use std::net::UdpSocket;
use std::thread;
use std::thread::sleep;
use std::sync::mpsc::channel;

mod nmea;
mod message_factory;

pub struct Controller {
    remote_ip: String,
    tx_port: String,
    rx_port: String,
}

impl Controller {

    pub fn new(ip: String, tx_prt: String, rx_prt: String) -> Controller{
        Controller{
            remote_ip: ip,
            tx_port: tx_prt,
            rx_port: rx_prt,
        }
    }

    pub fn control_loop(self) {

        let rx_socket = UdpSocket::bind(format!("127.0.0.1:{}", self.rx_port)).expect("Unable to open rx socket");
        let tx_socket = UdpSocket::bind("0.0.0.0:0").expect("Unable to open tx socket");
        let (tx, rx) = channel();

        thread::spawn(move || {
            loop {
                let mut rx_buf:Vec<u8> = vec![0; 1024];;
                let result = rx_socket.recv(&mut rx_buf).expect("Error when receiving datagram");

                if result > 0 {
                    let incoming_msg = String::from_utf8(rx_buf).expect("Unable to convert datagram to string");
                    //println!("Received: {}", incoming_msg);
                    let parsed = nmea::parse_nmea_message(&incoming_msg);
                    tx.send(parsed);
                }
            }
        });

        self.send_udp_message(&message_factory::get_reset_message(), &tx_socket);
        sleep(Duration::from_millis(1000));




        loop {
            loop {
                match rx.recv_timeout(Duration::from_millis(0)) {
                    Ok(response) => {
                        match response {
                            nmea::SimulatorResponseType::Reset { ok } => { println!("Reset: {}", ok) }
                            nmea::SimulatorResponseType::PingPong => { println!("PingPong") }
                            nmea::SimulatorResponseType::LeftOdometry { microsteps } => { println!("Left Odometry: {}", microsteps) }
                            nmea::SimulatorResponseType::RightOdometry { microsteps } => { println!("Right Odometry: {}", microsteps) }
                            nmea::SimulatorResponseType::Ok => { println!("Ok") }
                            nmea::SimulatorResponseType::Sensor{index, value} =>  {println!("Sensor {} value: {}", index, value)}
                            nmea::SimulatorResponseType::Sensor { index, value } => { println!("{}", value) }
                            nmea::SimulatorResponseType::Invalid => { println! {"Invalid response!"} }
                        }
                    }
                    _ => {break;}
                }
            }
            
            self.send_udp_message(&message_factory::get_left_wheel_speed_request(100.0), &tx_socket);
            self.send_udp_message(&message_factory::get_right_wheel_speed_request(100.0), &tx_socket);
            self.send_udp_message(&message_factory::get_sensor_value_request(0), &tx_socket);
            sleep(Duration::from_millis(100));
        }
    }


    fn send_udp_message(&self, msg: &str, socket: &UdpSocket) -> std::io::Result<()>{

        let datagram: Vec<u8> = msg.as_bytes().to_vec();
        match socket.send_to(&datagram, format!("{}:{}", self.remote_ip, self.tx_port)) {
            Ok(n) => {
                //println!("Send: {}", msg);
                Ok(())
            },
            Err(e) => {
                println!("Error {}", e);
                Err(e)
            }
        }
    }
}