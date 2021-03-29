mod nmea;

use std::net::UdpSocket;
use std::thread::sleep;
use std::time::Duration;
use adash_bpc_prp::Controller;



fn main() {
    let ip = String::from("127.0.0.1");
    let tx_port = String::from("8080");
    let rx_port = String::from("8081");

    let controller = Controller::new(ip, tx_port, rx_port);
    controller.control_loop();
}
