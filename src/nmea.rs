
pub enum SimulatorResponseType {
    Reset{ok: bool},
    PingPong,
    LeftOdometry{microsteps: i32},
    RightOdometry{microsteps: i32},
    Ok,
    Sensor{index: usize, value: u16},
    Invalid,
}


pub fn encapsulate_string_to_nmea_message(str: &str) -> String {

    let checksum = get_message_checksum(str);
    format!("{}{}{}{}", "$", str, "*", checksum)
}

pub fn parse_nmea_message(msg: &str) -> SimulatorResponseType {
    let splited_msg = split_nmea_message(msg);

    if splited_msg.len() >= 1 {
        match splited_msg[0].as_str() {
            "PONG" => {
                return SimulatorResponseType::PingPong
            },
            "ok" => {
                return SimulatorResponseType::Ok
            },
            _ => {},
        }
    }
    if splited_msg.len() >= 2 {
        match splited_msg[0].as_str() {
            "RESET" => {
                return SimulatorResponseType::Reset {
                    ok: splited_msg[1].as_str() == "DONE"
                }
            },
            "LODO" => {
                return SimulatorResponseType::LeftOdometry {
                    microsteps: splited_msg[1].trim().parse::<i32>().unwrap(),
                }
            }
            "RODO" => {
                return SimulatorResponseType::LeftOdometry {
                    microsteps: splited_msg[1].trim().parse::<i32>().unwrap(),
                }
            }
            _ => {},
        }

    }
    if splited_msg.len() >= 3 {
        match splited_msg[0].as_str() {
            "SENSOR" => {
                return SimulatorResponseType::Sensor {
                    index: splited_msg[1].trim().parse::<usize>().unwrap(),
                    value: splited_msg[2].trim().parse::<u16>().unwrap(),
                }
            },
            _ => {},
        }
    }
    SimulatorResponseType::Invalid
}

fn split_nmea_message(str: &str) -> Vec<String> {
    let output: Vec<String> = vec![];

    if str.len() <= 0 {
        return output
    }

    if str.chars().nth(0).unwrap() != '$' {
        return output
    }

    let res: Vec<String> = str.clone().replace("$", "").split("*").map(|s| s.to_string()).collect();
    if res.len() != 2 {
        return output
    }

    let core_message = res.get(0).unwrap();
    let checksum = String::from(res.get(1).unwrap().to_uppercase().replace('\0', ""));

    let check = get_message_checksum(core_message);
    if check != checksum {
        return output
    }

    core_message.split(",").map(|s| s.to_string()).collect()
}

fn get_message_checksum(str: &str) -> String {

    let mut checksum = 0 as u8;
    for char in str.as_bytes() {
        checksum ^= char;
    }
    format!("{:02X}", checksum)
}