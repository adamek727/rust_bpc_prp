use crate::nmea;

pub fn get_reset_message() -> String {
    nmea::encapsulate_string_to_nmea_message(&String::from("RESET"))
}

pub fn get_ping_message() -> String {
    nmea::encapsulate_string_to_nmea_message(&String::from("PING"))
}

pub fn get_left_odometry_request() -> String {
    nmea::encapsulate_string_to_nmea_message(&String::from("LODO"))
}

pub fn get_right_odometry_request() -> String {
    nmea::encapsulate_string_to_nmea_message(&String::from("RODO"))
}

pub fn get_left_wheel_speed_request(speed: f32) -> String {
    nmea::encapsulate_string_to_nmea_message(&String::from(format!("LSPEED,{}", speed)))
}

pub fn get_right_wheel_speed_request(speed: f32) -> String {
    nmea::encapsulate_string_to_nmea_message(&String::from(format!("RSPEED,{}", speed)))
}

pub fn get_sensor_value_request(sensor_index: u16) -> String {
    nmea::encapsulate_string_to_nmea_message(&String::from(format!("SENSOR,{}", sensor_index)))
}

