#[derive(Clone)]
pub struct SensorModel {
    max_val: u16,
    min_val: u16,
}

impl SensorModel {

    pub fn new() -> SensorModel {
        SensorModel {
            max_val: std::u16::MIN,
            min_val: std::u16::MAX-1,
        }
    }

    pub fn on_new_calibration_value(&mut self, value: u16) {
        self.max_val = self.max_val.max(value);
        self.min_val = self.min_val.min(value);
    }

    pub fn normalize_measured_value(&self, val: u16) -> f32 {
        if self.max_val <= self.min_val {
            return 0.0
        }

        let bias = self.min_val;
        let amplitude = (self.max_val - self.min_val);
        let amp_norm = amplitude as f32;

        let normalized = (val as i32 - bias as i32) as f32 / amp_norm;
        normalized.max(0.0).min(1.0)
    }

    pub fn get_max(&self) -> u16 { self.max_val }
    pub fn get_min(&self) -> u16 { self.min_val }
}