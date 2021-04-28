
pub struct PSDController {
    p: f32,
    s: f32,
    d: f32,
    last_val: f32,
    integrated_val: f32,
}

impl PSDController {

    pub fn new(p: f32, s: f32, d: f32) -> PSDController {
        PSDController {
            p, s, d, last_val: 0.0, integrated_val: 0.0,
        }
    }

    pub fn on_error_value(&mut self, err_val: f32, dt: f32) -> f32 {
        let pp = err_val * self.p;
        self.integrated_val += err_val * self.s * dt;
        let dd = (err_val - self.last_val) * self.d / dt;
        println!("err: {} p:{} s:{} d:{}", err_val, pp, self.integrated_val, dd);
        pp + self.integrated_val + dd
    }
}