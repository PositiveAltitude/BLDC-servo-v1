#[derive(Clone, Debug)]
pub struct PositionPid {
    p_gain: f32,
    i_gain: f32,
    d_gain: f32,
    integral: i64,
    integral_limit: i64,
    previous_error: Option<i32>,
}

impl PositionPid {
    pub const fn new(p_gain: f32, i_gain: f32, d_gain: f32, integral_limit: i64) -> Self {
        Self {
            p_gain,
            i_gain,
            d_gain,
            integral: 0,
            integral_limit: if integral_limit < 0 {
                0
            } else {
                integral_limit
            },
            previous_error: None,
        }
    }

    pub fn update(&mut self, error: i32) -> f32 {
        self.integral = self
            .integral
            .saturating_add(error as i64)
            .clamp(-self.integral_limit, self.integral_limit);

        let derivative = self
            .previous_error
            .map(|previous_error| error.saturating_sub(previous_error))
            .unwrap_or(0);
        self.previous_error = Some(error);

        (self.p_gain * error as f32
            + self.i_gain * self.integral as f32
            + self.d_gain * derivative as f32)
            .clamp(-1.0, 1.0)
    }

    pub fn reset(&mut self) {
        self.integral = 0;
        self.previous_error = None;
    }

    pub fn set_p_gain(&mut self, gain: f32) {
        self.p_gain = gain;
    }

    pub fn set_i_gain(&mut self, gain: f32) {
        self.i_gain = gain;
    }

    pub fn set_d_gain(&mut self, gain: f32) {
        self.d_gain = gain;
    }

    pub fn set_integral_limit(&mut self, limit: i64) {
        self.integral_limit = limit.max(0);
        self.integral = self
            .integral
            .clamp(-self.integral_limit, self.integral_limit);
    }
}

#[cfg(test)]
mod tests {
    use super::PositionPid;

    fn assert_close(actual: f32, expected: f32) {
        assert!((actual - expected).abs() < 1e-6, "{actual} != {expected}");
    }

    #[test]
    fn combines_proportional_integral_and_derivative_terms() {
        let mut pid = PositionPid::new(0.1, 0.01, 0.5, 1_000);

        assert_close(pid.update(4), 0.44);
        assert_close(pid.update(6), 1.0);
    }

    #[test]
    fn clamps_integral_and_output() {
        let mut pid = PositionPid::new(0.0, 0.1, 0.0, 5);

        assert_close(pid.update(4), 0.4);
        assert_close(pid.update(4), 0.5);
        assert_close(pid.update(-20), -0.5);
    }

    #[test]
    fn reset_clears_integral_and_derivative_history() {
        let mut pid = PositionPid::new(0.0, 0.1, 0.1, 100);
        pid.update(10);
        pid.reset();

        assert_close(pid.update(2), 0.2);
    }

    #[test]
    fn applies_new_gains_and_integral_limit() {
        let mut pid = PositionPid::new(0.0, 0.0, 0.0, 100);
        pid.update(10);
        pid.set_p_gain(0.1);
        pid.set_i_gain(0.1);
        pid.set_d_gain(0.1);
        pid.set_integral_limit(5);

        assert_close(pid.update(4), 0.3);
    }
}
