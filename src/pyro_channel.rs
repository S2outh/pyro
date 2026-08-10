use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer};

pub struct PyroChannel {
    safe: Output<'static>,
    fire: Output<'static>,
    fired: bool,
}

const FIRE_DURATION: Duration = Duration::from_millis(400);

impl PyroChannel {
    pub fn new(safe: Output<'static>, fire: Output<'static>) -> Self {
        Self {
            safe,
            fire,
            fired: false,
        }
    }
    pub fn is_armed(&self) -> bool {
        self.safe.is_set_high() && !self.is_fired()
    }
    pub fn is_fired(&self) -> bool {
        self.fired
    }
    pub fn arm(&mut self) {
        if self.is_fired() { return }
        self.safe.set_high();
    }
    pub fn disarm(&mut self) {
        if self.is_fired() { return }
        self.safe.set_low();
    }
    pub async fn fire(&mut self) {
        if self.is_fired() { return }
        self.fire.set_high();
        Timer::after(FIRE_DURATION).await;
        self.fire.set_low();
        self.safe.set_low();
        self.fired = true;
    }
    pub fn reset(&mut self) {
        self.fired = false;
    }
}
