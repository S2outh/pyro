use embassy_stm32::gpio::Output;

pub struct PyroChannel {
    safe: Output<'static>,
    fire: Output<'static>,
    fired: bool,
}

impl PyroChannel {
    pub fn new(safe: Output<'static>, fire: Output<'static>) -> Self {
        Self {
            safe,
            fire,
            fired: false,
        }
    }
    pub fn is_armed(&self) -> bool {
        self.safe.is_set_high()
    }
    pub fn is_fired(&self) -> bool {
        self.fired
    }
    pub fn arm(&mut self) {
        self.safe.set_high();
    }
    pub fn disarm(&mut self) {
        self.safe.set_low();
    }
    pub fn fire(&mut self) {
        self.fire.set_high();
        self.fired = true;
    }
    pub fn reset(&mut self) {
        self.fire.set_low();
        self.disarm();
    }
}
