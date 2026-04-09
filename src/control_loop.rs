use embassy_futures::select::{Either, select};
use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Ticker, Timer};
use south_common::definitions::telemetry::pyro as tm;
use south_common::types::Telecommand;

use south_common::types::pyro::{PyroCommand, StateFlags};

use crate::{PyroTMContainer, TCReceiver, TMSender};

const CTRL_LOOP_TM_INTERVAL: Duration = Duration::from_millis(500);

pub struct ControlLoop {
    cmd_receiver: TCReceiver,
    tm_sender: TMSender,
    safe_a: Output<'static>,
    fire_a: Output<'static>,
    safe_b: Output<'static>,
    fire_b: Output<'static>
}

impl ControlLoop {
    pub fn spawn(
        cmd_receiver: TCReceiver,
        tm_sender: TMSender,
        safe_a: Output<'static>,
        fire_a: Output<'static>,
        safe_b: Output<'static>,
        fire_b: Output<'static>
    ) -> Self {
        Self {
            cmd_receiver,
            tm_sender,
            safe_a,
            fire_a,
            safe_b,
            fire_b
        }
    }

    async fn handle_cmd(&mut self, cmd: Telecommand) {
        let Telecommand::Pyro(telecommand) = cmd else {
            return;
        };
        match telecommand {
            PyroCommand::Arm(channel) => {
                match channel {
                    0 => self.safe_a.set_low(),
                    1 => self.safe_b.set_low(),
                    _ => panic!("temp"),
                }
            }

            PyroCommand::Disarm(channel) => {
                match channel {
                    0 => self.safe_a.set_high(),
                    1 => self.safe_b.set_high(),
                    _ => panic!("temp"),
                }
            }

            PyroCommand::Fire(channel) => {
                let pin = match channel {
                    0 => &mut self.fire_a,
                    1 => &mut self.fire_b,
                    _ => panic!("temp"),
                };
                pin.set_high();
                Timer::after_micros(100).await;
                pin.set_low();
            }
        }
    }
    async fn send_state(&mut self) {
        let mut state_bitmap = StateFlags::empty();
        state_bitmap.set(
            StateFlags::SAFE_A,
            self.safe_a.is_set_low(),
        );

        state_bitmap.set(
            StateFlags::SAFE_B,
            self.safe_b.is_set_low(),
        );

        state_bitmap.set(
            StateFlags::FIRE_A,
            self.fire_a.is_set_high(),
        );

        state_bitmap.set(
            StateFlags::FIRE_B,
            self.fire_b.is_set_high(),
        );

        let container = PyroTMContainer::new(&tm::Status, &state_bitmap.bits()).unwrap();
        self.tm_sender.send(container).await;
    }
    pub async fn run(&mut self) -> ! {
        let mut tm_ticker = Ticker::every(CTRL_LOOP_TM_INTERVAL);

        loop {
            match select(
                tm_ticker.next(),
                self.cmd_receiver.receive(),
            ).await {
                Either::First(_) => self.send_state().await,
                Either::Second(cmd) => self.handle_cmd(cmd).await,
            }
        }
    }
}
