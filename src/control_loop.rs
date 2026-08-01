use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Ticker, Timer};
use south_common::definitions::telemetry::pyro as tm;

use south_common::types::pyro::{PyroChannel as Ch, PyroCommand, StateFlags};

use crate::pyro_channel::PyroChannel;
use crate::{PyroChellUnion, PyroComChannels};

pub struct ControlLoop {
    com_channel: &'static PyroComChannels,
    pyro_channel_a: PyroChannel,
    pyro_channel_b: PyroChannel,
}

impl ControlLoop {
    pub fn spawn(
        com_channel: &'static PyroComChannels,
        pyro_channel_a: PyroChannel,
        pyro_channel_b: PyroChannel,
    ) -> Self {
        Self {
            com_channel,
            pyro_channel_a,
            pyro_channel_b,
        }
    }

    async fn handle_cmd(&mut self, cmd: PyroCommand) {
        match cmd {
            PyroCommand::Arm(channel) => match channel {
                Ch::Channel1 => self.pyro_channel_a.arm(),
                Ch::Channel2 => self.pyro_channel_b.arm(),
            },
            PyroCommand::Disarm(channel) => match channel {
                Ch::Channel1 => self.pyro_channel_a.disarm(),
                Ch::Channel2 => self.pyro_channel_b.disarm(),
            },

            PyroCommand::Fire(channel) => {
                let pyro_channel = match channel {
                    Ch::Channel1 => &mut self.pyro_channel_a,
                    Ch::Channel2 => &mut self.pyro_channel_b,
                };
                pyro_channel.fire();
                Timer::after_millis(400).await;
                pyro_channel.reset();
            }
        }
        self.send_state().await;
    }
    async fn send_state(&mut self) {
        let mut state_bitmap = StateFlags::empty();
        state_bitmap.set(StateFlags::ARMD_A, self.pyro_channel_a.is_armed());

        state_bitmap.set(StateFlags::FIRE_A, self.pyro_channel_a.is_fired());

        state_bitmap.set(StateFlags::ARMD_B, self.pyro_channel_b.is_armed());

        state_bitmap.set(StateFlags::FIRE_B, self.pyro_channel_b.is_fired());

        let container = PyroChellUnion::new(&tm::Status, &state_bitmap.bits()).unwrap();
        self.com_channel.send_tm(container).await;
    }
    pub async fn run(&mut self) -> ! {
        const CTRL_LOOP_TM_INTERVAL: Duration = Duration::from_millis(500);
        let mut tm_ticker = Ticker::every(CTRL_LOOP_TM_INTERVAL);

        loop {
            match select(tm_ticker.next(), self.com_channel.receive_tc()).await {
                Either::First(()) => self.send_state().await,
                Either::Second(cmd) => self.handle_cmd(cmd).await,
            }
        }
    }
}
