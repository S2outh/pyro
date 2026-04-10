use core::sync::atomic::Ordering;

use defmt::*;
use embassy_futures::select::{Either, select};
use embassy_stm32::can::{
    BufferedFdCanReceiver, BufferedFdCanSender,
    frame::{FdEnvelope, FdFrame},
};
use embassy_time::{Duration, Instant, Ticker};

use portable_atomic::{AtomicU8, AtomicU64};
use south_common::{
    chell::{ChellDefinition, ChellValue},
    definitions::internal_msgs,
    types::{Telecommand, Timesync},
};

use crate::{PyroTMContainer, TCSender, TMReceiver};

/// Request a timesync frame every N seconds
const TIMESYNC_REQ_ID: u8 = 3;
static REQ_TIME: AtomicU64 = AtomicU64::new(0);
static REQ_ANS_PRIO: AtomicU8 = AtomicU8::new(0);
static TIME_REF: AtomicU64 = AtomicU64::new(0);

fn gen_timesync_frame() -> FdFrame {
    let frame = FdFrame::new_standard(
        internal_msgs::TimesyncRequest.id(),
        core::slice::from_ref(&TIMESYNC_REQ_ID),
    )
    .unwrap();
    REQ_TIME.store(Instant::now().as_micros(), Ordering::Release);
    REQ_ANS_PRIO.store(u8::MAX, Ordering::Release);
    frame
}

fn gen_tm_frame(container: PyroTMContainer) -> FdFrame {
    FdFrame::new_standard(container.id(), container.fd_bytes()).unwrap()
}

#[embassy_executor::task]
pub async fn can_sender_thread(mut can_sender: BufferedFdCanSender, tm_channel: TMReceiver) {
    const REQ_INTERVALL: Duration = Duration::from_secs(10);
    let mut timesync_req_ticker = Ticker::every(REQ_INTERVALL);
    loop {
        can_sender
            .write(
                match select(timesync_req_ticker.next(), tm_channel.receive()).await {
                    Either::First(()) => gen_timesync_frame(),
                    Either::Second(tm) => gen_tm_frame(tm),
                },
            )
            .await;
    }
}

fn update_time_ref(frame: &FdFrame) {
    match Timesync::read(frame.data()) {
        Ok((_len, timesync_answer)) => {
            if timesync_answer.request_id != TIMESYNC_REQ_ID
                || timesync_answer.priority >= REQ_ANS_PRIO.load(Ordering::Acquire)
            {
                return;
            }
            REQ_ANS_PRIO.store(timesync_answer.priority, Ordering::Release);
            let transfer_time = Instant::now().as_micros() - REQ_TIME.load(Ordering::Acquire);
            let time_ref =
                timesync_answer.unix_time + transfer_time / 2 - Instant::now().as_micros();
            info!("Time ref is now {}", time_ref);
            TIME_REF.store(time_ref, Ordering::Relaxed);
        }
        Err(e) => error!("could not read timesync msg {}", Debug2Format(&e)),
    }
}

pub async fn handle_can_msg(envelope: FdEnvelope, tc_channel: TCSender) {
    if let embedded_can::Id::Standard(id) = envelope.frame.id() {
        if let Ok(def) = internal_msgs::from_id(id.as_raw()) {
            if def.as_any().is::<internal_msgs::TimesyncAnswer>() {
                update_time_ref(&envelope.frame);
            }
            if def.as_any().is::<internal_msgs::Telecommand>() {
                match Telecommand::read(envelope.frame.data()) {
                    Ok((_, cmd)) => tc_channel.send(cmd).await,
                    Err(_) => error!("error parsing tc"),
                }
            }
        } else {
            defmt::unreachable!("id not in any chell def block")
        }
    } else {
        defmt::unreachable!()
    };
}

/// receive can messages and put them in the corresponding beacons
#[embassy_executor::task]
pub async fn can_receiver_thread(can: BufferedFdCanReceiver, tc_channel: TCSender) {
    loop {
        // receive from can
        match can.receive().await {
            Ok(envelope) => handle_can_msg(envelope, tc_channel).await,
            Err(e) => error!("error in can frame! {}", e),
        };
    }
}
