#![no_std]
#![no_main]
#![feature(variant_count)]
#![feature(type_alias_impl_trait)]
#![feature(iter_collect_into)]
#![feature(iterator_try_collect)]

mod io_threads;
mod adc;

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::{
    Config, bind_interrupts,
    can::{
        self, CanConfigurator, RxFdBuf, TxFdBuf,
    },
    exti::InterruptHandler,
    gpio::{Level, Output, Speed},
    interrupt::typelevel::EXTI4_15,
    peripherals::{FDCAN1, IWDG},
    rcc::{self, mux::Fdcansel},
    wdg::IndependentWatchdog,
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, Receiver, Sender},
};
use embassy_time::Timer;
use south_common::{
    configs::can_config::CanPeriphConfig, definitions::{internal_msgs, telemetry::lower_sensor as tm}, chell::{ChellDefinition, fd_compat_chell_union}, types::Telecommand
};
use static_cell::StaticCell;

use crate::io_threads::{can_receiver_thread, can_sender_thread};

use {defmt_rtt as _, panic_probe as _};

// bind interrupts
bind_interrupts!(struct Irqs {
    EXTI4_15 => InterruptHandler<EXTI4_15>;

    TIM16_FDCAN_IT0 => can::IT0InterruptHandler<FDCAN1>;
    TIM17_FDCAN_IT1 => can::IT1InterruptHandler<FDCAN1>;

    // TIM16_FDCAN_IT0 => can::IT0InterruptHandler<FDCAN2>;
    // TIM17_FDCAN_IT1 => can::IT1InterruptHandler<FDCAN2>;
});

/// config rcc for higher sysclock and fdcan periph clock to make sure
/// all messages can be received without package drop
fn get_rcc_config() -> rcc::Config {
    let mut rcc_config = rcc::Config::default();
    rcc_config.hsi = Some(rcc::Hsi {
        sys_div: rcc::HsiSysDiv::DIV1,
    });
    rcc_config.sys = rcc::Sysclk::PLL1_R;
    rcc_config.pll = Some(rcc::Pll {
        source: rcc::PllSource::HSI,
        prediv: rcc::PllPreDiv::DIV1,
        mul: rcc::PllMul::MUL8,
        divp: None,
        divq: Some(rcc::PllQDiv::DIV2),
        divr: Some(rcc::PllRDiv::DIV2),
    });
    rcc_config.mux.fdcansel = Fdcansel::PLL1_Q;
    rcc_config
}

// General setup stuff
const STARTUP_DELAY: u64 = 300;

const WATCHDOG_TIMEOUT_US: u32 = 300_000;
const WATCHDOG_PETTING_INTERVAL_US: u32 = WATCHDOG_TIMEOUT_US / 2;

// Telemtry container
type LowerSensorTMContainer = fd_compat_chell_union!(tm);

const TM_CHANNEL_BUF_SIZE: usize = 5;
const CMD_CHANNEL_BUF_SIZE: usize = 5;

type TMSender = Sender<'static, ThreadModeRawMutex, LowerSensorTMContainer, TM_CHANNEL_BUF_SIZE>;
type TMReceiver = Receiver<'static, ThreadModeRawMutex, LowerSensorTMContainer, TM_CHANNEL_BUF_SIZE>;
static TMC: StaticCell<Channel<ThreadModeRawMutex, LowerSensorTMContainer, TM_CHANNEL_BUF_SIZE>> =
    StaticCell::new();
type TCSender = Sender<'static, ThreadModeRawMutex, Telecommand, CMD_CHANNEL_BUF_SIZE>;
type TCReceiver = Receiver<'static, ThreadModeRawMutex, Telecommand, CMD_CHANNEL_BUF_SIZE>;
static CMDC: StaticCell<Channel<ThreadModeRawMutex, Telecommand, CMD_CHANNEL_BUF_SIZE>> =
    StaticCell::new();

// CAN configuration
const RX_BUF_SIZE: usize = 64;
const TX_BUF_SIZE: usize = 64;

static RX_BUF: StaticCell<RxFdBuf<RX_BUF_SIZE>> = StaticCell::new();
static TX_BUF: StaticCell<TxFdBuf<TX_BUF_SIZE>> = StaticCell::new();

#[embassy_executor::task]
async fn petter(mut watchdog: IndependentWatchdog<'static, IWDG>) {
    loop {
        watchdog.pet();
        Timer::after_micros(WATCHDOG_PETTING_INTERVAL_US.into()).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.rcc = get_rcc_config();
    let p = embassy_stm32::init(config);
    
    const FW_VERSION: &str = env!("FW_VERSION");
    const FW_HASH: &str = env!("FW_HASH");

    info!(
        "Launching: FW version={} hash={}",
        FW_VERSION,
        FW_HASH
    );

    // create independent watchdog
    let mut watchdog = IndependentWatchdog::new(p.IWDG, WATCHDOG_TIMEOUT_US);

    // TM channel setup
    let tm_channel = TMC.init(Channel::new());
    let cmd_channel = CMDC.init(Channel::new());


    // -- CAN configuration
    // can 1 configuration
    let mut can_configurator =
        CanPeriphConfig::new(CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs));

    // can 2 configuration
    // let mut can_configurator =
    //     CanPeriphConfig::new(CanConfigurator::new(p.FDCAN2, p.PB0, p.PB1, Irqs));
    
    let _can_1_standby = Output::new(p.PA10, Level::Low, Speed::Low);
    // let _can_2_standby = Output::new(p.PB2, Level::Low, Speed::Low);

    can_configurator
        .add_receive_topic(internal_msgs::Telecommand.id())
        .unwrap();

    let can_interface = can_configurator.activate(
        TX_BUF.init(TxFdBuf::<TX_BUF_SIZE>::new()),
        RX_BUF.init(RxFdBuf::<RX_BUF_SIZE>::new()),
    );

    // Adc configuration
    let safe_a = Output::new(p.PA10, Level::Low, Speed::Low);
    let fire_a = Output::new(p.PB3, Level::Low, Speed::Low);

    let safe_b = Output::new(p.PB5, Level::Low, Speed::Low);
    let fire_b = Output::new(p.PB4, Level::Low, Speed::Low);

    let adc_periph = Adc::new(p.ADC1);

    let temp_watch = Watch::<ThreadModeRawMutex, i16, 1>::new();
    let out_a_watch = Watch::<ThreadModeRawMutex, i16, 1>::new();
    let out_b_watch = Watch::<ThreadModeRawMutex, i16, 1>::new();
    let current_test_watch = TW.init(Watch::new());
    
    let out_a_channel = AdcCtrlChannel::new(
        p.PA0.degrade_adc(),
        out_a_watch.sender().as_dyn(),
        adc::conversion::calculate_voltage_10mv
    );

    let out_b_channel = AdcCtrlChannel::new(
        p.PA1.degrade_adc(),
        out_b_watch.sender().as_dyn(),
        adc::conversion::calculate_voltage_10mv
    );

    let mut adc: AdcCtrl<'_, '_, _, 4> = AdcCtrl::new(adc_periph, p.DMA1_CH1, temp_watch.sender().as_dyn(), [out_a_channel, out_b_channel]);

    // Thread spawning
    watchdog.unleash();
    spawner.spawn(petter(watchdog).unwrap());

    Timer::after_millis(STARTUP_DELAY).await;

    spawner.spawn(can_sender_thread(can_interface.writer(), tm_channel.receiver()).unwrap());
    spawner.spawn(can_receiver_thread(can_interface.reader(), cmd_channel.sender()).unwrap());

    // wait until all other threads finished (never)
    core::future::pending::<()>().await;
}
