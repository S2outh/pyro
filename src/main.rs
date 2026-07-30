#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)] // required by `embassy_executor::task` with the `nightly` feature
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

mod adc;
mod control_loop;
mod pyro_channel;

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::{
    Config,
    adc::{AdcChannel, Resolution, SampleTime},
    bind_interrupts,
    can::{self, CanConfigurator, RxFdBuf, TxFdBuf},
    dma,
    gpio::{Level, Output, Speed},
    peripherals::{ADC1, DMA1_CH1, FDCAN1, IWDG},
    rcc::{
        self,
        mux::{Adcsel, Fdcansel},
    },
    wdg::IndependentWatchdog,
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    watch::{self, Watch},
};
use embassy_time::Timer;
use south_common::{
    chell::ChellDefinition,
    configs::can_config::CanPeriphConfig,
    definitions::{internal_msgs, telemetry::pyro as tm},
    gen_obdh_types,
    obdh::EmptyFunc,
    types::pyro::PyroCommand,
};
use static_cell::StaticCell;

use crate::{
    adc::{AdcCtrl, AdcCtrlChannel, Averaging},
    control_loop::ControlLoop,
    pyro_channel::PyroChannel,
};

use {defmt_rtt as _, panic_probe as _};

// bind interrupts
bind_interrupts!(struct Irqs {
    TIM16_FDCAN_IT0 => can::IT0InterruptHandler<FDCAN1>;
    TIM17_FDCAN_IT1 => can::IT1InterruptHandler<FDCAN1>;

    // TIM16_FDCAN_IT0 => can::IT0InterruptHandler<FDCAN2>;
    // TIM17_FDCAN_IT1 => can::IT1InterruptHandler<FDCAN2>;

    // Adc dma stream
    DMA1_CHANNEL1 => dma::InterruptHandler<DMA1_CH1>;
});

/// config rcc for higher sysclock and fdcan periph clock to make sure
/// all messages can be received without package drop
fn get_rcc_config() -> rcc::Config {
    let mut rcc_config = rcc::Config::default();
    // 16 MHz
    rcc_config.hsi = Some(rcc::Hsi {
        sys_div: rcc::HsiSysDiv::DIV1,
    });
    rcc_config.pll = Some(rcc::Pll {
        source: rcc::PllSource::HSI,     // 16 MHz
        prediv: rcc::PllPreDiv::DIV1,    // 16 MHz
        mul: rcc::PllMul::MUL8,          // 128 MHz
        divp: Some(rcc::PllPDiv::DIV32), // 4 MHz
        divq: Some(rcc::PllQDiv::DIV2),  // 64 MHz
        divr: Some(rcc::PllRDiv::DIV2),  // 64 MHz
    });
    rcc_config.sys = rcc::Sysclk::PLL1_R; // 64 MHz
    rcc_config.mux.fdcansel = Fdcansel::PLL1_Q; // 64 MHz
    rcc_config.mux.adcsel = Adcsel::PLL1_P; // 4 MHz
    rcc_config
}

// General setup stuff
const STARTUP_DELAY: u64 = 300;

const WATCHDOG_TIMEOUT_US: u32 = 300_000;
const WATCHDOG_PETTING_INTERVAL_US: u32 = WATCHDOG_TIMEOUT_US / 2;

// adc buffer
const ADC_NUM_CHANNELS: usize = 6;
const ADC_BUF_SIZE: usize = ADC_NUM_CHANNELS * 14; // At least two times num_channels
static ADC_BUF: StaticCell<[u16; ADC_BUF_SIZE]> = StaticCell::new();

// TM container
gen_obdh_types!(Pyro, tm, cmd => PyroCommand);

// internal messaging channels
static COM_CHANNELS: PyroComChannels = PyroComChannels::new(6);

// CAN configuration
const C_RX_BUF_SIZE: usize = 64;
const C_TX_BUF_SIZE: usize = 64;

static C_RX_BUF: StaticCell<RxFdBuf<C_RX_BUF_SIZE>> = StaticCell::new();
static C_TX_BUF: StaticCell<TxFdBuf<C_TX_BUF_SIZE>> = StaticCell::new();

// ADC watch channels
type AdcWatch = Watch<ThreadModeRawMutex, i16, 1>;
static TEMP_WATCH: AdcWatch = Watch::new();
static OUT_A_WATCH: AdcWatch = Watch::new();
static OUT_B_WATCH: AdcWatch = Watch::new();
static BAT_A_WATCH: AdcWatch = Watch::new();
static BAT_B_WATCH: AdcWatch = Watch::new();

#[embassy_executor::task]
async fn petter(mut watchdog: IndependentWatchdog<'static, IWDG>) {
    loop {
        watchdog.pet();
        Timer::after_micros(WATCHDOG_PETTING_INTERVAL_US.into()).await;
    }
}

#[embassy_executor::task]
pub async fn can_receiver_task(mut can_receiver: PyroCanReceiver) -> ! {
    can_receiver.run().await
}

#[embassy_executor::task]
pub async fn can_sender_task(mut can_sender: PyroCanSender) -> ! {
    can_sender.run().await
}

// Adc running task
#[embassy_executor::task]
pub async fn adc_thread(
    mut adc: AdcCtrl<'static, 'static, ADC1, ADC_NUM_CHANNELS, { ADC_BUF_SIZE / 2 }>,
) -> ! {
    adc.run().await
}

// control loop task
#[embassy_executor::task]
pub async fn ctrl_thread(mut control_loop: ControlLoop) -> ! {
    control_loop.run().await
}

// adc to telem conversion tasks
#[embassy_executor::task(pool_size = 5)]
pub async fn adc_telem_thread(
    tm_sender: PyroTMSender,
    mut adc_recv: watch::Receiver<'static, ThreadModeRawMutex, i16, 1>,
    addr: &'static dyn ChellDefinition,
) {
    loop {
        let value = adc_recv.changed().await;
        let container = PyroChellUnion::new(addr, &value).unwrap();
        tm_sender.send(container).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.rcc = get_rcc_config();
    let p = embassy_stm32::init(config);

    const FW_VERSION: &str = env!("FW_VERSION");
    const FW_HASH: &str = env!("FW_HASH");

    info!("Launching: FW version={} hash={}", FW_VERSION, FW_HASH);

    // create independent watchdog
    let mut watchdog = IndependentWatchdog::new(p.IWDG, WATCHDOG_TIMEOUT_US);

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

    let can_instance = can_configurator.activate(
        C_TX_BUF.init(TxFdBuf::<C_TX_BUF_SIZE>::new()),
        C_RX_BUF.init(RxFdBuf::<C_RX_BUF_SIZE>::new()),
    );

    // Setup can sender and receiver runners
    let can_receiver = PyroCanReceiver::new(can_instance.reader(), &COM_CHANNELS, EmptyFunc);

    let can_sender = PyroCanSender::new(can_instance.writer(), &COM_CHANNELS);

    // Pyro channel configuration
    let safe_a = Output::new(p.PB9, Level::Low, Speed::Low);
    let fire_a = Output::new(p.PB4, Level::Low, Speed::Low);

    let safe_b = Output::new(p.PB8, Level::Low, Speed::Low);
    let fire_b = Output::new(p.PB5, Level::Low, Speed::Low);

    // Adc configuration
    let out_a_channel = AdcCtrlChannel::new(
        p.PA1.degrade_adc(),
        OUT_A_WATCH.dyn_sender(),
        adc::conversion::calculate_out_voltage_mv,
    );

    let out_b_channel = AdcCtrlChannel::new(
        p.PA0.degrade_adc(),
        OUT_B_WATCH.dyn_sender(),
        adc::conversion::calculate_out_voltage_mv,
    );

    let bat_a_channel = AdcCtrlChannel::new(
        p.PA3.degrade_adc(),
        BAT_A_WATCH.dyn_sender(),
        adc::conversion::calculate_bat_voltage_mv,
    );

    let bat_b_channel = AdcCtrlChannel::new(
        p.PA2.degrade_adc(),
        BAT_B_WATCH.dyn_sender(),
        adc::conversion::calculate_bat_voltage_mv,
    );

    // cycle num per channel = (sample_time + conversion_time(fixed by resolution)) * oversampeling
    // = (160.5 + 12.5) * 256 = 44288 cycles.
    // cycle time per channel = cycle num / adc clock = 44288 / 4_000_000 = 11.072 ms
    // total cycle time = cycle time per channel * number of channels = 11.072 ms * 6 = 66.432 ms
    // dma triggers when buffer is half full:
    // trigger = total cycle time * (adc buf size multiplier / 2) = 66.432 * (14 / 2) = 464.024 ms
    //
    // alt. (every second) trigger = 66.432 * (30 / 2) = 996.48 ms
    //
    // The adc is in continuous trigger mode and will not pause between reads
    // The adc ctrl loop software averages the v/alues read on interrupt

    let adc: AdcCtrl<'_, '_, _, ADC_NUM_CHANNELS, { ADC_BUF_SIZE / 2 }> = AdcCtrl::new(
        p.ADC1,
        p.DMA1_CH1,
        ADC_BUF.init([0; _]),
        Irqs,
        Resolution::BITS12,
        Averaging::Samples256,
        SampleTime::CYCLES160_5,
        TEMP_WATCH.dyn_sender(),
        [out_a_channel, out_b_channel, bat_a_channel, bat_b_channel],
    );

    // Control loop setup
    let pyro_channel_a = PyroChannel::new(safe_a, fire_a);
    let pyro_channel_b = PyroChannel::new(safe_b, fire_b);

    let control_loop = ControlLoop::spawn(
        COM_CHANNELS.get_tc_receiver(),
        COM_CHANNELS.get_tm_sender(),
        pyro_channel_a,
        pyro_channel_b,
    );

    // Thread spawning
    watchdog.unleash();
    spawner.spawn(petter(watchdog).unwrap());

    Timer::after_millis(STARTUP_DELAY).await;

    spawner.spawn(adc_thread(adc).unwrap());
    spawner.spawn(ctrl_thread(control_loop).unwrap());

    spawner.spawn(can_sender_task(can_sender).unwrap());
    spawner.spawn(can_receiver_task(can_receiver).unwrap());

    // adc telem threads
    spawner.spawn(
        adc_telem_thread(
            COM_CHANNELS.get_tm_sender(),
            TEMP_WATCH.receiver().unwrap(),
            &tm::InternalTemperature,
        )
        .unwrap(),
    );

    spawner.spawn(
        adc_telem_thread(
            COM_CHANNELS.get_tm_sender(),
            BAT_A_WATCH.receiver().unwrap(),
            &tm::Bat1Voltage,
        )
        .unwrap(),
    );

    spawner.spawn(
        adc_telem_thread(
            COM_CHANNELS.get_tm_sender(),
            BAT_B_WATCH.receiver().unwrap(),
            &tm::Bat2Voltage,
        )
        .unwrap(),
    );

    spawner.spawn(
        adc_telem_thread(
            COM_CHANNELS.get_tm_sender(),
            OUT_A_WATCH.receiver().unwrap(),
            &tm::Out1Voltage,
        )
        .unwrap(),
    );

    spawner.spawn(
        adc_telem_thread(
            COM_CHANNELS.get_tm_sender(),
            OUT_B_WATCH.receiver().unwrap(),
            &tm::Out2Voltage,
        )
        .unwrap(),
    );

    // wait until all other threads finished (never)
    core::future::pending::<()>().await;
}
