mod factory_calibrated_values;
mod util;

use core::array;

use heapless::Vec;
use util::Sortable;

use embassy_stm32::{
    Peri,
    adc::{
        Adc, AdcChannel, AdcConfig, AnyAdcChannel, CONTINUOUS, Exten, Instance, Ovsr, Ovss,
        Resolution, RingBufferedAdc, RxDma, SampleTime,
    },
    dma::InterruptHandler,
    interrupt::typelevel::Binding,
    pac,
};
use embassy_sync::watch::DynSender;

pub struct AdcCtrlChannel<'a, T: Instance> {
    channel: AnyAdcChannel<'a, T>,
    sender: Option<DynSender<'a, i16>>,
    conversion_func: fn(u16, u16) -> i16,
}
impl<'a, T: Instance> AdcCtrlChannel<'a, T> {
    pub fn new(
        channel: AnyAdcChannel<'a, T>,
        sender: DynSender<'a, i16>,
        conversion_func: fn(u16, u16) -> i16,
    ) -> Self {
        Self {
            channel,
            sender: Some(sender),
            conversion_func,
        }
    }
    fn new_ref(channel: AnyAdcChannel<'a, T>) -> Self {
        Self {
            channel,
            sender: None,
            conversion_func: |_, _| 0,
        }
    }
}

struct AdcChannelCtx<'a> {
    sender: Option<DynSender<'a, i16>>,
    conversion_func: fn(u16, u16) -> i16,
}
impl<'a> AdcChannelCtx<'a> {
    fn from(value: &mut AdcCtrlChannel<'a, impl Instance>) -> Self {
        Self {
            sender: value.sender.take(),
            conversion_func: value.conversion_func,
        }
    }
}

pub mod conversion {
    use super::factory_calibrated_values::FactoryCalibratedValues;
    use embassy_sync::lazy_lock::LazyLock;

    static CALIB: LazyLock<FactoryCalibratedValues> =
        LazyLock::new(|| FactoryCalibratedValues::new());

    // datasheet reference conditions
    const VREF_CALIB_MV: i64 = 3_000;
    const TS_1_VAL_TENTH_DEG: i64 = 30_0;
    const TS_2_VAL_TENTH_DEG: i64 = 130_0;
    const TS_REL_VAL_TENTH_DEG: i64 = TS_2_VAL_TENTH_DEG - TS_1_VAL_TENTH_DEG;

    const RAW_VALUE_RANGE: i64 = u16::MAX as i64;

    fn calculate_vref(calib_measurement: u16) -> i64 {
        let vref_measurement = calib_measurement as i64;
        VREF_CALIB_MV * CALIB.get().v_refint / vref_measurement
    }

    pub fn calculate_temperature_tenth_deg(measurement: u16, calib_measurement: u16) -> i16 {
        let vref_mv = calculate_vref(calib_measurement);
        let temp_measurement = measurement as i64;
        let temp_calibrated_measurement = temp_measurement * vref_mv / VREF_CALIB_MV;
        let calib = CALIB.get();
        let temp_tenth_deg = TS_REL_VAL_TENTH_DEG * (temp_calibrated_measurement - calib.ts_cal_1)
            / calib.ts_cal_rel
            + TS_1_VAL_TENTH_DEG;
        temp_tenth_deg as i16
    }

    pub fn calculate_bat_voltage_mv(measurement: u16, calib_measurement: u16) -> i16 {
        const R1_KOHM: i64 = 680;
        const R2_KOHM: i64 = 330;

        let vref_mv = calculate_vref(calib_measurement);
        let v_measurement = measurement as i64;
        let voltage_mv =
            v_measurement * (R1_KOHM + R2_KOHM) * vref_mv / (R2_KOHM * RAW_VALUE_RANGE);
        voltage_mv as i16
    }

    pub fn calculate_out_voltage_mv(measurement: u16, calib_measurement: u16) -> i16 {
        const R1_KOHM: i64 = 100;
        const R2_KOHM: i64 = 27;

        let vref_mv = calculate_vref(calib_measurement);
        let v_measurement = measurement as i64;
        let voltage_mv =
            v_measurement * (R1_KOHM + R2_KOHM) * vref_mv / (R2_KOHM * RAW_VALUE_RANGE);
        voltage_mv as i16
    }
}

pub enum Averaging {
    Samples16,
    Samples32,
    Samples64,
    Samples128,
    Samples256,
}

impl Averaging {
    fn oversampeling_ratio(&self) -> Ovsr {
        match *self {
            Averaging::Samples16 => Ovsr::MUL16,
            Averaging::Samples32 => Ovsr::MUL32,
            Averaging::Samples64 => Ovsr::MUL64,
            Averaging::Samples128 => Ovsr::MUL128,
            Averaging::Samples256 => Ovsr::MUL256,
        }
    }
    fn oversampeling_shift(&self) -> Ovss {
        match *self {
            Averaging::Samples16 => Ovss::NO_SHIFT,
            Averaging::Samples32 => Ovss::SHIFT1,
            Averaging::Samples64 => Ovss::SHIFT2,
            Averaging::Samples128 => Ovss::SHIFT3,
            Averaging::Samples256 => Ovss::SHIFT4,
        }
    }
}

pub struct AdcCtrl<
    'a,
    'c,
    T: Instance<Regs = pac::adc::Adc>,
    const CHANNELS: usize,
    const MES_SZE: usize,
> {
    rb_adc: RingBufferedAdc<'a, T>,
    channel_ctx: [AdcChannelCtx<'c>; CHANNELS],
    ref_channel_idx: usize,
}

impl<'a, 'c, T: Instance<Regs = pac::adc::Adc>, const CHANNELS: usize, const MES_SZE: usize>
    AdcCtrl<'a, 'c, T, CHANNELS, MES_SZE>
{
    pub fn new<D: RxDma<T>, const EXT_CHANNELS: usize>(
        adc_periph: Peri<'a, T>,
        dma_channel: Peri<'a, D>,
        dma_buffer: &'a mut [u16],
        irq: impl Binding<D::Interrupt, InterruptHandler<D>> + 'a,
        resolution: Resolution,
        averaging: Averaging,
        sample_time: SampleTime,
        temp_sender: DynSender<'c, i16>,
        external_channels: [AdcCtrlChannel<'c, T>; EXT_CHANNELS],
    ) -> Self {
        assert_eq!(
            EXT_CHANNELS,
            CHANNELS - 2,
            "Number of external channels should be total channels - 2"
        );
        assert_eq!(
            MES_SZE * 2,
            dma_buffer.len(),
            "Measurement buffer shoult be exactly half the size of the DMA buffer"
        );

        let mut adc_config = AdcConfig::default();
        adc_config.resolution = Some(resolution);
        adc_config.oversampling_ratio = Some(averaging.oversampeling_ratio()); // oversampling steps
        adc_config.oversampling_shift = Some(averaging.oversampeling_shift()); // right shift of oversampling reg
        adc_config.oversampling_enable = Some(true); // enable oversampling feature

        let adc = Adc::new_with_config(adc_periph, adc_config);

        let temp_channel = AdcCtrlChannel::new(
            adc.enable_temperature().degrade_adc(),
            temp_sender,
            conversion::calculate_temperature_tenth_deg,
        );
        let ref_channel = AdcCtrlChannel::new_ref(adc.enable_vrefint().degrade_adc());
        let mut channels: Vec<AdcCtrlChannel<'c, T>, CHANNELS> =
            external_channels.into_iter().collect();
        channels.push(temp_channel).ok();
        channels.push(ref_channel).ok();
        channels.sort_by(|c1, c2| {
            c1.channel
                .get_hw_channel()
                .cmp(&c2.channel.get_hw_channel())
        });
        let ref_channel_idx = channels.iter().position(|c| c.sender.is_none()).unwrap();

        let (channel_ctx, sequence): (Vec<_, CHANNELS>, Vec<_, CHANNELS>) = channels
            .into_iter()
            .map(|mut c| (AdcChannelCtx::from(&mut c), (c.channel, sample_time)))
            .unzip();

        let rb_adc = adc.into_ring_buffered(
            dma_channel,
            dma_buffer,
            irq,
            sequence.into_iter(),
            CONTINUOUS,
            Exten::RISING_EDGE,
        );
        let channel_ctx = channel_ctx.into_array().unwrap_or_else(|_| unreachable!());

        Self {
            rb_adc,
            ref_channel_idx,
            channel_ctx,
        }
    }

    async fn measure(&mut self) -> [u16; CHANNELS] {
        let mut measurements = [0u16; MES_SZE];

        if let Err(e) = self.rb_adc.read(&mut measurements).await {
            defmt::error!("adc error: {}", e);
            self.rb_adc.clear();
        }

        let mut averaged = [0u32; CHANNELS];
        for slice in measurements.chunks_exact(CHANNELS) {
            for (avg, val) in averaged.iter_mut().zip(slice) {
                *avg += *val as u32;
            }
        }

        let swr_ovs = MES_SZE / CHANNELS;
        averaged.map(|v| (v / swr_ovs as u32) as u16)
    }
    fn convert(&self, values: [u16; CHANNELS]) -> [i16; CHANNELS] {
        let v_ref_measurement: u16 = values[self.ref_channel_idx];

        let mut iter = self
            .channel_ctx
            .iter()
            .zip(values)
            .map(|(c, v)| (c.conversion_func)(v, v_ref_measurement));

        array::from_fn(|_| iter.next().unwrap())
    }
    fn send(&self, values: [i16; CHANNELS]) {
        self.channel_ctx.iter().zip(values).for_each(|(c, v)| {
            if let Some(sender) = c.sender.as_ref() {
                sender.send(v)
            }
        });
    }

    pub async fn run(&mut self) -> ! {
        self.rb_adc.start();

        loop {
            let raw_values = self.measure().await;
            let converted_values = self.convert(raw_values);
            self.send(converted_values);
        }
    }
}
