mod factory_calibrated_values;
mod util;

use core::array;

use heapless::Vec;
use util::Sortable;

use embassy_stm32::{
    Peri,
    adc::{Adc, AdcChannel, AnyAdcChannel, Exten, Instance, RegularTrigger, RingBufferedAdc, RxDma, SampleTime},
    dma::InterruptHandler,
    interrupt::typelevel::Binding,
    pac
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
        conversion_func: fn(u16, u16) -> i16
    ) -> Self {
        Self { channel, sender: Some(sender), conversion_func }
    }
    fn new_ref(
        channel: AnyAdcChannel<'a, T>,
    ) -> Self {
        Self { channel, sender: None, conversion_func: |_,_|{0} }
    }
}

struct AdcChannelCtx<'a> {
    sender: Option<DynSender<'a, i16>>,
    conversion_func: fn(u16, u16) -> i16,
}
impl<'a> AdcChannelCtx<'a> {
    fn from(value: &mut AdcCtrlChannel<'a, impl Instance>) -> Self {
        Self { sender: value.sender.take(), conversion_func: value.conversion_func }
    }
}

pub mod conversion {
    use super::factory_calibrated_values::FactoryCalibratedValues;
    use embassy_sync::lazy_lock::LazyLock;

    static CALIB: LazyLock<FactoryCalibratedValues> = LazyLock::new(|| FactoryCalibratedValues::new());

    // datasheet reference conditions
    const VREF_CALIB_10MV: i32 = 3_00;
    const TS_1_VAL_TENTH_DEG: i32 = 30_0;
    const TS_2_VAL_TENTH_DEG: i32 = 130_0;
    const TS_REL_VAL_TENTH_DEG: i32 = TS_2_VAL_TENTH_DEG - TS_1_VAL_TENTH_DEG;

    const RAW_VALUE_RANGE_X100: i32 = 4096_00;

    // == Voltage divider == 
    const R1_OHM: i32 = 27;
    const R2_OHM: i32 = 100;

    const V_DIVIDER_MULT: i32 = (R1_OHM + R2_OHM) / R2_OHM;

    fn calculate_vref(calib_measurement: u16) -> i32 {
        let vref_measurement_x100 = 100 * calib_measurement as i32;
        VREF_CALIB_10MV * CALIB.get().v_refint_x100 / vref_measurement_x100
    }

    pub fn calculate_temperature_tenth_deg(measurement: u16, calib_measurement: u16) -> i16 {
        let vref_10mv = calculate_vref(calib_measurement);
        let temp_measurement_x10 = 10 * measurement as i32;
        let temp_calibrated_measurement = temp_measurement_x10 * vref_10mv / VREF_CALIB_10MV;
        let calib = CALIB.get();
        let temp_tenth_deg = TS_REL_VAL_TENTH_DEG
            * (temp_calibrated_measurement - calib.ts_cal_1_x10)
            / calib.ts_cal_rel_x10
            + TS_1_VAL_TENTH_DEG;
        temp_tenth_deg as i16
    }

    pub fn calculate_voltage_10mv(measurement: u16, calib_measurement: u16) -> i16 {
        let vref_10mv = calculate_vref(calib_measurement);
        let vbat_1_measurement_x100 = 100 * measurement as i32;
        let voltage_mv =
            vbat_1_measurement_x100 * V_DIVIDER_MULT * vref_10mv / RAW_VALUE_RANGE_X100;
        voltage_mv as i16
    }
}

pub struct AdcCtrl<'a, 'c, T: Instance<Regs = pac::adc::Adc>, const CHANNELS: usize, const MES_SZE: usize> {
    rb_adc: RingBufferedAdc<'a, T>,
    channel_ctx: [AdcChannelCtx<'c>; CHANNELS],
    ref_channel_idx: usize,
}

impl<'a, 'c, T: Instance<Regs = pac::adc::Adc>, const CHANNELS: usize, const MES_SZE: usize> AdcCtrl<'a, 'c, T, CHANNELS, MES_SZE> {
    pub fn new<D: RxDma<T>>(
        adc: Adc<'a, T>,
        dma_channel: Peri<'a, D>,
        dma_buffer: &'a mut [u16],
        irq: impl Binding<D::Interrupt, InterruptHandler<D>> + 'a,
        trigger: impl RegularTrigger<T>,
        edge: Exten,
        sample_time: SampleTime,
        temp_sender: DynSender<'c, i16>,
        external_channels: [AdcCtrlChannel<'c, T>; CHANNELS - 2],
    ) -> Self {
        assert_eq!(MES_SZE * 2, dma_buffer.len());

        let temp_channel = AdcCtrlChannel::new(
            adc.enable_temperature().degrade_adc(),
            temp_sender,
            conversion::calculate_temperature_tenth_deg,
        );
        let ref_channel = AdcCtrlChannel::new_ref(adc.enable_vrefint().degrade_adc());
        let mut channels: Vec<AdcCtrlChannel<'c, T>, CHANNELS> = external_channels.into_iter().collect();
        channels.push(temp_channel).ok();
        channels.push(ref_channel).ok();
        channels.sort_by(|c1, c2| {
            c1.channel
                .get_hw_channel()
                .cmp(&c2.channel.get_hw_channel())
        });
        let ref_channel_idx = channels.iter().position(|c| c.sender.is_none()).unwrap();

        let (channel_ctx, sequence): (Vec<_, CHANNELS>, Vec<_, CHANNELS>) =
            channels
            .into_iter()
            .map(|mut c| (AdcChannelCtx::from(&mut c), (c.channel, sample_time)))
            .unzip();
        
        let rb_adc = adc.into_ring_buffered(dma_channel, dma_buffer, irq, sequence.into_iter(), trigger, edge);
        let channel_ctx = channel_ctx.into_array().unwrap_or_else(|_| unreachable!());

        Self {
            rb_adc,
            ref_channel_idx,
            channel_ctx,
        }
    }

    async fn measure(&mut self) -> [u16; CHANNELS] {
        let mut measurements = [0u16; MES_SZE];

        self.rb_adc
            .read(&mut measurements)
            .await.unwrap_or_else(|_| panic!("adc overrun"));

        measurements[MES_SZE-CHANNELS..].try_into().unwrap()
    }
    fn convert(&self, values: [u16; CHANNELS]) -> [i16; CHANNELS] {
        let v_ref_measurement: u16 = values[self.ref_channel_idx];

        let mut iter = self.channel_ctx
            .iter()
            .zip(values)
            .map(|(c, v)| (c.conversion_func)(v, v_ref_measurement));
        
        array::from_fn(|_| iter.next().unwrap())
    }
    fn send(&self, values: [i16; CHANNELS]) {
        self.channel_ctx
            .iter()
            .zip(values)
            .for_each(|(c, v)| 
                if let Some(sender) = c.sender.as_ref() {
                    sender.send(v)
                }
            );
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

