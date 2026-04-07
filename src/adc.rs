mod factory_calibrated_values;
mod util;

use embassy_time::{Duration, Ticker};
use util::Sortable;

use embassy_stm32::{
    Peri,
    adc::{Adc, AdcChannel, AnyAdcChannel, RxDma, SampleTime},
    peripherals::{ADC1, DMA1_CH1},
};
use embassy_sync::watch::DynSender;
use heapless::Vec;

// Adc reading task
#[embassy_executor::task]
pub async fn adc_thread(mut adc: AdcCtrl<'static, 'static, DMA1_CH1, 4>) {
    const ADC_LOOP_LEN: Duration = Duration::from_millis(50);
    let mut ticker = Ticker::every(ADC_LOOP_LEN);
    loop {
        adc.run().await;
        ticker.next().await;
    }
}

pub struct AdcCtrlChannel<'a> {
    channel: AnyAdcChannel<ADC1>,
    sender: Option<DynSender<'a, i16>>,
    conversion_func: fn(u16, u16) -> i16,
}
impl<'a> AdcCtrlChannel<'a> {
    pub fn new(
        channel: AnyAdcChannel<ADC1>,
        sender: DynSender<'a, i16>,
        conversion_func: fn(u16, u16) -> i16
    ) -> Self {
        Self { channel, sender: Some(sender), conversion_func }
    }
    fn new_ref(
        channel: AnyAdcChannel<ADC1>,
    ) -> Self {
        Self { channel, sender: None, conversion_func: |_,_|{0} }
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

pub struct AdcCtrl<'a, 'd, D: RxDma<ADC1>, const N: usize> {
    adc: Adc<'d, ADC1>,
    dma_channel: Peri<'d, D>,
    ref_channel_idx: usize,
    // adc channels
    channels: Vec<AdcCtrlChannel<'a>, N>,
}

impl<'a, 'd, D: RxDma<ADC1>, const N: usize> AdcCtrl<'a, 'd, D, N> {
    pub fn new(
        mut adc: Adc<'d, ADC1>,
        dma_channel: Peri<'d, D>,
        temp_sender: DynSender<'a, i16>,
        external_channels: [AdcCtrlChannel<'a>; N - 2],
    ) -> Self {
        adc.set_resolution(embassy_stm32::adc::Resolution::BITS12);
        // 16x oversampling
        adc.set_oversampling_ratio(0x03); // 2^n oversampling steps: 2^3 = 16
        adc.set_oversampling_shift(0x04); // right shift of oversampling reg, usually n+1: avg = sum >> n+1
        adc.oversampling_enable(true); // enable oversampling feature

        let temp_channel = AdcCtrlChannel::new(
            adc.enable_temperature().degrade_adc(),
            temp_sender,
            conversion::calculate_temperature_tenth_deg,
        );
        let ref_channel = AdcCtrlChannel::new_ref(adc.enable_vrefint().degrade_adc());
        let mut channels: Vec<AdcCtrlChannel<'a>, N> = external_channels.into_iter().collect();
        channels.push(temp_channel).ok();
        channels.push(ref_channel).ok();
        channels.sort_by(|c1, c2| {
            c1.channel
                .get_hw_channel()
                .cmp(&c2.channel.get_hw_channel())
        });
        let ref_channel_idx = channels.iter().position(|c| c.sender.is_none()).unwrap();

        Self {
            adc,
            dma_channel,
            ref_channel_idx,
            channels,
        }
    }

    async fn measure(&mut self) -> Vec<u16, N> {
        let mut measurements = [0u16; N];
        let sequence = self
            .channels
            .iter_mut()
            .map(|c| (&mut c.channel, SampleTime::CYCLES160_5));

        self.adc
            .read(self.dma_channel.reborrow(), sequence, &mut measurements)
            .await;

        Vec::from_array(measurements)
    }
    fn convert(&self, values: Vec<u16, N>) -> Vec<i16, N> {
        let v_ref_measurement: u16 = values[self.ref_channel_idx];

        self.channels
            .iter()
            .zip(values)
            .map(|(c, v)| (c.conversion_func)(v, v_ref_measurement))
            .collect()
    }
    fn send(&self, values: Vec<i16, N>) {
        self.channels
            .iter()
            .zip(values)
            .for_each(|(c, v)| 
                if let Some(sender) = c.sender.as_ref() {
                    sender.send(v)
                }
            );
    }

    pub async fn run(&mut self) {
        let raw_values = self.measure().await;
        let converted_values = self.convert(raw_values);
        self.send(converted_values);
    }
}

