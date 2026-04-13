use core::ptr::read_volatile;

const TS_CAL_1_REG: usize = 0x1FFF_75A8;
const TS_CAL_2_REG: usize = 0x1FFF_75CA;
const V_REFINT_REG: usize = 0x1FFF_75AA;

pub struct FactoryCalibratedValues {
    pub ts_cal_1: i32,
    pub ts_cal_rel: i32,
    pub v_refint: i32,
}
impl FactoryCalibratedValues {
    pub fn new() -> Self {
        unsafe {
            // Multipyl by 16 (leftshift 4) for 16 bit resolution
            let ts_cal_1 = 16 * read_volatile(TS_CAL_1_REG as *const u16) as i32;
            let ts_cal_2 = 16 * read_volatile(TS_CAL_2_REG as *const u16) as i32;
            let ts_cal_rel = ts_cal_2 - ts_cal_1;
            let v_refint = 16 * read_volatile(V_REFINT_REG as *const u16) as i32;
            Self {
                ts_cal_1,
                ts_cal_rel,
                v_refint,
            }
        }
    }
}
