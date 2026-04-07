use core::ptr::read_volatile;

const TS_CAL_1_REG: usize = 0x1FFF_75A8;
const TS_CAL_2_REG: usize = 0x1FFF_75CA;
const V_REFINT_REG: usize = 0x1FFF_75AA;

pub struct FactoryCalibratedValues {
    pub ts_cal_1_x10: i32,
    pub ts_cal_rel_x10: i32,
    pub v_refint_x100: i32,
}
impl FactoryCalibratedValues {
    pub fn new() -> Self {
        unsafe {
            let ts_cal_1_x10 = 10 * read_volatile(TS_CAL_1_REG as *const u16) as i32;
            let ts_cal_2_x10 = 10 * read_volatile(TS_CAL_2_REG as *const u16) as i32;
            let ts_cal_rel_x10 = ts_cal_2_x10 - ts_cal_1_x10;
            let v_refint_x100 = 100 * read_volatile(V_REFINT_REG as *const u16) as i32;
            Self {
                ts_cal_1_x10,
                ts_cal_rel_x10,
                v_refint_x100,
            }
        }
    }
}
