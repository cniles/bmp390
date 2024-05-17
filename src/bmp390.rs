use std::sync::{Arc, Mutex};

use embedded_hal::i2c::I2c;

#[derive(Debug)]
pub struct BMP390<I2C> {
    i2c: Arc<Mutex<I2C>>,
    address: DeviceAddr,
    cs: Option<CalibrationStruct>,
}

impl<I2C> Clone for BMP390<I2C> {
    fn clone(&self) -> Self {
        Self {
            i2c: self.i2c.clone(),
            address: self.address.clone(),
            cs: self.cs.clone(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Copy)]
pub enum DeviceAddr {
    AD0 = 0b1110110,
    AD1 = 0b1110111,
}

#[derive(Clone, Copy, Debug)]
pub enum Bmp390Error<I2C> {
    I2c(I2C),
}

impl<I2C> BMP390<I2C>
where
    I2C: I2c,
{
    pub fn new(i2c: Arc<Mutex<I2C>>, address: DeviceAddr) -> Result<Self, Bmp390Error<I2C::Error>> {
        let mut bmp390 = Self {
            i2c,
            address,
            cs: None,
        };
        let cs = bmp390.read_calibration_struct()?;

        bmp390.cs = Some(cs.to_coefficients());

        Ok(bmp390)
    }

    pub fn read_calibration_struct(
        &mut self,
    ) -> Result<RawCalibrationStruct, Bmp390Error<I2C::Error>> {
        Ok(RawCalibrationStruct {
            t1: self.read_u16(Register::NvmParT1)?,
            t2: self.read_u16(Register::NvmParT2)?,
            t3: self.read_i8(Register::NvmParT3)?,
            p1: self.read_i16(Register::NvmParP1)?,
            p2: self.read_i16(Register::NvmParP2)?,
            p3: self.read_i8(Register::NvmParP3)?,
            p4: self.read_i8(Register::NvmParP4)?,
            p5: self.read_u16(Register::NvmParP5)?,
            p6: self.read_u16(Register::NvmParP6)?,
            p7: self.read_i8(Register::NvmParP7)?,
            p8: self.read_i8(Register::NvmParP8)?,
            p9: self.read_i16(Register::NvmParP9)?,
            p10: self.read_i8(Register::NvmParP10)?,
            p11: self.read_i8(Register::NvmParP11)?,
        })
    }

    pub fn read_device_id_register(&mut self) -> Result<u8, Bmp390Error<I2C::Error>> {
        self.read_u8(Register::ChipId)
    }

    pub fn write_register(
        &mut self,
        register: Register,
        val: u8,
    ) -> Result<(), Bmp390Error<I2C::Error>> {
        self.i2c
            .lock()
            .unwrap()
            .write(self.address as u8, &[register.address(), val])
            .map_err(Bmp390Error::I2c)?;

        Ok(())
    }

    pub fn read_u8(&mut self, register: Register) -> Result<u8, Bmp390Error<I2C::Error>> {
        let mut bytes = [0; 1];
        self.read_register(register, &mut bytes)?;
        Ok(u8::from_le_bytes(bytes))
    }

    pub fn read_i8(&mut self, register: Register) -> Result<i8, Bmp390Error<I2C::Error>> {
        let mut bytes = [0; 1];
        self.read_register(register, &mut bytes)?;
        Ok(i8::from_le_bytes(bytes))
    }

    pub fn read_u16(&mut self, register: Register) -> Result<u16, Bmp390Error<I2C::Error>> {
        let mut bytes = [0; 2];
        self.read_register(register, &mut bytes)?;
        Ok(u16::from_le_bytes(bytes))
    }

    pub fn read_i16(&mut self, register: Register) -> Result<i16, Bmp390Error<I2C::Error>> {
        let mut bytes = [0; 2];
        self.read_register(register, &mut bytes)?;
        Ok(i16::from_le_bytes(bytes))
    }

    pub fn read_u24(&mut self, register: Register) -> Result<u32, Bmp390Error<I2C::Error>> {
        let mut bytes = [0; 4];
        self.read_register(register, &mut bytes[..3])?;
        Ok(u32::from_le_bytes(bytes))
    }

    pub fn read_register(
        &mut self,
        register: Register,
        data: &mut [u8],
    ) -> Result<(), Bmp390Error<I2C::Error>> {
        self.i2c
            .lock()
            .unwrap()
            .write_read(self.address as u8, &[register.address()], data)
            .map_err(Bmp390Error::I2c)?;

        Ok(())
    }

    pub fn read_temperature(&mut self) -> Result<f64, Bmp390Error<I2C::Error>> {
        let te = self.read_u24(Register::TemperatureData0)?;
        Ok(Self::temperature_compensation(
            te,
            self.cs.as_ref().unwrap(),
        ))
    }

    pub fn read_pressure(&mut self, temp: f64) -> Result<f64, Bmp390Error<I2C::Error>> {
        let pr = self.read_u24(Register::PressureData0)?;
        Ok(Self::pressure_compensation(pr, temp, &self.cs.unwrap()))
    }

    pub fn soft_reset(&mut self) -> Result<(), Bmp390Error<I2C::Error>> {
        self.write_register(Register::Cmd, 0xb6)
    }

    fn temperature_compensation(temp: u32, cs: &CalibrationStruct) -> f64 {
        let temp = temp as f64;

        let pd1 = temp - cs.t1;
        let pd2 = pd1 * cs.t2;

        let pd1 = pd1 as f64;

        pd2 + (pd1 * pd1) * cs.t3
    }

    fn pressure_compensation(pressure: u32, temp: f64, cs: &CalibrationStruct) -> f64 {
        let pressure = pressure as f64;
        let pressure2 = pressure * pressure;
        let pressure3 = pressure2 * pressure;

        let temp2 = temp * temp;
        let temp3 = temp2 * temp;

        let pd1 = cs.p6 * temp;
        let pd2 = cs.p7 * temp2;
        let pd3 = cs.p8 * temp3;
        let po1 = cs.p5 + pd1 + pd2 + pd3;

        let pd1 = cs.p2 * temp;
        let pd2 = cs.p3 * temp2;
        let pd3 = cs.p4 * temp3;
        let po2 = pressure * (cs.p1 + pd1 + pd2 + pd3);

        let pd1 = pressure2;
        let pd2 = cs.p9 + cs.p10 * temp;
        let pd3 = pd1 * pd2;
        let pd4 = pd3 + pressure3 * cs.p11;

        po1 + po2 + pd4
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum PwrCtrl {
    Sleep { press_en: bool, temp_en: bool } = 0b00000000,
    Forced { press_en: bool, temp_en: bool } = 0b00010000,
    Normal { press_en: bool, temp_en: bool } = 0b00110000,
}

impl PwrCtrl {
    fn discriminant(&self) -> u8 {
        unsafe { *(self as *const Self as *const u8) }
    }

    pub fn value(&self) -> u8 {
        let (press_en, temp_en) = match self {
            PwrCtrl::Sleep { press_en, temp_en } => (press_en, temp_en),
            PwrCtrl::Forced { press_en, temp_en } => (press_en, temp_en),
            PwrCtrl::Normal { press_en, temp_en } => (press_en, temp_en),
        };

        let press_bit = if *press_en { 0b01 } else { 0 };
        let temp_bit = if *temp_en { 0b10 } else { 0 };

        let pwr = self.discriminant();

        pwr | press_bit | temp_bit
    }
}

#[derive(Clone, Copy)]
pub enum Register {
    ChipId = 0x00,
    RevId = 0x01,
    ErrReg = 0x02,
    Status = 0x03,
    PressureData0 = 0x04,
    PressureData1 = 0x05,
    PressureData2 = 0x06,
    TemperatureData0 = 0x07,
    TemperatureData1 = 0x08,
    TemperatureData2 = 0x09,
    SensorTime0 = 0x0C,
    SensorTime1 = 0x0D,
    SensorTime2 = 0x0E,
    Event = 0x10,
    IntStatus = 0x11,
    FifoLength0 = 0x12,
    FifoLength1 = 0x13,
    FifoData = 0x14,
    FifoWatermark0 = 0x15,
    FifoWatermark1 = 0x16,
    FifoConfig1 = 0x17,
    FifoConfig2 = 0x18,
    IntCtrl = 0x19,
    IfConf = 0x1A,
    PwrCtrl = 0x1B,
    Osr = 0x1C,
    Odr = 0x1D,
    Config = 0x1F,
    NvmParT1 = 0x31,
    NvmParT2 = 0x33,
    NvmParT3 = 0x35,
    NvmParP1 = 0x36,
    NvmParP2 = 0x38,
    NvmParP3 = 0x3A,
    NvmParP4 = 0x3B,
    NvmParP5 = 0x3C,
    NvmParP6 = 0x3E,
    NvmParP7 = 0x40,
    NvmParP8 = 0x41,
    NvmParP9 = 0x42,
    NvmParP10 = 0x44,
    NvmParP11 = 0x45,
    Cmd = 0x7E,
}

#[derive(Clone, Copy)]
pub enum OdrSel {
    Odr200 = 0x00,
    Odr100 = 0x01,
    Odr50 = 0x02,
    Odr25 = 0x03,
    Odr12p5 = 0x04,
    Odr6p25 = 0x05,
    Odr3p1 = 0x06,
    Odr1p5 = 0x07,
    Odr0p78 = 0x08,
    Odr0p39 = 0x09,
    Odr0p2 = 0x0A,
    Odr0p1 = 0x0B,
    Odr0p05 = 0x0C,
    Odr0p02 = 0x0D,
    Odr0p01 = 0x0E,
    Odr0p006 = 0x0F,
    Odr0p003 = 0x10,
    Odr0p0015 = 0x11,
}

#[derive(Copy, Clone)]
pub enum OsrPress {
    x1 = 0b000,
    x2 = 0b001,
    x4 = 0b010,
    x8 = 0b011,
    x16 = 0b100,
    x32 = 0b101,
}

#[derive(Copy, Clone)]
pub enum OsrTemp {
    x1 = 0b000000,
    x2 = 0b001000,
    x4 = 0b010000,
    x8 = 0b011000,
    x16 = 0b100000,
    x32 = 0b101000,
}

#[derive(Copy, Clone)]
pub enum Osr {
    Select(OsrTemp, OsrPress),
}

impl Osr {
    pub fn value(&self) -> u8 {
        match *self {
            Osr::Select(ref osr_temp, ref osr_press) => *osr_temp as u8 | *osr_press as u8,
        }
    }
}

impl Register {
    fn address(&self) -> u8 {
        *self as u8
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CalibrationStruct {
    pub t1: f64,
    pub t2: f64,
    pub t3: f64,
    pub p1: f64,
    pub p2: f64,
    pub p3: f64,
    pub p4: f64,
    pub p5: f64,
    pub p6: f64,
    pub p7: f64,
    pub p8: f64,
    pub p9: f64,
    pub p10: f64,
    pub p11: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct RawCalibrationStruct {
    pub t1: u16,
    pub t2: u16,
    pub t3: i8,
    pub p1: i16,
    pub p2: i16,
    pub p3: i8,
    pub p4: i8,
    pub p5: u16,
    pub p6: u16,
    pub p7: i8,
    pub p8: i8,
    pub p9: i16,
    pub p10: i8,
    pub p11: i8,
}

impl RawCalibrationStruct {
    pub fn to_coefficients(&self) -> CalibrationStruct {
        CalibrationStruct {
            t1: self.t1 as f64 * 256_f64,
            t2: self.t2 as f64 / 30_f64.exp2(),
            t3: self.t3 as f64 / 48_f64.exp2(),
            p1: (self.p1 as f64 - 14_f64.exp2()) / 20_f64.exp2(),

            p2: (self.p2 as f64 - 14_f64.exp2()) / 29_f64.exp2(),
            p3: self.p3 as f64 / 32_f64.exp2(),
            p4: self.p4 as f64 / 37_f64.exp2(),
            p5: self.p5 as f64 * 8_f64,
            p6: self.p6 as f64 / 6_f64.exp2(),
            p7: self.p7 as f64 / 8_f64.exp2(),
            p8: self.p8 as f64 / 15_f64.exp2(),
            p9: self.p9 as f64 / 48_f64.exp2(),
            p10: self.p10 as f64 / 48_f64.exp2(),
            p11: self.p11 as f64 / 65_f64.exp2(),
        }
    }
}
