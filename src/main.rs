use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use esp_idf_hal::{
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    prelude::*,
};

mod bmp390;

const SEA_LEVEL_ATMOSPHERES: f64 = 101600.00_f64;

fn altitude(pressure: f64) -> f64 {
    (1_f64 - (pressure / SEA_LEVEL_ATMOSPHERES).powf(0.190284_f64)) * 145366.45_f64
}

fn main() {
    esp_idf_svc::sys::link_patches();

    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let sda = peripherals.pins.gpio21;
    let scl = peripherals.pins.gpio22;

    let config = I2cConfig::new().baudrate(400.kHz().into());

    let i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &config).unwrap();

    let i2c_mu = Arc::new(Mutex::new(i2c));

    let mut sensor = bmp390::BMP390::new(i2c_mu.clone(), bmp390::DeviceAddr::AD0).unwrap();

    let chip_id = sensor.read_device_id_register().unwrap();
    let status = sensor.read_u8(bmp390::Register::Status).unwrap();

    log::info!("chip id: {}", chip_id);
    log::info!("status: {}", status);

    let cs = sensor.read_calibration_struct().unwrap().to_coefficients();

    log::info!("calibration struct: {:?}", cs);

    sensor
        .write_register(
            bmp390::Register::Osr,
            bmp390::Osr::Select(bmp390::OsrTemp::x32, bmp390::OsrPress::x2).value(),
        )
        .unwrap();

    loop {
        sensor
            .write_register(
                bmp390::Register::PwrCtrl,
                bmp390::PwrCtrl::Forced {
                    press_en: true,
                    temp_en: true,
                }
                .value(),
            )
            .unwrap();

        std::thread::sleep(Duration::from_millis(500));

        log::info!("status: {}", status);

        let compensated_temperature = sensor.read_temperature().unwrap();
        let compensated_pressure = sensor.read_pressure(compensated_temperature).unwrap();

        let alt = altitude(compensated_pressure);

        log::info!("temperature: {} C", compensated_temperature);
        log::info!("pressure: {} Pa", compensated_pressure);
        log::info!("altitude: {} ft", alt);

        std::thread::sleep(Duration::from_secs(1));
    }
}
