#![no_main]
#![no_std]

use bilge::prelude::*;
use cortex_m::delay::Delay;
use cortex_m::peripheral::Peripherals as CorePeripherals;
use cortex_m_rt::entry;
use defmt::println;
use libm::truncf;
use stm32f4xx_hal::hal::spi::MODE_0;
use stm32f4xx_hal::i2c::{I2c, Instance, Mode};
use stm32f4xx_hal::pac::Peripherals;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::spi::NoMiso;

use map_examples as _;
use map_examples::utilities::leds;

#[allow(non_camel_case_types)]
#[repr(u32)]
#[derive(Copy, Clone)]
enum Delays {
    SHORT = 10,
    MEDIUM = 50,
    LONG = 200,
    VERY_LONG = 1000,
}

#[allow(non_camel_case_types)]
#[repr(u8)]
#[derive(Copy, Clone)]
enum AssociatedConstants {
    CHIP_ID = 0xA4,
    POWER_ON_RESET = 0x40,
    EMPTY = 0x00,
}

#[allow(non_camel_case_types)]
#[repr(u8)]
#[derive(Copy, Clone)]
enum Registers {
    DEVICE = 0x6C,
    PRODUCER = 0x18,
    INTERRUPT = 0x06,
    MODE = 0x07,
    DCM = 0x15,
    POWER_ON_RESET = 0x1C,
    ANALOG_GAIN = 0x2B,
    X_LSB_ADDRESS = 0x0D,
}

#[bitsize(8)]
struct ModeRegisterValues {
    mode_state: ModeState,
    reserved: u5,
}

#[bitsize(3)]
#[derive(Debug, Clone, TryFromBits)]
#[allow(non_camel_case_types)]
enum ModeState {
    SLEEP = 0b000,
    CWAKE = 0b001,
    RESERVED = 0b010,
    STANDBY = 0b011,
}

impl Registers {
    fn addr(&self) -> u8 {
        *self as u8
    }
}

struct Accelerometer<'a, T: Instance> {
    i2c_instance: &'a mut I2c<T>,
    checked_device: Option<bool>,
    address: u8,
}

impl<'a, T: Instance> Accelerometer<'a, T> {
    fn new(i2c_instance: &'a mut I2c<T>) -> Self {
        Self {
            i2c_instance,
            checked_device: Some(false),
            address: 0,
        }
    }

    fn check_device(&mut self, delay: &mut Delay) {
        self.reset(delay);

        let mut buffer: [u8; 1] = [0; 1];

        self.i2c_instance.write_read(
            self.address,
            &[Registers::PRODUCER.addr()],
            &mut buffer,
        ).unwrap();

        if buffer[0] != AssociatedConstants::CHIP_ID as u8 {
            self.checked_device = None;
        } else {
            self.checked_device = Some(true);
        }
    }

    fn scan(&mut self, delay: &mut Delay) {
        for address in 0_u8..=u8::MAX {
            if self.i2c_instance.write(
                address,
                &[],
            ).is_ok() {
                self.address = address;
                break;
            };

            delay.delay_ms(Delays::SHORT as u32);
        }
    }

    fn reset(&mut self, delay: &mut Delay) {
        let mode_register_value = ModeRegisterValues::new(ModeState::STANDBY);

        // Standby mode
        self.i2c_instance.write(
            self.address,
            &[Registers::MODE.addr(), mode_register_value.value],
        ).unwrap_or(());

        delay.delay_ms(Delays::SHORT as u32);

        // Power-on-reset
        self.i2c_instance.write(
            self.address,
            &[Registers::POWER_ON_RESET.addr(), AssociatedConstants::POWER_ON_RESET as u8],
        ).unwrap_or(());

        delay.delay_ms(Delays::MEDIUM as u32);

        // Disable interrupt
        self.i2c_instance.write(
            self.address,
            &[Registers::INTERRUPT.addr(), AssociatedConstants::EMPTY as u8],
        ).unwrap_or(());

        delay.delay_ms(Delays::SHORT as u32);

        // 1.00x analog gain
        self.i2c_instance.write(
            self.address,
            &[Registers::ANALOG_GAIN.addr(), AssociatedConstants::EMPTY as u8],
        ).unwrap_or(());

        delay.delay_ms(Delays::SHORT as u32);

        // DCM disable
        self.i2c_instance.write(
            self.address,
            &[Registers::DCM.addr(), AssociatedConstants::EMPTY as u8],
        ).unwrap_or(());

        delay.delay_ms(Delays::MEDIUM as u32);
    }
}

const ACC_DIV: f32 = 32768.0;
const LED_POSITIONS_HALF: f32 = 3.5;
const ACC_RANGE: f32 = 2.0;

fn mc2float(accelerator_value: i16) -> f32 {
    (accelerator_value as f32 * ACC_RANGE) / ACC_DIV
}

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(80.MHz()).freeze();
    let ahb_frequency = clocks.hclk().raw();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let scl = gpiob.pb8.into_alternate_open_drain::<4>();
    let sda = gpiob.pb9.into_alternate_open_drain::<4>();

    let mut i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        Mode::standard(100.kHz()),
        &clocks,
    );

    let mut delay = Delay::new(cp.SYST, ahb_frequency);

    let mut accelerometer = Accelerometer::new(&mut i2c);
    accelerometer.scan(&mut delay);
    accelerometer.check_device(&mut delay);

    let active_mode = ModeRegisterValues::new(ModeState::CWAKE);

    accelerometer.i2c_instance.write(
        accelerometer.address,
        &[Registers::MODE.addr(), active_mode.value],
    ).unwrap();

    let mut direct_leds = leds::DirectLeds::new([
        gpioc.pc0.into_push_pull_output().erase_number(),
        gpioc.pc1.into_push_pull_output().erase_number(),
        gpioc.pc2.into_push_pull_output().erase_number(),
        gpioc.pc3.into_push_pull_output().erase_number(),
        gpioc.pc4.into_push_pull_output().erase_number(),
        gpioc.pc5.into_push_pull_output().erase_number(),
        gpioc.pc6.into_push_pull_output().erase_number(),
        gpioc.pc7.into_push_pull_output().erase_number(),
    ]);

    let clock_pin = gpioa.pa5.into_alternate();
    let serial_data_pin = gpioa.pa7.into_alternate();
    let latch_pin = gpioa.pa8.into_push_pull_output();

    let mut spi_shift_register_leds = leds::SpiShiftRegisterLeds::new(
        dp.SPI1.spi(
            (clock_pin, NoMiso::new(), serial_data_pin),
            MODE_0,
            2.MHz(),
            &clocks,
        ),
        latch_pin.erase_number(),
    );

    let mut buf: [u8; 6] = [0; 6];

    loop {
        let mut x_pattern: u8 = 0b00011000;
        let mut y_pattern: u8 = 0b00011000;

        accelerometer.i2c_instance.write_read(
            accelerometer.address,
            &[Registers::X_LSB_ADDRESS.addr()],
            &mut buf,
        ).unwrap();

        let fx = mc2float(i16::from_le_bytes([buf[0], buf[1]]));
        let fy = mc2float(i16::from_le_bytes([buf[2], buf[3]]));
        let fz = mc2float(i16::from_le_bytes([buf[4], buf[5]]));

        let x_leds_shift = truncf(fx * LED_POSITIONS_HALF) as i8;
        let y_leds_shift = truncf(fy * LED_POSITIONS_HALF) as i8;

        if x_leds_shift < 0 {
            x_pattern <<= -x_leds_shift as u8
        } else {
            x_pattern >>= x_leds_shift as u8
        }

        if y_leds_shift < 0 {
            y_pattern >>= -y_leds_shift as u8
        } else {
            y_pattern <<= y_leds_shift as u8
        }

        direct_leds.apply_pattern_to_leds(x_pattern);
        spi_shift_register_leds.apply_pattern_to_leds(y_pattern);

        delay.delay_ms(Delays::SHORT as u32);
    }
}
