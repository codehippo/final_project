#![no_main]
#![no_std]

use cortex_m::delay::Delay;
use cortex_m::peripheral::Peripherals as CorePeripherals;
use cortex_m_rt::entry;
use stm32f4xx_hal::gpio::{Output, PartiallyErasedPin};
use stm32f4xx_hal::hal::spi::MODE_0;
use stm32f4xx_hal::pac::{Peripherals, SPI1};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::spi::{NoMiso, Spi, Spi1};

use map_examples as _;

struct ShiftRegister {
    spi: Spi<SPI1>,
    latch_pin: PartiallyErasedPin<'A', Output>,
}

impl ShiftRegister {
    fn new(spi: Spi1, latch_pin: PartiallyErasedPin<'A', Output>) -> Self {
        Self { spi, latch_pin }
    }

    fn apply_pattern_to_leds(&mut self, pattern: u8) {
        self.latch_pin.set_low();

        self.spi.write(&[pattern]).unwrap();

        self.latch_pin.set_high();
        self.latch_pin.set_low();
    }
}

const DELAY_IN_MS: u32 = 200;

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();
    
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let ahb_frequency = clocks.hclk().raw();
    
    let gpioa = dp.GPIOA.split();

    let clock_pin = gpioa.pa5.into_alternate();
    let serial_data_pin = gpioa.pa7.into_alternate();
    let latch_pin = gpioa.pa8.into_push_pull_output();

    let mut shift_register: ShiftRegister = ShiftRegister::new(
        dp.SPI1.spi(
            (clock_pin, NoMiso::new(), serial_data_pin),
            MODE_0,
            2.MHz(),
            &clocks,
        ),
        latch_pin.erase_number(),
    );
    
    let mut delay = Delay::new(cp.SYST, ahb_frequency);
    
    loop {
        let mut pattern = 0b1010_1010_u8;
        shift_register.apply_pattern_to_leds(pattern);

        delay.delay_ms(DELAY_IN_MS);

        pattern = 0b0101_0101_u8;
        shift_register.apply_pattern_to_leds(pattern);

        delay.delay_ms(DELAY_IN_MS);
    }
}
