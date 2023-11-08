#![no_main]
#![no_std]

use cortex_m::delay::Delay;
use cortex_m::peripheral::Peripherals as CorePeripherals;
use cortex_m_rt::entry;
use stm32f4xx_hal::hal::spi::MODE_0;
use stm32f4xx_hal::pac::Peripherals;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::spi::NoMiso;

use map_examples as _;
use map_examples::utilities::leds;

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let ahb_frequency = clocks.hclk().raw();

    let mut delay = Delay::new(cp.SYST, ahb_frequency);

    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    let clock_pin = gpioa.pa5.into_alternate();
    let serial_data_pin = gpioa.pa7.into_alternate();
    let latch_pin = gpioa.pa8.into_push_pull_output();

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

    let mut spi_shift_register_leds = leds::SpiShiftRegisterLeds::new(
        dp.SPI1.spi(
            (clock_pin, NoMiso::new(), serial_data_pin),
            MODE_0,
            2.MHz(),
            &clocks,
        ),
        latch_pin.erase_number(),
    );

    let mut unified_leds = leds::UnifiedLeds::new(&mut direct_leds, &mut spi_shift_register_leds);

    loop {
        let mut pattern = 0b1111_0000_1010_0011_u16;
        unified_leds.apply_pattern_to_all_leds(pattern);

        delay.delay_ms(200);

        pattern = 0_u16;
        unified_leds.apply_pattern_to_all_leds(pattern);

        delay.delay_ms(200);
    }
}
