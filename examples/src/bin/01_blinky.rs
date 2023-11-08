#![no_main]
#![no_std]

use cortex_m::delay::Delay;
use cortex_m::peripheral::Peripherals as CorePeripherals;
use cortex_m_rt::entry;
use stm32f4xx_hal::pac::Peripherals;
use stm32f4xx_hal::prelude::*;

use map_examples as _;

const DELAY_IN_MS: u32 = 25;

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let ahb_frequency = clocks.hclk().raw();
    let gpioa = dp.GPIOA.split();

    let mut delay = Delay::new(cp.SYST, ahb_frequency);

    let mut led = gpioa.pa5.into_push_pull_output();

    loop {
        led.toggle();

        delay.delay_ms(DELAY_IN_MS);
    }
}