#![no_main]
#![no_std]

use cortex_m::delay::Delay;
use cortex_m::peripheral::Peripherals as CorePeripherals;
use cortex_m_rt::entry;
use embedded_time::duration::Extensions;
use stm32f4xx_hal::pac::Peripherals;
use stm32f4xx_hal::prelude::*;

use map_examples as _;
use map_examples::utilities::colour::SimpleColour;
use map_examples::utilities::leds;
use map_examples::utilities::leds::{RgbBlinkyEvent, RgbBlinkyPattern};

#[entry]
fn main() -> ! {
    let police_car_pattern: RgbBlinkyPattern<16> = RgbBlinkyPattern::new(
        [
            RgbBlinkyEvent::new(500.milliseconds(),
                                SimpleColour::new(true, false, false)),
            RgbBlinkyEvent::new(500.milliseconds(),
                                SimpleColour::new(false, false, true)),
            RgbBlinkyEvent::new(500.milliseconds(),
                                SimpleColour::new(true, false, false)),
            RgbBlinkyEvent::new(500.milliseconds(),
                                SimpleColour::new(false, false, true)),
            RgbBlinkyEvent::new(250.milliseconds(),
                                SimpleColour::new(true, false, false)),
            RgbBlinkyEvent::new(250.milliseconds(),
                                SimpleColour::new(false, false, true)),
            RgbBlinkyEvent::new(250.milliseconds(),
                                SimpleColour::new(true, false, false)),
            RgbBlinkyEvent::new(250.milliseconds(),
                                SimpleColour::new(false, false, true)),
            RgbBlinkyEvent::new(250.milliseconds(),
                                SimpleColour::new(true, false, false)),
            RgbBlinkyEvent::new(250.milliseconds(),
                                SimpleColour::new(false, false, true)),
            RgbBlinkyEvent::new(250.milliseconds(),
                                SimpleColour::new(true, false, false)),
            RgbBlinkyEvent::new(250.milliseconds(),
                                SimpleColour::new(false, false, true)),
            RgbBlinkyEvent::new(500.milliseconds(),
                                SimpleColour::new(true, false, false)),
            RgbBlinkyEvent::new(500.milliseconds(),
                                SimpleColour::new(false, false, true)),
            RgbBlinkyEvent::new(500.milliseconds(),
                                SimpleColour::new(true, false, false)),
            RgbBlinkyEvent::new(500.milliseconds(),
                                SimpleColour::new(false, false, true))
        ]
    );

    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let ahb_frequency = clocks.hclk().raw();

    let gpiob = dp.GPIOB.split();

    let mut rgb_led = leds::RgbLed::new(
        gpiob.pb15.into_push_pull_output(),
        gpiob.pb14.into_push_pull_output(),
        gpiob.pb13.into_push_pull_output(),
    );

    let mut delay = Delay::new(cp.SYST, ahb_frequency);

    loop {
        police_car_pattern.play(&mut delay, &mut rgb_led);
    }
}
