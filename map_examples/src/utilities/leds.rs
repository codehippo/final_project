use core::array;

use approx::abs_diff_eq;
use arrayvec::ArrayVec;
use bitvec::prelude::Msb0;
use bitvec::view::BitView;
use cortex_m::delay::Delay;
use embedded_time::duration::Milliseconds;
use embedded_time::fixed_point::FixedPoint;
use libm::{fabsf, roundf};
use ordered_float::OrderedFloat;
use stm32f4xx_hal::gpio::{Output, PartiallyErasedPin, Pin, PushPull};
use stm32f4xx_hal::pac::{SPI1, TIM1};
use stm32f4xx_hal::spi::{Spi, Spi1};
use stm32f4xx_hal::timer::{Polarity, PwmChannel};

use crate::utilities::colour::SimpleColour;

pub const ALTERNATE_FUNCTION_TIM1_2: u8 = 1;

const R_CHANNEL: usize = 0;
const G_CHANNEL: usize = 1;
const B_CHANNEL: usize = 2;

pub struct RgbBlinkyEvent {
    duration: Milliseconds,
    colour: SimpleColour
}

impl RgbBlinkyEvent {
    pub const fn new(duration: Milliseconds, colour: SimpleColour) -> Self {
        Self {
            duration,
            colour
        }
    }
}

pub struct RgbBlinkyPattern<const LENGTH: usize> {
    pattern: [RgbBlinkyEvent; LENGTH]
}

impl<const LENGTH: usize> RgbBlinkyPattern<LENGTH> {
    pub const fn new(pattern: [RgbBlinkyEvent; LENGTH]) -> Self {
        Self {
            pattern
        }
    }

    pub fn play(&self, delay: &mut Delay, rgb_led: &mut RgbLed) {
        self.pattern.iter().for_each(|blinky_event| {
            rgb_led.turn_rgb_channels(blinky_event.colour.as_array());
            delay.delay_ms(blinky_event.duration.integer());
        });
    }
}

#[derive(Clone, Copy)]
pub struct Brightness<const LEVEL_COUNT: usize> {
    level_index: usize,
    pub levels: [f32; LEVEL_COUNT],
}

impl<const LEVEL_COUNT: usize> Brightness<LEVEL_COUNT> {
    pub fn new() -> Result<Self, &'static str> {
        if LEVEL_COUNT > u16::MAX as usize {
            Err("Supplied level count is greater than u16::MAX. Supply smaller number.")
        } else {
            let levels: [f32; LEVEL_COUNT] =
                array::from_fn(|num| num as f32).map(|num| num / (LEVEL_COUNT - 1) as f32);
            Ok(Self {
                level_index: LEVEL_COUNT - 1,
                levels,
            })
        }
    }

    pub fn lower(&mut self) {
        if self.level_index > 0 {
            self.level_index -= 1;
        }
    }

    pub fn higher(&mut self) {
        if self.level_index < LEVEL_COUNT - 1 {
            self.level_index += 1;
        }
    }

    pub fn get_level(&self) -> f32 {
        self.levels[self.level_index]
    }

    pub fn set_level(&mut self, level_to_set: f32) -> Result<(), &str> {
        match self
            .levels
            .iter()
            .position(|&level| abs_diff_eq!(level, level_to_set))
        {
            None => {
                match self
                    .levels
                    .map(|level| fabsf(level - level_to_set))
                    .map(OrderedFloat)
                    .iter()
                    .enumerate()
                    .min_by_key(|&(_, level_distance)| level_distance)
                    .map(|(index, _)| index)
                {
                    None => Err("Could not find the closest brightness value to pick."),
                    Some(index) => {
                        self.level_index = index;
                        Ok(())
                    }
                }
            }
            Some(index) => {
                self.level_index = index;
                Ok(())
            }
        }
    }

    pub fn set_min(&mut self) {
        self.level_index = 0;
    }

    pub fn set_max(&mut self) {
        self.level_index = LEVEL_COUNT - 1;
    }
}

pub struct PwmRgbLed<'a, const BRIGHTNESS_LEVEL_COUNT: usize> {
    r_channel: &'a mut PwmChannel<TIM1, 2, true>,
    g_channel: &'a mut PwmChannel<TIM1, 1, true>,
    b_channel: &'a mut PwmChannel<TIM1, 0, true>,
}

impl<'a, const BRIGHTNESS_LEVEL_COUNT: usize> PwmRgbLed<'a, BRIGHTNESS_LEVEL_COUNT> {
    pub fn new(
        channels: &'a mut (
            PwmChannel<TIM1, 0, true>,
            PwmChannel<TIM1, 1, true>,
            PwmChannel<TIM1, 2, true>,
        ),
    ) -> Self {
        let (b_channel, g_channel, r_channel) = channels;

        Self {
            r_channel,
            g_channel,
            b_channel,
        }
    }

    fn enable_channels(&mut self) {
        self.r_channel
            .set_complementary_polarity(Polarity::ActiveHigh);
        self.r_channel.enable_complementary();

        self.g_channel
            .set_complementary_polarity(Polarity::ActiveHigh);
        self.g_channel.enable_complementary();

        self.b_channel
            .set_complementary_polarity(Polarity::ActiveHigh);
        self.b_channel.enable_complementary();
    }

    pub fn set_rgb_channels(&mut self, rgb_channel_values: [u8; 3], brightness: f32) {
        let max_duties = [
            self.r_channel.get_max_duty(),
            self.g_channel.get_max_duty(),
            self.b_channel.get_max_duty(),
        ]
            .map(|duty| duty as f32);

        let rgb_channel_ratios = rgb_channel_values.map(|num| num as f32 / (u8::MAX as f32));
        let duties: ArrayVec<f32, 3> = max_duties
            .iter()
            .zip(rgb_channel_ratios.iter())
            .map(|(&max_duty, &channel_value)| channel_value * max_duty as f32)
            .map(|duty| duty * brightness)
            .collect();

        self.r_channel
            .set_duty(roundf(duties[R_CHANNEL]) as u16);
        self.g_channel
            .set_duty(roundf(duties[G_CHANNEL]) as u16);
        self.b_channel
            .set_duty(roundf(duties[B_CHANNEL]) as u16);

        self.enable_channels();
    }
}

pub struct RgbLed {
    r_pin: Pin<'B', 15, Output>,
    g_pin: Pin<'B', 14, Output>,
    b_pin: Pin<'B', 13, Output>,
}

impl RgbLed {
    pub fn new(
        r_pin: Pin<'B', 15, Output>,
        g_pin: Pin<'B', 14, Output>,
        b_pin: Pin<'B', 13, Output>,
    ) -> Self {
        Self {
            r_pin,
            g_pin,
            b_pin,
        }
    }

    pub fn turn_off_all(&mut self) {
        self.r_pin.set_low();
        self.g_pin.set_low();
        self.b_pin.set_low();
    }

    pub fn turn_on_all(&mut self) {
        self.r_pin.set_high();
        self.g_pin.set_high();
        self.b_pin.set_high();
    }

    pub fn turn_rgb_channels(&mut self, rgb_channels: [bool; 3]) {
        if rgb_channels[R_CHANNEL] {
            self.r_pin.set_high();
        } else {
            self.r_pin.set_low();
        }

        if rgb_channels[G_CHANNEL] {
            self.g_pin.set_high();
        } else {
            self.g_pin.set_low();
        }

        if rgb_channels[B_CHANNEL] {
            self.b_pin.set_high();
        } else {
            self.b_pin.set_low();
        }
    }
}

pub struct DirectLeds {
    led_pins: [PartiallyErasedPin<'C', Output<PushPull>>; 8],
}

impl DirectLeds {
    pub fn new(led_pins: [PartiallyErasedPin<'C', Output<PushPull>>; 8]) -> Self {
        Self { led_pins }
    }

    pub fn apply_pattern_to_leds(&mut self, pattern: u8) {
        let pattern_bits = pattern.view_bits::<Msb0>();

        for (bit_number, pattern_bit) in pattern_bits.into_iter().enumerate() {
            if *pattern_bit {
                self.led_pins[bit_number].set_high();
            } else {
                self.led_pins[bit_number].set_low();
            }
        }
    }
}

pub struct SpiShiftRegisterLeds {
    spi: Spi<SPI1>,
    latch_pin: PartiallyErasedPin<'A', Output>,
}

impl SpiShiftRegisterLeds {
    pub fn new(spi: Spi1, latch_pin: PartiallyErasedPin<'A', Output>) -> Self {
        Self { spi, latch_pin }
    }

    pub fn apply_pattern_to_leds(&mut self, pattern: u8) {
        self.latch_pin.set_low();

        self.spi.write(&[pattern]).unwrap();

        self.latch_pin.set_high();
        self.latch_pin.set_low();
    }
}

pub struct LedProgress<const TOTAL_LED_COUNT: usize> {
    pub state: f32,
}

impl<const TOTAL_LED_COUNT: usize> LedProgress<TOTAL_LED_COUNT> {
    pub fn new(led_count: Option<u8>) -> Result<Self, &'static str> {
        if TOTAL_LED_COUNT > u8::MAX as usize {
            Err("Total LED count too big. Limited by u8::MAX.")
        } else if led_count.unwrap_or(0) as usize > TOTAL_LED_COUNT {
            Err("LED count can't be higher than total LED count.")
        } else {
            Ok(Self {
                state: led_count.unwrap_or(0) as f32 / (TOTAL_LED_COUNT as f32),
            })
        }
    }

    pub fn set_state(&mut self, ratio_to_set: f32) -> Result<(), &'static str> {
        if ratio_to_set > 1_f32 {
            return Err("Ratio cannot be greater than 1.");
        }

        if abs_diff_eq!(ratio_to_set, 0_f32) {
            return {
                self.state = 0_f32;
                Ok(())
            };
        }

        let available_states: [f32; TOTAL_LED_COUNT] =
            array::from_fn(|num| (num + 1) as u16).map(|num| num as f32 / (TOTAL_LED_COUNT as f32));

        match available_states
            .iter()
            .position(|&ratio| abs_diff_eq!(ratio, ratio_to_set))
        {
            None => {
                match available_states
                    .map(|ratio| fabsf(ratio - ratio_to_set))
                    .map(OrderedFloat)
                    .iter()
                    .enumerate()
                    .min_by_key(|&(_, ratio_distance)| ratio_distance)
                    .map(|(index, _)| index)
                {
                    None => Err("Could not find the closest LED ratio to pick."),
                    Some(index) => {
                        self.state = available_states[index];
                        Ok(())
                    }
                }
            }
            Some(index) => {
                self.state = available_states[index];
                Ok(())
            }
        }
    }

    pub fn get_number_of_leds_to_turn_on(&self) -> u8 {
        roundf(self.state * TOTAL_LED_COUNT as f32) as u8
    }
}

pub struct UnifiedLeds<'a> {
    direct_leds: &'a mut DirectLeds,
    spi_shift_register_leds: &'a mut SpiShiftRegisterLeds,
}

impl<'a> UnifiedLeds<'a> {
    pub fn new(
        direct_leds: &'a mut DirectLeds,
        spi_shift_register_leds: &'a mut SpiShiftRegisterLeds,
    ) -> Self {
        Self {
            direct_leds,
            spi_shift_register_leds,
        }
    }

    pub fn apply_pattern_to_all_leds(&mut self, pattern: u16) {
        let pattern_first_half = (pattern >> 8) as u8;
        let pattern_second_half = ((pattern & 0xff) as u8).reverse_bits();

        self.direct_leds.apply_pattern_to_leds(pattern_first_half);
        self.spi_shift_register_leds
            .apply_pattern_to_leds(pattern_second_half);
    }

    pub fn apply_pattern_to_direct_leds(&mut self, pattern: u8) {
        self.direct_leds.apply_pattern_to_leds(pattern);
    }

    pub fn apply_pattern_to_spi_leds(&mut self, pattern: u8) {
        self.spi_shift_register_leds
            .apply_pattern_to_leds(pattern.reverse_bits());
    }

    pub fn apply_progress_to_all_leds(&mut self, led_progress: &LedProgress<16>) {
        self.apply_pattern_to_all_leds(
            u16::MAX
                .checked_shl((16 - led_progress.get_number_of_leds_to_turn_on()) as u32)
                .unwrap_or(0),
        );
    }

    pub fn apply_progress_to_direct_leds(&mut self, led_progress: &LedProgress<8>) {
        self.apply_pattern_to_direct_leds(
            u8::MAX
                .checked_shl((8 - led_progress.get_number_of_leds_to_turn_on()) as u32)
                .unwrap_or(0),
        );
    }

    pub fn apply_progress_to_spi_leds(&mut self, led_progress: &LedProgress<8>) {
        self.apply_pattern_to_spi_leds(
            u8::MAX
                .checked_shl((8 - led_progress.get_number_of_leds_to_turn_on()) as u32)
                .unwrap_or(0),
        );
    }

    pub fn clear(&mut self) {
        self.apply_pattern_to_all_leds(0);
    }
}
