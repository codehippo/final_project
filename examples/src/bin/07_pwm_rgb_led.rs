#![no_main]
#![no_std]

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use stm32f4xx_hal::adc::Adc;
use stm32f4xx_hal::adc::config::{AdcConfig, SampleTime};
use stm32f4xx_hal::gpio;
use stm32f4xx_hal::gpio::{Analog, Edge, gpioa, Input};
use stm32f4xx_hal::hal::spi::MODE_0;
use stm32f4xx_hal::pac::{ADC1, interrupt, Peripherals};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::spi::NoMiso;
use stm32f4xx_hal::timer::{Channel1, Channel2, Channel3};

use map_examples as _;
use map_examples::utilities::colour::{Colour, ColourChannel};
use map_examples::utilities::leds::{
    Brightness, DirectLeds, LedProgress, PwmRgbLed, SpiShiftRegisterLeds, UnifiedLeds,
};

const BRIGHTNESS_LEVEL_COUNT: usize = 256;
const COLOUR_DEPTH: usize = 8;
const ADC_12_BIT: u16 = 4_096;
const MODE_ORDER: [SettingMode; 3] = [SettingMode::Red, SettingMode::Green, SettingMode::Blue];

#[derive(Copy, Clone, PartialEq, Debug)]
enum SettingMode {
    Red,
    Green,
    Blue,
}

struct Mode {
    mode_index: usize,
    mode_order: [SettingMode; 3],
}

impl Mode {
    fn new() -> Self {
        Self {
            mode_index: 0,
            mode_order: MODE_ORDER,
        }
    }

    fn next(&mut self) {
        if self.mode_index < self.mode_order.len() - 1 {
            self.mode_index += 1;
        } else {
            self.mode_index = 0;
        }
    }

    fn get_mode(&self) -> SettingMode {
        self.mode_order[self.mode_index]
    }
}

type ButtonLeftPin = gpio::PB5<Input>;
type ButtonRightPin = gpio::PB4<Input>;
type UserButtonPin = gpio::PC13<Input>;
type AdcTrimpotPin = gpioa::PA0<Analog>;
type AdcTrimpot = Adc<ADC1>;

static G_USER_BUTTON: Mutex<RefCell<Option<UserButtonPin>>> = Mutex::new(RefCell::new(None));
static G_BUTTON_LEFT: Mutex<RefCell<Option<ButtonLeftPin>>> = Mutex::new(RefCell::new(None));
static G_BUTTON_RIGHT: Mutex<RefCell<Option<ButtonRightPin>>> = Mutex::new(RefCell::new(None));
static G_ADC_TRIMPOT_PIN: Mutex<RefCell<Option<AdcTrimpotPin>>> = Mutex::new(RefCell::new(None));
static G_ADC_TRIMPOT: Mutex<RefCell<Option<AdcTrimpot>>> = Mutex::new(RefCell::new(None));

static G_BRIGHTNESS: Mutex<RefCell<Option<Brightness<BRIGHTNESS_LEVEL_COUNT>>>> =
    Mutex::new(RefCell::new(None));
static G_COLOUR: Mutex<RefCell<Option<Colour<COLOUR_DEPTH>>>> = Mutex::new(RefCell::new(None));
static G_MODE: Mutex<RefCell<Option<Mode>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut syscfg = peripherals.SYSCFG.constrain();
    let rcc = peripherals.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(80.MHz()).freeze();

    let gpioa = peripherals.GPIOA.split();
    let gpiob = peripherals.GPIOB.split();
    let gpioc = peripherals.GPIOC.split();
    let gpioe = peripherals.GPIOE.split();

    let mut button_left = gpiob.pb5;
    button_left.make_interrupt_source(&mut syscfg);
    button_left.trigger_on_edge(&mut peripherals.EXTI, Edge::Rising);
    button_left.enable_interrupt(&mut peripherals.EXTI);

    let mut button_right = gpiob.pb4;
    button_right.make_interrupt_source(&mut syscfg);
    button_right.trigger_on_edge(&mut peripherals.EXTI, Edge::Rising);
    button_right.enable_interrupt(&mut peripherals.EXTI);

    let mut button_user = gpioc.pc13;
    button_user.make_interrupt_source(&mut syscfg);
    button_user.trigger_on_edge(&mut peripherals.EXTI, Edge::Rising);
    button_user.enable_interrupt(&mut peripherals.EXTI);

    unsafe {
        cortex_m::peripheral::NVIC::unmask(button_left.interrupt());
        cortex_m::peripheral::NVIC::unmask(button_right.interrupt());
        cortex_m::peripheral::NVIC::unmask(button_user.interrupt());
    }

    let mut direct_leds = DirectLeds::new([
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

    let mut spi_shift_register_leds = SpiShiftRegisterLeds::new(
        peripherals.SPI1.spi(
            (clock_pin, NoMiso::new(), serial_data_pin),
            MODE_0,
            2.MHz(),
            &clocks,
        ),
        latch_pin.erase_number(),
    );

    let mut unified_leds = UnifiedLeds::new(&mut direct_leds, &mut spi_shift_register_leds);

    let adc_pin = gpioa.pa0.into_analog();
    let mut adc_trimpot = Adc::adc1(
        peripherals.ADC1,
        true,
        AdcConfig::default()
    );
    let adc_value = adc_trimpot.convert(
        &adc_pin,
        SampleTime::Cycles_56
    );

    let r_channel_pwn_pin =
        Channel3::new(gpioe.pe13.into_alternate())
            .with_complementary(gpiob.pb15.into_alternate());
    let g_channel_pwn_pin =
        Channel2::new(gpioe.pe11.into_alternate())
            .with_complementary(gpiob.pb14.into_alternate());
    let b_channel_pwn_pin =
        Channel1::new(gpioe.pe9.into_alternate())
            .with_complementary(gpiob.pb13.into_alternate());

    let mut channels = peripherals
        .TIM1
        .pwm_hz(
            (b_channel_pwn_pin, g_channel_pwn_pin, r_channel_pwn_pin),
            25.kHz(),
            &clocks,
        )
        .split();

    let colour: Colour<8> = Colour::new(4, 4, 4).unwrap();
    let mut brightness: Brightness<BRIGHTNESS_LEVEL_COUNT> = Brightness::new().unwrap();
    brightness
        .set_level(adc_value as f32 / ADC_12_BIT as f32)
        .unwrap();

    let mut pwm_rgb_led: PwmRgbLed<BRIGHTNESS_LEVEL_COUNT> = PwmRgbLed::new(&mut channels);
    pwm_rgb_led.set_rgb_channels(
        colour.as_scaled_array(16).map(|num| num as u8),
        brightness.get_level(),
    );

    let mut led_progress: LedProgress<8> = LedProgress::new(None).unwrap();
    let mode = Mode::new();

    cortex_m::interrupt::free(|cs| {
        G_BUTTON_LEFT.borrow(cs).replace(Some(button_left));
        G_BUTTON_RIGHT.borrow(cs).replace(Some(button_right.into()));
        G_USER_BUTTON.borrow(cs).replace(Some(button_user));
        G_ADC_TRIMPOT_PIN.borrow(cs).replace(Some(adc_pin));
        G_ADC_TRIMPOT.borrow(cs).replace(Some(adc_trimpot));

        G_BRIGHTNESS.borrow(cs).replace(Some(brightness));
        G_MODE.borrow(cs).replace(Some(mode));
        G_COLOUR.borrow(cs).replace(Some(colour));
    });

    loop {
        let current_mode =
            cortex_m::interrupt::free(|cs|
                G_MODE.borrow(cs).borrow().as_ref().unwrap().get_mode()
            );

        update_brightness();
        let brightness = get_brightness();
        let colour = get_colour();
        let colour_as_array = colour.as_scaled_array(32).map(|num| num as u8);
        pwm_rgb_led.set_rgb_channels(colour_as_array, brightness);

        match current_mode {
            SettingMode::Red => {
                unified_leds.apply_pattern_to_direct_leds(0b1000_0000);

                led_progress.set_state(colour.get_r_as_float()).unwrap();
                unified_leds.apply_progress_to_spi_leds(&led_progress);
            }
            SettingMode::Green => {
                unified_leds.apply_pattern_to_direct_leds(0b0100_0000);

                led_progress.set_state(colour.get_g_as_float()).unwrap();
                unified_leds.apply_progress_to_spi_leds(&led_progress);
            }
            SettingMode::Blue => {
                unified_leds.apply_pattern_to_direct_leds(0b0010_0000);

                led_progress.set_state(colour.get_b_as_float()).unwrap();
                unified_leds.apply_progress_to_spi_leds(&led_progress);
            }
        }
    }
}

fn update_brightness() {
    cortex_m::interrupt::free(|cs| {
        let adc_trimpot_pin_ref = G_ADC_TRIMPOT_PIN.borrow(cs).borrow();
        let adc_trimpot_pin = adc_trimpot_pin_ref.as_ref().unwrap();

        let mut adc_trimpot_ref = G_ADC_TRIMPOT.borrow(cs).borrow_mut();
        let adc_trimpot = adc_trimpot_ref.as_mut().unwrap();
        let adc_value = adc_trimpot.convert(adc_trimpot_pin, SampleTime::Cycles_28);

        let mut brightness_ref = G_BRIGHTNESS.borrow(cs).borrow_mut();
        let brightness = brightness_ref.as_mut().unwrap();
        brightness
            .set_level(adc_value as f32 / ADC_12_BIT as f32)
            .unwrap();
    });
}

fn get_brightness() -> f32 {
    cortex_m::interrupt::free(|cs| {
        let brightness_ref = G_BRIGHTNESS.borrow(cs).borrow();
        let brightness = brightness_ref.as_ref().unwrap();
        brightness.get_level()
    })
}

fn get_colour() -> Colour<COLOUR_DEPTH> {
    cortex_m::interrupt::free(|cs| {
        let colour = G_COLOUR.borrow(cs).borrow();
        let current_colour = colour.as_ref().unwrap();
        *current_colour
    })
}

#[interrupt]
fn EXTI4() {
    cortex_m::interrupt::free(|cs| {
        match G_MODE.borrow(cs).borrow().as_ref().unwrap().get_mode() {
            SettingMode::Red => {
                G_COLOUR
                    .borrow(cs)
                    .borrow_mut()
                    .as_mut()
                    .unwrap()
                    .inc_channel(&ColourChannel::Red);
            }
            SettingMode::Green => {
                G_COLOUR
                    .borrow(cs)
                    .borrow_mut()
                    .as_mut()
                    .unwrap()
                    .inc_channel(&ColourChannel::Green);
            }
            SettingMode::Blue => {
                G_COLOUR
                    .borrow(cs)
                    .borrow_mut()
                    .as_mut()
                    .unwrap()
                    .inc_channel(&ColourChannel::Blue);
            }
        }

        let mut button_right = G_BUTTON_RIGHT.borrow(cs).borrow_mut();
        button_right.as_mut().unwrap().clear_interrupt_pending_bit();
    });
}

#[interrupt]
fn EXTI9_5() {
    cortex_m::interrupt::free(|cs| {
        match G_MODE.borrow(cs).borrow().as_ref().unwrap().get_mode() {
            SettingMode::Red => {
                G_COLOUR
                    .borrow(cs)
                    .borrow_mut()
                    .as_mut()
                    .unwrap()
                    .dec_channel(&ColourChannel::Red);
            }
            SettingMode::Green => {
                G_COLOUR
                    .borrow(cs)
                    .borrow_mut()
                    .as_mut()
                    .unwrap()
                    .dec_channel(&ColourChannel::Green);
            }
            SettingMode::Blue => {
                G_COLOUR
                    .borrow(cs)
                    .borrow_mut()
                    .as_mut()
                    .unwrap()
                    .dec_channel(&ColourChannel::Blue);
            }
        }

        let mut button_left = G_BUTTON_LEFT.borrow(cs).borrow_mut();
        button_left.as_mut().unwrap().clear_interrupt_pending_bit();
    });
}

#[interrupt]
fn EXTI15_10() {
    cortex_m::interrupt::free(|cs| {
        G_MODE.borrow(cs).borrow_mut().as_mut().unwrap().next();

        let mut button_user = G_USER_BUTTON.borrow(cs).borrow_mut();
        button_user.as_mut().unwrap().clear_interrupt_pending_bit();
    });
}
