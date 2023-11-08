#![no_main]
#![no_std]

use core::cell::{Cell, RefCell};

use bitvec::prelude::*;

use cortex_m::delay::Delay;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::Peripherals as CorePeripherals;
use cortex_m_rt::entry;
use stm32f4xx_hal::gpio;
use stm32f4xx_hal::gpio::{Edge, Input, PartiallyErasedPin, PushPull};
use stm32f4xx_hal::pac::{interrupt, Peripherals};
use stm32f4xx_hal::prelude::*;

use map_examples as _;
use map_examples::utilities::leds;

type ButtonLeftPin = gpio::PB5<Input>;
type ButtonRightPin = gpio::PB4<Input>;

static G_BUTTON_LEFT: Mutex<RefCell<Option<ButtonLeftPin>>> =
    Mutex::new(RefCell::new(None));
static G_BUTTON_RIGHT: Mutex<RefCell<Option<ButtonRightPin>>> =
    Mutex::new(RefCell::new(None));

static G_DELAY_IN_MS: Mutex<Cell<u32>> = Mutex::new(Cell::new(500_u32));

const fn starts_with_one(num: u8) -> bool {
    (0b1000_0000 & num) != 0
}

const fn ends_with_one(num: u8) -> bool {
    (0b0000_0001 & num) != 0
}

enum Direction {
    Up,
    Down,
}

#[entry]
fn main() -> ! {
    let mut dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let mut syscfg = dp.SYSCFG.constrain();
    let clocks = rcc.cfgr.freeze();
    let ahb_frequency = clocks.hclk().raw();

    let mut delay = Delay::new(cp.SYST, ahb_frequency);

    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let mut button_left = gpiob.pb5;
    let mut button_right = gpiob.pb4;

    button_left.make_interrupt_source(&mut syscfg);
    button_left.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    button_left.enable_interrupt(&mut dp.EXTI);

    button_right.make_interrupt_source(&mut syscfg);
    button_right.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    button_right.enable_interrupt(&mut dp.EXTI);

    unsafe {
        cortex_m::peripheral::NVIC::unmask(button_left.interrupt());
        cortex_m::peripheral::NVIC::unmask(button_right.interrupt());
    }

    cortex_m::interrupt::free(|cs| {
        G_BUTTON_LEFT.borrow(cs).replace(Some(button_left));
        G_BUTTON_RIGHT.borrow(cs).replace(Some(button_right.into()));
    });

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

    let mut pattern: u8 = 0b_1010_0000;
    let mut direction = Direction::Down;

    loop {
        direct_leds.apply_pattern_to_leds(pattern);
        delay.delay_ms(cortex_m::interrupt::free(|cs| {
            G_DELAY_IN_MS.borrow(cs).get()
        }));

        match direction {
            Direction::Down => pattern >>= 1,
            Direction::Up => pattern <<= 1,
        };

        match (starts_with_one(pattern), ends_with_one(pattern)) {
            (true, false) => direction = Direction::Down,
            (false, true) => direction = Direction::Up,
            _ => {}
        }
    }
}

#[allow(non_snake_case)]
#[interrupt]
fn EXTI4() {
    cortex_m::interrupt::free(|cs| {
        G_DELAY_IN_MS
            .borrow(cs)
            .set(G_DELAY_IN_MS.borrow(cs).get() - 25_u32);
        if G_DELAY_IN_MS.borrow(cs).get() < 25_u32 {
            G_DELAY_IN_MS.borrow(cs).set(25_u32);
        }

        let mut button_right = G_BUTTON_RIGHT.borrow(cs).borrow_mut();
        button_right.as_mut().unwrap().clear_interrupt_pending_bit();
    });
}

#[allow(non_snake_case)]
#[interrupt]
fn EXTI9_5() {
    cortex_m::interrupt::free(|cs| {
        G_DELAY_IN_MS
            .borrow(cs)
            .set(G_DELAY_IN_MS.borrow(cs).get() + 25_u32);
        if G_DELAY_IN_MS.borrow(cs).get() > 1_000_u32 {
            G_DELAY_IN_MS.borrow(cs).set(1_000_u32);
        }

        let mut button_left = G_BUTTON_LEFT.borrow(cs).borrow_mut();
        button_left.as_mut().unwrap().clear_interrupt_pending_bit();
    });
}
