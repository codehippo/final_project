#![no_main]
#![no_std]

use bitvec::prelude::*;
use bitvec::view::BitView;
use core::cell::{Cell, RefCell};
use core::fmt::Write;
use core::ops::DerefMut;
use bitvec::macros::internal::funty::Fundamental;
use cortex_m::delay::Delay;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::Peripherals as CorePeripherals;
use cortex_m_rt::entry;
use map_examples as _;
use stm32f4xx_hal::gpio::{Output, PartiallyErasedPin, PushPull};
use stm32f4xx_hal::pac::{interrupt, Peripherals, USART2};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::serial::{Config, Event, Serial};

static G_SERIAL: Mutex<RefCell<Option<Serial<USART2, u8>>>> = Mutex::new(RefCell::new(None));
static G_DELAY_IN_MS: Mutex<Cell<u32>> = Mutex::new(Cell::new(500_u32));

const FASTER_CHAR: char = 'f';
const SLOWER_CHAR: char = 's';

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

fn apply_pattern_to_leds(pattern: u8, leds: &mut [PartiallyErasedPin<'C', Output<PushPull>>; 8]) {
    let pattern_bits = pattern.view_bits::<Msb0>();

    for (bit_number, pattern_bit) in pattern_bits.into_iter().enumerate() {
        if *pattern_bit {
            leds[bit_number].set_high();
        } else {
            leds[bit_number].set_low();
        }
    }
}

// Define the entry point of the program
#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze();
    let ahb_frequency = clocks.hclk().raw();

    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    let tx_pin = gpioa.pa2;
    let rx_pin = gpioa.pa3;

    let mut serial: Serial<USART2, u8> = dp.USART2.serial(
        (tx_pin, rx_pin),
        Config::default()
            .baudrate(115200.bps())
            .wordlength_8()
            .parity_none(),
        &clocks
    ).unwrap();

    // Configure LEDs pins as a push-pull output
    let mut leds = [
        gpioc.pc0.into_push_pull_output().erase_number(),
        gpioc.pc1.into_push_pull_output().erase_number(),
        gpioc.pc2.into_push_pull_output().erase_number(),
        gpioc.pc3.into_push_pull_output().erase_number(),
        gpioc.pc4.into_push_pull_output().erase_number(),
        gpioc.pc5.into_push_pull_output().erase_number(),
        gpioc.pc6.into_push_pull_output().erase_number(),
        gpioc.pc7.into_push_pull_output().erase_number(),
    ];

    writeln!(serial, "Control the speed of LEDs by pressing either {} for faster, and {} for slower speed.\r", FASTER_CHAR, SLOWER_CHAR).unwrap();
    
    let mut delay = Delay::new(cp.SYST, ahb_frequency);
    let mut pattern: u8 = 0b_1000_0000;
    let mut direction = Direction::Down;

    serial.listen(Event::Rxne);

    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::USART2);
    }

    cortex_m::interrupt::free(|cs| {
        G_SERIAL.borrow(cs).replace(Some(serial));
    });
    
    loop {
        apply_pattern_to_leds(pattern, &mut leds);
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

#[interrupt]
fn USART2() {
    // Start a Critical Section
    cortex_m::interrupt::free(|cs| {
        let mut serial_ref = G_SERIAL.borrow(cs).borrow_mut();

        if let Some(ref mut serial) = serial_ref.deref_mut() {
            if let Ok(character) = serial.read() {
                match character.as_char().unwrap() {
                    FASTER_CHAR => {
                        G_DELAY_IN_MS
                            .borrow(cs)
                            .set(G_DELAY_IN_MS.borrow(cs).get() - 25_u32);
                        if G_DELAY_IN_MS.borrow(cs).get() < 25_u32 {
                            G_DELAY_IN_MS.borrow(cs).set(25_u32);
                            writeln!(serial, "Sorry, can't go any faster!\r").unwrap();
                        } else {
                            writeln!(serial, "Faster we go!\r").unwrap();
                        }
                    }
                    SLOWER_CHAR => {
                        G_DELAY_IN_MS
                            .borrow(cs)
                            .set(G_DELAY_IN_MS.borrow(cs).get() + 25_u32);
                        if G_DELAY_IN_MS.borrow(cs).get() > 1_000_u32 {
                            writeln!(serial, "Sorry, can't go any slower!\r").unwrap();
                            G_DELAY_IN_MS.borrow(cs).set(1_000_u32);
                        } else {
                            writeln!(serial, "Slower we go!\r").unwrap();
                        }
                    }
                    _ => {
                        writeln!(serial, "Unrecognised character, try again.\r").unwrap();
                    }
                }
            }
        }
    });
}
