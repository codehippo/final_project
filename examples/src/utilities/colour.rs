pub enum ColourChannel {
    Red,
    Green,
    Blue,
}

pub struct SimpleColour {
    r: bool,
    g: bool,
    b: bool,
}

impl SimpleColour {
    pub const fn new(r: bool, g: bool, b: bool) -> Self {
        Self {
            r,
            g,
            b
        }
    }

    pub fn as_array(&self) -> [bool; 3] {
        [self.r, self.g, self.b]
    }
}

#[derive(Clone, Copy)]
pub struct Colour<const CAP: usize> {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    cap: u8,
}

impl<const CAP: usize> Colour<CAP> {
    pub fn new(r: u8, g: u8, b: u8) -> Result<Self, &'static str> {
        if CAP > u8::MAX as usize {
            Err("Cap too high. Limited by u8::MAX.")
        } else {
            match (r as usize <= CAP, g as usize <= CAP, b as usize <= CAP) {
                (true, true, true) => Ok(Self {
                    r,
                    g,
                    b,
                    cap: CAP as u8,
                }),
                (_, _, _) => Err("One of the R, G, B channels is too big. Limit to CAP constant."),
            }
        }
    }

    pub fn inc_channel(&mut self, colour_channel: &ColourChannel) {
        match colour_channel {
            ColourChannel::Red => {
                if self.r < self.cap {
                    self.r += 1;
                }
            }
            ColourChannel::Green => {
                if self.g < self.cap {
                    self.g += 1;
                }
            }
            ColourChannel::Blue => {
                if self.b < self.cap {
                    self.b += 1;
                }
            }
        }
    }

    pub fn dec_channel(&mut self, colour_channel: &ColourChannel) {
        match colour_channel {
            ColourChannel::Red => {
                if self.r > 0 {
                    self.r -= 1;
                }
            }
            ColourChannel::Green => {
                if self.g > 0 {
                    self.g -= 1;
                }
            }
            ColourChannel::Blue => {
                if self.b > 0 {
                    self.b -= 1;
                }
            }
        }
    }

    pub fn get_r_as_float(&self) -> f32 {
        self.r as f32 / CAP as f32
    }

    pub fn get_g_as_float(&self) -> f32 {
        self.g as f32 / CAP as f32
    }

    pub fn get_b_as_float(&self) -> f32 {
        self.b as f32 / CAP as f32
    }

    pub fn as_array(&self) -> [u8; 3] {
        [self.r, self.g, self.b]
    }

    pub fn as_scaled_array(&self, scale: usize) -> [usize; 3] {
        self.as_array()
            .map(|channel| (channel as usize * scale).clamp(0, CAP * scale - 1))
    }
}
