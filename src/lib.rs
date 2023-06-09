#![no_std]

use embedded_hal_1::{delay::DelayUs, digital::OutputPin, i2c::I2c};

pub const DEFAULT_ADDR: u8 = 0x38;

pub mod regs {
    pub const CHIPID: u8 = 0xA3;
    pub const VENDID: u8 = 0xA8;
    pub const FIRMVERS: u8 = 0xA6;
    pub const THRESHHOLD: u8 = 0x80;
    pub const NUMTOUCHES: u8 = 0x02;
    pub const GEST_ID: u8 = 0x01;
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Gesture {
    MoveUp = 0x10,
    MoveRight = 0x14,
    MoveDown = 0x18,
    MoveLeft = 0x1C,
    ZoomIn = 0x48,
    ZoomOut = 0x49,
    None = 0x00,
}

impl Gesture {
    pub fn gesture_id(&self) -> u8 {
        match self {
            Gesture::MoveUp => 0x10,
            Gesture::MoveRight => 0x14,
            Gesture::MoveDown => 0x18,
            Gesture::MoveLeft => 0x1C,
            Gesture::ZoomIn => 0x48,
            Gesture::ZoomOut => 0x49,
            Gesture::None => 0x00,
        }
    }

    fn from_u8(gesture: u8) -> Self {
        match gesture {
            0x10 => Gesture::MoveUp,
            0x14 => Gesture::MoveRight,
            0x18 => Gesture::MoveDown,
            0x1C => Gesture::MoveLeft,
            0x48 => Gesture::ZoomIn,
            0x49 => Gesture::ZoomOut,
            _ => Gesture::None,
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EventType {
    PressDown = 0b00,
    LiftUp = 0b01,
    Contact = 0b10,
    None = 0b11,
}

impl EventType {
    fn from_u8(event: u8) -> Self {
        match event {
            0b00 => EventType::PressDown,
            0b01 => EventType::LiftUp,
            0b10 => EventType::Contact,
            _ => EventType::None,
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PointEvent {
    pub x: u16,
    pub y: u16,
    pub event: EventType,
    pub weight: u8,
    pub area: u8,
    pub touch_id: Option<u8>,
}

pub struct FT6236<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C> FT6236<I2C>
where
    I2C: I2c,
{
    pub fn new(i2c: I2C) -> Self {
        FT6236 {
            i2c,
            addr: DEFAULT_ADDR,
        }
    }

    pub fn new_with_addr(i2c: I2C, addr: u8) -> Self {
        FT6236 { i2c, addr }
    }

    pub fn init(&mut self, config: Config) -> Result<(), I2C::Error> {
        let chipid = self.read_reg(regs::CHIPID)?;
        let vendid = self.read_reg(regs::VENDID)?;

        #[cfg(feature = "defmt")]
        defmt::info!("chipid 0x{:02x}, vendid 0x{:02x}", chipid, vendid);

        if vendid != 0x11 {
            #[cfg(feature = "defmt")]
            defmt::error!("invalid vendid");
        }
        if chipid != 0x06 && chipid != 0x36 && chipid != 0x64 {
            #[cfg(feature = "defmt")]
            defmt::error!("invalid chipid");
        }

        self.write_reg(regs::THRESHHOLD, config.threshhold)?;

        Ok(())
    }

    pub fn reset<P: OutputPin, D: DelayUs>(
        &mut self,
        rst: &mut P,
        delay: &mut D,
    ) -> Result<(), P::Error> {
        rst.set_high()?;
        delay.delay_us(100);
        rst.set_low()?;
        delay.delay_us(100);
        rst.set_high()?;
        delay.delay_us(100);

        Ok(())
    }

    /// Number of touches, 0, 1 or 2
    pub fn get_number_of_touches(&mut self) -> Result<u8, I2C::Error> {
        let n = self.read_reg(regs::NUMTOUCHES)?;
        if n & 0b11 <= 2 {
            Ok(n)
        } else {
            Ok(0) // invalid
        }
    }

    /// get first touch point
    pub fn get_point0(&mut self) -> Result<Option<PointEvent>, I2C::Error> {
        self.get_point(0)
    }

    /// get second touch point
    pub fn get_point1(&mut self) -> Result<Option<PointEvent>, I2C::Error> {
        self.get_point(1)
    }

    #[inline]
    pub fn get_point(&mut self, nth: u8) -> Result<Option<PointEvent>, I2C::Error> {
        if self.get_number_of_touches()? <= nth {
            return Ok(None);
        }
        let mut buf = [0u8; 6];
        self.i2c
            .write_read(self.addr, &[0x03 + 6 * nth], &mut buf)?;

        let event = buf[0] >> 6;

        let x = (((buf[0] as u16) & 0b111) << 8) | (buf[1] as u16);

        let touch_id = buf[2] >> 4;
        let y = (((buf[2] as u16) & 0b111) << 8) | (buf[3] as u16);
        let weight = buf[4];
        let area = buf[5] & 0b1111;

        Ok(Some(PointEvent {
            x,
            y,
            event: EventType::from_u8(event),
            weight,
            area,
            touch_id: if touch_id == 0x0f {
                None
            } else {
                Some(touch_id)
            },
        }))
    }

    /// Get the gesture, this is not always available for all touch panels
    pub fn get_gesture(&mut self) -> Result<Gesture, I2C::Error> {
        let gesture = self.read_reg(regs::GEST_ID)?;
        Ok(Gesture::from_u8(gesture))
    }

    fn read_reg(&mut self, reg_addr: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0u8];
        self.i2c.write_read(self.addr, &[reg_addr], &mut buf)?;

        Ok(buf[0])
    }

    fn write_reg(&mut self, reg_addr: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[reg_addr, value])?;

        Ok(())
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub struct Config {
    /// threshold for touch detection
    pub threshhold: u8,
}

impl Default for Config {
    fn default() -> Self {
        Config { threshhold: 0x80 }
    }
}
