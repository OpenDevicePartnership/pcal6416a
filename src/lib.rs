//! This is a platform-agnostic Rust driver for the NXP PCAL6416A IO Expander
//! based on the [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal
//!
//! For further details of the device architecture and operation, please refer
//! to the official [`Datasheet`].
//!
//! [`Datasheet`]: https://www.nxp.com/docs/en/data-sheet/PCAL6416A.pdf

#![doc = include_str!("../README.md")]
#![cfg_attr(not(test), no_std)]
#![allow(missing_docs)]

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pcal6416aError<E> {
    /// I2C bus error
    I2c(E),
}
const IOEXP_ADDR_LOW: u8 = 0x20;
const IOEXP_ADDR_HIGH: u8 = 0x21;
const LARGEST_REG_SIZE_BYTES: usize = 2;

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Debug)]
pub enum AddrPinState {
    High,
    Low,
}

impl AddrPinState {
    #[must_use]
    pub fn address(&self) -> u8 {
        match self {
            Self::High => IOEXP_ADDR_HIGH,
            Self::Low => IOEXP_ADDR_LOW,
        }
    }
}

pub struct Pcal6416aDevice<I2c> {
    pub addr_pin: AddrPinState,
    pub i2cbus: I2c,
}

device_driver::create_device!(
    device_name: Device,
    manifest: "device.yaml"
);

impl<I2c: embedded_hal_async::i2c::I2c> device_driver::AsyncRegisterInterface for Pcal6416aDevice<I2c> {
    type Error = Pcal6416aError<I2c::Error>;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        assert!((data.len() <= LARGEST_REG_SIZE_BYTES), "Register size too big");

        // Add one byte for register address
        let mut buf = [0u8; 1 + LARGEST_REG_SIZE_BYTES];
        buf[0] = address;
        buf[1..=data.len()].copy_from_slice(data);

        // Because the pcal6416a has a mix of 1 byte and 2 byte registers that can be written to,
        // we pass in a slice of the appropriate size so we do not accidentally write to the register at
        // address + 1 when writing to a 1 byte register
        self.i2cbus
            .write(self.addr_pin.address(), &buf[..=data.len()])
            .await
            .map_err(Pcal6416aError::I2c)
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2cbus
            .write_read(self.addr_pin.address(), &[address], data)
            .await
            .map_err(Pcal6416aError::I2c)
    }
}

impl<I2c: embedded_hal::i2c::I2c> device_driver::RegisterInterface for Pcal6416aDevice<I2c> {
    type Error = Pcal6416aError<I2c::Error>;
    type AddressType = u8;

    fn write_register(&mut self, address: Self::AddressType, _size_bits: u32, data: &[u8]) -> Result<(), Self::Error> {
        assert!((data.len() <= LARGEST_REG_SIZE_BYTES), "Register size too big");

        // Add one byte for register address
        let mut buf = [0u8; 1 + LARGEST_REG_SIZE_BYTES];
        buf[0] = address;
        buf[1..=data.len()].copy_from_slice(data);

        // Because the pcal6416a has a mix of 1 byte and 2 byte registers that can be written to,
        // we pass in a slice of the appropriate size so we do not accidentally write to the register at
        // address + 1 when writing to a 1 byte register
        self.i2cbus
            .write(self.addr_pin.address(), &buf[..=data.len()])
            .map_err(Pcal6416aError::I2c)
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2cbus
            .write_read(self.addr_pin.address(), &[address], data)
            .map_err(Pcal6416aError::I2c)
    }
}

/// Pin number for the PCAL6416A device
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pin {
    /// Pin 0 (Port 0, bit 0)
    Pin0,
    /// Pin 1 (Port 0, bit 1)
    Pin1,
    /// Pin 2 (Port 0, bit 2)
    Pin2,
    /// Pin 3 (Port 0, bit 3)
    Pin3,
    /// Pin 4 (Port 0, bit 4)
    Pin4,
    /// Pin 5 (Port 0, bit 5)
    Pin5,
    /// Pin 6 (Port 0, bit 6)
    Pin6,
    /// Pin 7 (Port 0, bit 7)
    Pin7,
    /// Pin 8 (Port 1, bit 0)
    Pin8,
    /// Pin 9 (Port 1, bit 1)
    Pin9,
    /// Pin 10 (Port 1, bit 2)
    Pin10,
    /// Pin 11 (Port 1, bit 3)
    Pin11,
    /// Pin 12 (Port 1, bit 4)
    Pin12,
    /// Pin 13 (Port 1, bit 5)
    Pin13,
    /// Pin 14 (Port 1, bit 6)
    Pin14,
    /// Pin 15 (Port 1, bit 7)
    Pin15,
}

impl Pin {
    /// Get the port number (0 or 1)
    #[must_use]
    const fn port(self) -> u8 {
        match self {
            Self::Pin0 | Self::Pin1 | Self::Pin2 | Self::Pin3 | Self::Pin4 | Self::Pin5 | Self::Pin6 | Self::Pin7 => 0,
            Self::Pin8
            | Self::Pin9
            | Self::Pin10
            | Self::Pin11
            | Self::Pin12
            | Self::Pin13
            | Self::Pin14
            | Self::Pin15 => 1,
        }
    }

    /// Get the bit position within the port (0-7)
    #[must_use]
    const fn bit(self) -> u8 {
        match self {
            Self::Pin0 | Self::Pin8 => 0,
            Self::Pin1 | Self::Pin9 => 1,
            Self::Pin2 | Self::Pin10 => 2,
            Self::Pin3 | Self::Pin11 => 3,
            Self::Pin4 | Self::Pin12 => 4,
            Self::Pin5 | Self::Pin13 => 5,
            Self::Pin6 | Self::Pin14 => 6,
            Self::Pin7 | Self::Pin15 => 7,
        }
    }

    /// Get the pin number (0-15)
    #[must_use]
    pub const fn number(&self) -> u8 {
        match self {
            Self::Pin0 => 0,
            Self::Pin1 => 1,
            Self::Pin2 => 2,
            Self::Pin3 => 3,
            Self::Pin4 => 4,
            Self::Pin5 => 5,
            Self::Pin6 => 6,
            Self::Pin7 => 7,
            Self::Pin8 => 8,
            Self::Pin9 => 9,
            Self::Pin10 => 10,
            Self::Pin11 => 11,
            Self::Pin12 => 12,
            Self::Pin13 => 13,
            Self::Pin14 => 14,
            Self::Pin15 => 15,
        }
    }
}

impl<I2c: embedded_hal::i2c::I2c> Device<Pcal6416aDevice<I2c>> {
    /// Read the state of an input pin
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub fn is_pin_high(&mut self, pin: Pin) -> Result<bool, Pcal6416aError<I2c::Error>> {
        let port = pin.port();
        let bit = pin.bit();

        let value = if port == 0 {
            let reg = self.input_port_0().read()?;
            let reg: [u8; 1] = reg.into();
            reg[0] & (1 << bit) != 0
        } else {
            let reg = self.input_port_1().read()?;
            let reg: [u8; 1] = reg.into();
            reg[0] & (1 << bit) != 0
        };

        Ok(value)
    }

    /// Read the state of an input pin
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub fn is_pin_low(&mut self, pin: Pin) -> Result<bool, Pcal6416aError<I2c::Error>> {
        Ok(!self.is_pin_high(pin)?)
    }

    /// Set an output pin to high state
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub fn set_pin_high(&mut self, pin: Pin) -> Result<(), Pcal6416aError<I2c::Error>> {
        let port = pin.port();
        let bit = pin.bit();

        if port == 0 {
            self.output_port_0().modify(|r| match bit {
                0 => r.set_o_0_0(true),
                1 => r.set_o_0_1(true),
                2 => r.set_o_0_2(true),
                3 => r.set_o_0_3(true),
                4 => r.set_o_0_4(true),
                5 => r.set_o_0_5(true),
                6 => r.set_o_0_6(true),
                7 => r.set_o_0_7(true),
                _ => unreachable!(),
            })
        } else {
            self.output_port_1().modify(|r| match bit {
                0 => r.set_o_1_0(true),
                1 => r.set_o_1_1(true),
                2 => r.set_o_1_2(true),
                3 => r.set_o_1_3(true),
                4 => r.set_o_1_4(true),
                5 => r.set_o_1_5(true),
                6 => r.set_o_1_6(true),
                7 => r.set_o_1_7(true),
                _ => unreachable!(),
            })
        }
    }

    /// Set an output pin to low state
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub fn set_pin_low(&mut self, pin: Pin) -> Result<(), Pcal6416aError<I2c::Error>> {
        let port = pin.port();
        let bit = pin.bit();

        if port == 0 {
            self.output_port_0().modify(|r| match bit {
                0 => r.set_o_0_0(false),
                1 => r.set_o_0_1(false),
                2 => r.set_o_0_2(false),
                3 => r.set_o_0_3(false),
                4 => r.set_o_0_4(false),
                5 => r.set_o_0_5(false),
                6 => r.set_o_0_6(false),
                7 => r.set_o_0_7(false),
                _ => unreachable!(),
            })
        } else {
            self.output_port_1().modify(|r| match bit {
                0 => r.set_o_1_0(false),
                1 => r.set_o_1_1(false),
                2 => r.set_o_1_2(false),
                3 => r.set_o_1_3(false),
                4 => r.set_o_1_4(false),
                5 => r.set_o_1_5(false),
                6 => r.set_o_1_6(false),
                7 => r.set_o_1_7(false),
                _ => unreachable!(),
            })
        }
    }

    /// Toggle an output pin state
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub fn toggle_pin(&mut self, pin: Pin) -> Result<(), Pcal6416aError<I2c::Error>> {
        let port = pin.port();
        let bit = pin.bit();

        if port == 0 {
            self.output_port_0().modify(|r| match bit {
                0 => r.set_o_0_0(!r.o_0_0()),
                1 => r.set_o_0_1(!r.o_0_1()),
                2 => r.set_o_0_2(!r.o_0_2()),
                3 => r.set_o_0_3(!r.o_0_3()),
                4 => r.set_o_0_4(!r.o_0_4()),
                5 => r.set_o_0_5(!r.o_0_5()),
                6 => r.set_o_0_6(!r.o_0_6()),
                7 => r.set_o_0_7(!r.o_0_7()),
                _ => unreachable!(),
            })
        } else {
            self.output_port_1().modify(|r| match bit {
                0 => r.set_o_1_0(!r.o_1_0()),
                1 => r.set_o_1_1(!r.o_1_1()),
                2 => r.set_o_1_2(!r.o_1_2()),
                3 => r.set_o_1_3(!r.o_1_3()),
                4 => r.set_o_1_4(!r.o_1_4()),
                5 => r.set_o_1_5(!r.o_1_5()),
                6 => r.set_o_1_6(!r.o_1_6()),
                7 => r.set_o_1_7(!r.o_1_7()),
                _ => unreachable!(),
            })
        }
    }

    /// Read the current state of an output pin
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub fn is_pin_set_high(&mut self, pin: Pin) -> Result<bool, Pcal6416aError<I2c::Error>> {
        let port = pin.port();
        let bit = pin.bit();

        let value = if port == 0 {
            let reg = self.output_port_0().read()?;
            let reg: [u8; 1] = reg.into();
            reg[0] & (1 << bit) != 0
        } else {
            let reg = self.output_port_1().read()?;
            let reg: [u8; 1] = reg.into();
            reg[0] & (1 << bit) != 0
        };

        Ok(value)
    }

    /// Read the current state of an output pin
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub fn is_pin_set_low(&mut self, pin: Pin) -> Result<bool, Pcal6416aError<I2c::Error>> {
        Ok(!self.is_pin_set_high(pin)?)
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    use super::*;

    #[tokio::test]
    async fn read_input_port_0_async() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x00], vec![0b01110111])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let input_port_0 = dev.input_port_0().read_async().await.unwrap();
        assert_eq!(input_port_0.i_0_7(), false);
        assert_eq!(input_port_0.i_0_6(), true);
        assert_eq!(input_port_0.i_0_5(), true);
        assert_eq!(input_port_0.i_0_4(), true);
        assert_eq!(input_port_0.i_0_3(), false);
        assert_eq!(input_port_0.i_0_2(), true);
        assert_eq!(input_port_0.i_0_1(), true);
        assert_eq!(input_port_0.i_0_0(), true);
        dev.interface.i2cbus.done();
    }

    #[test]
    fn read_input_port_0() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x00], vec![0b01110111])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let input_port_0 = dev.input_port_0().read().unwrap();
        assert_eq!(input_port_0.i_0_7(), false);
        assert_eq!(input_port_0.i_0_6(), true);
        assert_eq!(input_port_0.i_0_5(), true);
        assert_eq!(input_port_0.i_0_4(), true);
        assert_eq!(input_port_0.i_0_3(), false);
        assert_eq!(input_port_0.i_0_2(), true);
        assert_eq!(input_port_0.i_0_1(), true);
        assert_eq!(input_port_0.i_0_0(), true);
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn read_input_port_1_async() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x01], vec![0b01010101])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let input_port_1 = dev.input_port_1().read_async().await.unwrap();
        assert_eq!(input_port_1.i_1_7(), false);
        assert_eq!(input_port_1.i_1_6(), true);
        assert_eq!(input_port_1.i_1_5(), false);
        assert_eq!(input_port_1.i_1_4(), true);
        assert_eq!(input_port_1.i_1_3(), false);
        assert_eq!(input_port_1.i_1_2(), true);
        assert_eq!(input_port_1.i_1_1(), false);
        assert_eq!(input_port_1.i_1_0(), true);
        dev.interface.i2cbus.done();
    }

    #[test]
    fn read_input_port_1() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x01], vec![0b01010101])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let input_port_1 = dev.input_port_1().read().unwrap();
        assert_eq!(input_port_1.i_1_7(), false);
        assert_eq!(input_port_1.i_1_6(), true);
        assert_eq!(input_port_1.i_1_5(), false);
        assert_eq!(input_port_1.i_1_4(), true);
        assert_eq!(input_port_1.i_1_3(), false);
        assert_eq!(input_port_1.i_1_2(), true);
        assert_eq!(input_port_1.i_1_1(), false);
        assert_eq!(input_port_1.i_1_0(), true);
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn read_output_port_0_async() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b01000011])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let output_port_0 = dev.output_port_0().read_async().await.unwrap();
        assert_eq!(output_port_0.o_0_7(), false);
        assert_eq!(output_port_0.o_0_6(), true);
        assert_eq!(output_port_0.o_0_5(), false);
        assert_eq!(output_port_0.o_0_4(), false);
        assert_eq!(output_port_0.o_0_3(), false);
        assert_eq!(output_port_0.o_0_2(), false);
        assert_eq!(output_port_0.o_0_1(), true);
        assert_eq!(output_port_0.o_0_0(), true);
        dev.interface.i2cbus.done();
    }

    #[test]
    fn read_output_port_0() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b01000011])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let output_port_0 = dev.output_port_0().read().unwrap();
        assert_eq!(output_port_0.o_0_7(), false);
        assert_eq!(output_port_0.o_0_6(), true);
        assert_eq!(output_port_0.o_0_5(), false);
        assert_eq!(output_port_0.o_0_4(), false);
        assert_eq!(output_port_0.o_0_3(), false);
        assert_eq!(output_port_0.o_0_2(), false);
        assert_eq!(output_port_0.o_0_1(), true);
        assert_eq!(output_port_0.o_0_0(), true);
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn write_output_port_0_async() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b11110101])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.output_port_0()
            .write_async(|c| {
                c.set_o_0_7(true);
                c.set_o_0_6(true);
                c.set_o_0_5(true);
                c.set_o_0_4(true);
                c.set_o_0_3(false);
                c.set_o_0_2(true);
                c.set_o_0_1(false);
                c.set_o_0_0(true);
            })
            .await
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[test]
    fn write_output_port_0() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b11110101])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.output_port_0()
            .write(|c| {
                c.set_o_0_7(true);
                c.set_o_0_6(true);
                c.set_o_0_5(true);
                c.set_o_0_4(true);
                c.set_o_0_3(false);
                c.set_o_0_2(true);
                c.set_o_0_1(false);
                c.set_o_0_0(true);
            })
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn read_output_port_1_async() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x03], vec![0b01010010])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let output_port_1 = dev.output_port_1().read_async().await.unwrap();
        assert_eq!(output_port_1.o_1_7(), false);
        assert_eq!(output_port_1.o_1_6(), true);
        assert_eq!(output_port_1.o_1_5(), false);
        assert_eq!(output_port_1.o_1_4(), true);
        assert_eq!(output_port_1.o_1_3(), false);
        assert_eq!(output_port_1.o_1_2(), false);
        assert_eq!(output_port_1.o_1_1(), true);
        assert_eq!(output_port_1.o_1_0(), false);
        dev.interface.i2cbus.done();
    }

    #[test]
    fn read_output_port_1() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x03], vec![0b01010010])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let output_port_1 = dev.output_port_1().read().unwrap();
        assert_eq!(output_port_1.o_1_7(), false);
        assert_eq!(output_port_1.o_1_6(), true);
        assert_eq!(output_port_1.o_1_5(), false);
        assert_eq!(output_port_1.o_1_4(), true);
        assert_eq!(output_port_1.o_1_3(), false);
        assert_eq!(output_port_1.o_1_2(), false);
        assert_eq!(output_port_1.o_1_1(), true);
        assert_eq!(output_port_1.o_1_0(), false);
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn write_output_port_1_async() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_LOW, vec![0x03, 0b11010101])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.output_port_1()
            .write_async(|c| {
                c.set_o_1_7(true);
                c.set_o_1_6(true);
                c.set_o_1_5(false);
                c.set_o_1_4(true);
                c.set_o_1_3(false);
                c.set_o_1_2(true);
                c.set_o_1_1(false);
                c.set_o_1_0(true);
            })
            .await
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[test]
    fn write_output_port_1() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_LOW, vec![0x03, 0b11010101])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.output_port_1()
            .write(|c| {
                c.set_o_1_7(true);
                c.set_o_1_6(true);
                c.set_o_1_5(false);
                c.set_o_1_4(true);
                c.set_o_1_3(false);
                c.set_o_1_2(true);
                c.set_o_1_1(false);
                c.set_o_1_0(true);
            })
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn read_config_port_0_async() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x06], vec![0b01010111])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let config_port_0 = dev.config_port_0().read_async().await.unwrap();
        assert_eq!(config_port_0.c_0_7(), false);
        assert_eq!(config_port_0.c_0_6(), true);
        assert_eq!(config_port_0.c_0_5(), false);
        assert_eq!(config_port_0.c_0_4(), true);
        assert_eq!(config_port_0.c_0_3(), false);
        assert_eq!(config_port_0.c_0_2(), true);
        assert_eq!(config_port_0.c_0_1(), true);
        assert_eq!(config_port_0.c_0_0(), true);
        dev.interface.i2cbus.done();
    }

    #[test]
    fn read_config_port_0() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x06], vec![0b01010111])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let config_port_0 = dev.config_port_0().read().unwrap();
        assert_eq!(config_port_0.c_0_7(), false);
        assert_eq!(config_port_0.c_0_6(), true);
        assert_eq!(config_port_0.c_0_5(), false);
        assert_eq!(config_port_0.c_0_4(), true);
        assert_eq!(config_port_0.c_0_3(), false);
        assert_eq!(config_port_0.c_0_2(), true);
        assert_eq!(config_port_0.c_0_1(), true);
        assert_eq!(config_port_0.c_0_0(), true);
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn write_config_port_0_async() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_LOW, vec![0x06, 0b01010101])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.config_port_0()
            .write_async(|c| {
                c.set_c_0_7(false);
                c.set_c_0_6(true);
                c.set_c_0_5(false);
                c.set_c_0_4(true);
                c.set_c_0_3(false);
                c.set_c_0_2(true);
                c.set_c_0_1(false);
                c.set_c_0_0(true);
            })
            .await
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[test]
    fn write_config_port_0() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_LOW, vec![0x06, 0b01010101])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.config_port_0()
            .write(|c| {
                c.set_c_0_7(false);
                c.set_c_0_6(true);
                c.set_c_0_5(false);
                c.set_c_0_4(true);
                c.set_c_0_3(false);
                c.set_c_0_2(true);
                c.set_c_0_1(false);
                c.set_c_0_0(true);
            })
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn read_config_port_1_async() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x07], vec![0b01110111])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let config_port_1 = dev.config_port_1().read_async().await.unwrap();
        assert_eq!(config_port_1.c_1_7(), false);
        assert_eq!(config_port_1.c_1_6(), true);
        assert_eq!(config_port_1.c_1_5(), true);
        assert_eq!(config_port_1.c_1_4(), true);
        assert_eq!(config_port_1.c_1_3(), false);
        assert_eq!(config_port_1.c_1_2(), true);
        assert_eq!(config_port_1.c_1_1(), true);
        assert_eq!(config_port_1.c_1_0(), true);
        dev.interface.i2cbus.done();
    }

    #[test]
    fn read_config_port_1() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x07], vec![0b01110111])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let config_port_1 = dev.config_port_1().read().unwrap();
        assert_eq!(config_port_1.c_1_7(), false);
        assert_eq!(config_port_1.c_1_6(), true);
        assert_eq!(config_port_1.c_1_5(), true);
        assert_eq!(config_port_1.c_1_4(), true);
        assert_eq!(config_port_1.c_1_3(), false);
        assert_eq!(config_port_1.c_1_2(), true);
        assert_eq!(config_port_1.c_1_1(), true);
        assert_eq!(config_port_1.c_1_0(), true);
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn write_config_port_1_async() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_LOW, vec![0x07, 0b11110101])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.config_port_1()
            .write_async(|c| {
                c.set_c_1_7(true);
                c.set_c_1_6(true);
                c.set_c_1_5(true);
                c.set_c_1_4(true);
                c.set_c_1_3(false);
                c.set_c_1_2(true);
                c.set_c_1_1(false);
                c.set_c_1_0(true);
            })
            .await
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[test]
    fn write_config_port_1() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_LOW, vec![0x07, 0b11110101])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.config_port_1()
            .write(|c| {
                c.set_c_1_7(true);
                c.set_c_1_6(true);
                c.set_c_1_5(true);
                c.set_c_1_4(true);
                c.set_c_1_3(false);
                c.set_c_1_2(true);
                c.set_c_1_1(false);
                c.set_c_1_0(true);
            })
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn read_pull_up_down_enable_port_0_async() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x46], vec![0b00111010])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let pull_up_down_enable_port_0 = dev.pull_up_down_enable_port_0().read_async().await.unwrap();
        assert_eq!(pull_up_down_enable_port_0.pe_0_7(), false);
        assert_eq!(pull_up_down_enable_port_0.pe_0_6(), false);
        assert_eq!(pull_up_down_enable_port_0.pe_0_5(), true);
        assert_eq!(pull_up_down_enable_port_0.pe_0_4(), true);
        assert_eq!(pull_up_down_enable_port_0.pe_0_3(), true);
        assert_eq!(pull_up_down_enable_port_0.pe_0_2(), false);
        assert_eq!(pull_up_down_enable_port_0.pe_0_1(), true);
        assert_eq!(pull_up_down_enable_port_0.pe_0_0(), false);
        dev.interface.i2cbus.done();
    }

    #[test]
    fn read_pull_up_down_enable_port_0() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x46], vec![0b00111010])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let pull_up_down_enable_port_0 = dev.pull_up_down_enable_port_0().read().unwrap();
        assert_eq!(pull_up_down_enable_port_0.pe_0_7(), false);
        assert_eq!(pull_up_down_enable_port_0.pe_0_6(), false);
        assert_eq!(pull_up_down_enable_port_0.pe_0_5(), true);
        assert_eq!(pull_up_down_enable_port_0.pe_0_4(), true);
        assert_eq!(pull_up_down_enable_port_0.pe_0_3(), true);
        assert_eq!(pull_up_down_enable_port_0.pe_0_2(), false);
        assert_eq!(pull_up_down_enable_port_0.pe_0_1(), true);
        assert_eq!(pull_up_down_enable_port_0.pe_0_0(), false);
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn write_pull_up_down_enable_port_0_async() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_LOW, vec![0x46, 0b00111010])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.pull_up_down_enable_port_0()
            .write_async(|c| {
                c.set_pe_0_7(false);
                c.set_pe_0_6(false);
                c.set_pe_0_5(true);
                c.set_pe_0_4(true);
                c.set_pe_0_3(true);
                c.set_pe_0_2(false);
                c.set_pe_0_1(true);
                c.set_pe_0_0(false);
            })
            .await
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[test]
    fn write_pull_up_down_enable_port_0() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_LOW, vec![0x46, 0b00111010])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.pull_up_down_enable_port_0()
            .write(|c| {
                c.set_pe_0_7(false);
                c.set_pe_0_6(false);
                c.set_pe_0_5(true);
                c.set_pe_0_4(true);
                c.set_pe_0_3(true);
                c.set_pe_0_2(false);
                c.set_pe_0_1(true);
                c.set_pe_0_0(false);
            })
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn read_pull_up_down_enable_port_1_async() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x47], vec![0b11101100])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let pull_up_down_enable_port_1 = dev.pull_up_down_enable_port_1().read_async().await.unwrap();
        assert_eq!(pull_up_down_enable_port_1.pe_1_7(), true);
        assert_eq!(pull_up_down_enable_port_1.pe_1_6(), true);
        assert_eq!(pull_up_down_enable_port_1.pe_1_5(), true);
        assert_eq!(pull_up_down_enable_port_1.pe_1_4(), false);
        assert_eq!(pull_up_down_enable_port_1.pe_1_3(), true);
        assert_eq!(pull_up_down_enable_port_1.pe_1_2(), true);
        assert_eq!(pull_up_down_enable_port_1.pe_1_1(), false);
        assert_eq!(pull_up_down_enable_port_1.pe_1_0(), false);
        dev.interface.i2cbus.done();
    }

    #[test]
    fn read_pull_up_down_enable_port_1() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x47], vec![0b11101100])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let pull_up_down_enable_port_1 = dev.pull_up_down_enable_port_1().read().unwrap();
        assert_eq!(pull_up_down_enable_port_1.pe_1_7(), true);
        assert_eq!(pull_up_down_enable_port_1.pe_1_6(), true);
        assert_eq!(pull_up_down_enable_port_1.pe_1_5(), true);
        assert_eq!(pull_up_down_enable_port_1.pe_1_4(), false);
        assert_eq!(pull_up_down_enable_port_1.pe_1_3(), true);
        assert_eq!(pull_up_down_enable_port_1.pe_1_2(), true);
        assert_eq!(pull_up_down_enable_port_1.pe_1_1(), false);
        assert_eq!(pull_up_down_enable_port_1.pe_1_0(), false);
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn write_pull_up_down_enable_port_1_async() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_LOW, vec![0x47, 0b01011100])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.pull_up_down_enable_port_1()
            .write_async(|c| {
                c.set_pe_1_7(false);
                c.set_pe_1_6(true);
                c.set_pe_1_5(false);
                c.set_pe_1_4(true);
                c.set_pe_1_3(true);
                c.set_pe_1_2(true);
                c.set_pe_1_1(false);
                c.set_pe_1_0(false);
            })
            .await
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[test]
    fn write_pull_up_down_enable_port_1() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_LOW, vec![0x47, 0b11101010])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.pull_up_down_enable_port_1()
            .write(|c| {
                c.set_pe_1_7(true);
                c.set_pe_1_6(true);
                c.set_pe_1_5(true);
                c.set_pe_1_4(false);
                c.set_pe_1_3(true);
                c.set_pe_1_2(false);
                c.set_pe_1_1(true);
                c.set_pe_1_0(false);
            })
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn read_pull_up_down_select_port_0_async() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x48], vec![0b00111010])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let pull_up_down_select_port_0 = dev.pull_up_down_select_port_0().read_async().await.unwrap();
        assert_eq!(pull_up_down_select_port_0.pud_0_7(), false);
        assert_eq!(pull_up_down_select_port_0.pud_0_6(), false);
        assert_eq!(pull_up_down_select_port_0.pud_0_5(), true);
        assert_eq!(pull_up_down_select_port_0.pud_0_4(), true);
        assert_eq!(pull_up_down_select_port_0.pud_0_3(), true);
        assert_eq!(pull_up_down_select_port_0.pud_0_2(), false);
        assert_eq!(pull_up_down_select_port_0.pud_0_1(), true);
        assert_eq!(pull_up_down_select_port_0.pud_0_0(), false);
        dev.interface.i2cbus.done();
    }

    #[test]
    fn read_pull_up_down_select_port_0() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x48], vec![0b00111010])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let pull_up_down_select_port_0 = dev.pull_up_down_select_port_0().read().unwrap();
        assert_eq!(pull_up_down_select_port_0.pud_0_7(), false);
        assert_eq!(pull_up_down_select_port_0.pud_0_6(), false);
        assert_eq!(pull_up_down_select_port_0.pud_0_5(), true);
        assert_eq!(pull_up_down_select_port_0.pud_0_4(), true);
        assert_eq!(pull_up_down_select_port_0.pud_0_3(), true);
        assert_eq!(pull_up_down_select_port_0.pud_0_2(), false);
        assert_eq!(pull_up_down_select_port_0.pud_0_1(), true);
        assert_eq!(pull_up_down_select_port_0.pud_0_0(), false);
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn write_pull_up_down_select_port_0_async() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_LOW, vec![0x48, 0b01011001])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.pull_up_down_select_port_0()
            .write_async(|c| {
                c.set_pud_0_7(false);
                c.set_pud_0_6(true);
                c.set_pud_0_5(false);
                c.set_pud_0_4(true);
                c.set_pud_0_3(true);
                c.set_pud_0_2(false);
                c.set_pud_0_1(false);
                c.set_pud_0_0(true);
            })
            .await
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[test]
    fn write_pull_up_down_select_port_0() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_LOW, vec![0x48, 0b11101010])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.pull_up_down_select_port_0()
            .write(|c| {
                c.set_pud_0_7(true);
                c.set_pud_0_6(true);
                c.set_pud_0_5(true);
                c.set_pud_0_4(false);
                c.set_pud_0_3(true);
                c.set_pud_0_2(false);
                c.set_pud_0_1(true);
                c.set_pud_0_0(false);
            })
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn read_pull_up_down_select_port_1_async() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x49], vec![0b01100111])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let pull_up_down_select_port_1 = dev.pull_up_down_select_port_1().read_async().await.unwrap();
        assert_eq!(pull_up_down_select_port_1.pud_1_7(), false);
        assert_eq!(pull_up_down_select_port_1.pud_1_6(), true);
        assert_eq!(pull_up_down_select_port_1.pud_1_5(), true);
        assert_eq!(pull_up_down_select_port_1.pud_1_4(), false);
        assert_eq!(pull_up_down_select_port_1.pud_1_3(), false);
        assert_eq!(pull_up_down_select_port_1.pud_1_2(), true);
        assert_eq!(pull_up_down_select_port_1.pud_1_1(), true);
        assert_eq!(pull_up_down_select_port_1.pud_1_0(), true);
        dev.interface.i2cbus.done();
    }

    #[test]
    fn read_pull_up_down_select_port_1() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x49], vec![0b01100111])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let pull_up_down_select_port_1 = dev.pull_up_down_select_port_1().read().unwrap();
        assert_eq!(pull_up_down_select_port_1.pud_1_7(), false);
        assert_eq!(pull_up_down_select_port_1.pud_1_6(), true);
        assert_eq!(pull_up_down_select_port_1.pud_1_5(), true);
        assert_eq!(pull_up_down_select_port_1.pud_1_4(), false);
        assert_eq!(pull_up_down_select_port_1.pud_1_3(), false);
        assert_eq!(pull_up_down_select_port_1.pud_1_2(), true);
        assert_eq!(pull_up_down_select_port_1.pud_1_1(), true);
        assert_eq!(pull_up_down_select_port_1.pud_1_0(), true);
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn write_pull_up_down_select_port_1_async() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_LOW, vec![0x49, 0b00011011])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.pull_up_down_select_port_1()
            .write_async(|c| {
                c.set_pud_1_7(false);
                c.set_pud_1_6(false);
                c.set_pud_1_5(false);
                c.set_pud_1_4(true);
                c.set_pud_1_3(true);
                c.set_pud_1_2(false);
                c.set_pud_1_1(true);
                c.set_pud_1_0(true);
            })
            .await
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[test]
    fn write_pull_up_down_select_port_1() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_LOW, vec![0x49, 0b00011011])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.pull_up_down_select_port_1()
            .write(|c| {
                c.set_pud_1_7(false);
                c.set_pud_1_6(false);
                c.set_pud_1_5(false);
                c.set_pud_1_4(true);
                c.set_pud_1_3(true);
                c.set_pud_1_2(false);
                c.set_pud_1_1(true);
                c.set_pud_1_0(true);
            })
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn write_low_address() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_LOW, vec![0x07, 0])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.config_port_1()
            .write_async(|c| {
                c.set_c_1_7(false);
                c.set_c_1_6(false);
                c.set_c_1_5(false);
                c.set_c_1_4(false);
                c.set_c_1_3(false);
                c.set_c_1_2(false);
                c.set_c_1_1(false);
                c.set_c_1_0(false);
            })
            .await
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn write_high_address() {
        let expectations = vec![Transaction::write(IOEXP_ADDR_HIGH, vec![0x07, 0x0])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::High,
            i2cbus,
        });
        dev.config_port_1()
            .write_async(|c| {
                c.set_c_1_7(false);
                c.set_c_1_6(false);
                c.set_c_1_5(false);
                c.set_c_1_4(false);
                c.set_c_1_3(false);
                c.set_c_1_2(false);
                c.set_c_1_1(false);
                c.set_c_1_0(false);
            })
            .await
            .unwrap();
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn read_low_address() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x07], vec![0x0])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let _ = dev.config_port_1().read_async().await.unwrap();
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn read_high_address() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_HIGH, vec![0x07], vec![0x0])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::High,
            i2cbus,
        });
        let _ = dev.config_port_1().read_async().await.unwrap();
        dev.interface.i2cbus.done();
    }

    #[test]
    fn input_pin_is_high() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x00], vec![0b0000_0001])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        assert!(dev.is_pin_high(Pin::Pin0).unwrap());
        dev.interface.i2cbus.done();
    }

    #[test]
    fn input_pin_is_low() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x00], vec![0b0000_0000])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        assert!(dev.is_pin_low(Pin::Pin0).unwrap());
        dev.interface.i2cbus.done();
    }

    #[test]
    fn input_pin_port1() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x01], vec![0b1000_0000])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        assert!(dev.is_pin_high(Pin::Pin15).unwrap());
        dev.interface.i2cbus.done();
    }

    #[test]
    fn output_pin_set_high() {
        let expectations = vec![
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0000]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b0000_0001]),
        ];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.set_pin_high(Pin::Pin0).unwrap();
        dev.interface.i2cbus.done();
    }

    #[test]
    fn output_pin_set_low() {
        let expectations = vec![
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b1111_1111]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b1111_1110]),
        ];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.set_pin_low(Pin::Pin0).unwrap();
        dev.interface.i2cbus.done();
    }

    #[test]
    fn output_pin_port1() {
        let expectations = vec![
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x03], vec![0b0111_1111]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x03, 0b1111_1111]),
        ];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.set_pin_high(Pin::Pin15).unwrap();
        dev.interface.i2cbus.done();
    }

    #[test]
    fn toggle_pin() {
        let expectations = vec![
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0001]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b0000_0000]),
        ];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.toggle_pin(Pin::Pin0).unwrap();
        dev.interface.i2cbus.done();
    }

    #[test]
    fn is_pin_set_high() {
        let expectations = vec![Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0001])];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        assert!(dev.is_pin_set_high(Pin::Pin0).unwrap());
        dev.interface.i2cbus.done();
    }

    #[test]
    fn multiple_pins_at_once() {
        let expectations = vec![
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0000]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b0000_0001]),
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0001]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b0000_0011]),
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x00], vec![0b1111_1111]),
        ];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        // Set multiple pins without borrowing conflicts
        dev.set_pin_high(Pin::Pin0).unwrap();
        dev.set_pin_high(Pin::Pin1).unwrap();
        assert!(dev.is_pin_high(Pin::Pin7).unwrap());
        dev.interface.i2cbus.done();
    }
}
