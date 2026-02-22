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

impl<E: core::fmt::Debug> embedded_hal::digital::Error for Pcal6416aError<E> {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
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

pub struct SharedDevice<I2c: embedded_hal_async::i2c::I2c, M: embassy_sync::blocking_mutex::raw::RawMutex> {
    pub device: embassy_sync::mutex::Mutex<M, Device<Pcal6416aDevice<I2c>>>,
}

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

/// Port number for the PCAL6416A device
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Port {
    /// Port 0 (pins 0-7)
    Port0,
    /// Port 1 (pins 0-7)
    Port1,
}

/// Pin number within a port (0-7) for the PCAL6416A device
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pin {
    /// Pin 0
    Pin0,
    /// Pin 1
    Pin1,
    /// Pin 2
    Pin2,
    /// Pin 3
    Pin3,
    /// Pin 4
    Pin4,
    /// Pin 5
    Pin5,
    /// Pin 6
    Pin6,
    /// Pin 7
    Pin7,
}

impl Pin {
    /// Get the bit position within the port (0-7)
    #[must_use]
    pub const fn bit(self) -> u8 {
        match self {
            Self::Pin0 => 0,
            Self::Pin1 => 1,
            Self::Pin2 => 2,
            Self::Pin3 => 3,
            Self::Pin4 => 4,
            Self::Pin5 => 5,
            Self::Pin6 => 6,
            Self::Pin7 => 7,
        }
    }

    /// Get the pin number within the port (0-7)
    #[must_use]
    pub const fn number(&self) -> u8 {
        self.bit()
    }
}

/// Individual pin instance that provides GPIO operations for a single pin
///
/// This struct is created by calling `split()` on a `SharedDevice` instance.
/// It provides methods to read and write the state of a single pin without
/// requiring mutable access to the entire device.
///
/// Note: This uses a shared mutex to provide safe concurrent access to the device.
/// All pin operations acquire the mutex lock before performing I2C operations.
pub struct IoPin<'a, I2c: embedded_hal_async::i2c::I2c, M: embassy_sync::blocking_mutex::raw::RawMutex> {
    port: Port,
    pin: Pin,
    device: &'a embassy_sync::mutex::Mutex<M, Device<Pcal6416aDevice<I2c>>>,
}

impl<'a, I2c: embedded_hal_async::i2c::I2c, M: embassy_sync::blocking_mutex::raw::RawMutex> IoPin<'a, I2c, M> {
    const fn new(
        port: Port,
        pin: Pin,
        device: &'a embassy_sync::mutex::Mutex<M, Device<Pcal6416aDevice<I2c>>>,
    ) -> Self {
        Self { port, pin, device }
    }

    /// Get the pin number within the port (0-7)
    #[must_use]
    pub const fn number(&self) -> u8 {
        self.pin.number()
    }

    /// Get the Pin enum for this pin
    #[must_use]
    pub const fn pin(&self) -> Pin {
        self.pin
    }

    /// Get the Port enum for this pin
    #[must_use]
    pub const fn port(&self) -> Port {
        self.port
    }
}

impl<I2c: embedded_hal_async::i2c::I2c, M: embassy_sync::blocking_mutex::raw::RawMutex> IoPin<'_, I2c, M> {
    /// Read the state of this input pin (async version)
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub async fn is_high_async(&self) -> Result<bool, Pcal6416aError<I2c::Error>> {
        self.device.lock().await.is_pin_high_async(self.port, self.pin).await
    }

    /// Read the state of this input pin (async version)
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub async fn is_low_async(&self) -> Result<bool, Pcal6416aError<I2c::Error>> {
        Ok(!self.is_high_async().await?)
    }

    /// Set this output pin to high state (async version)
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub async fn set_high_async(&self) -> Result<(), Pcal6416aError<I2c::Error>> {
        self.device.lock().await.set_pin_high_async(self.port, self.pin).await
    }

    /// Set this output pin to low state (async version)
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub async fn set_low_async(&self) -> Result<(), Pcal6416aError<I2c::Error>> {
        self.device.lock().await.set_pin_low_async(self.port, self.pin).await
    }

    /// Toggle this output pin state (async version)
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub async fn toggle_async(&self) -> Result<(), Pcal6416aError<I2c::Error>> {
        self.device.lock().await.toggle_pin_async(self.port, self.pin).await
    }

    /// Read the current state of this output pin (async version)
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub async fn is_set_high_async(&self) -> Result<bool, Pcal6416aError<I2c::Error>> {
        self.device
            .lock()
            .await
            .is_pin_set_high_async(self.port, self.pin)
            .await
    }

    /// Read the current state of this output pin (async version)
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub async fn is_set_low_async(&self) -> Result<bool, Pcal6416aError<I2c::Error>> {
        Ok(!self.is_set_high_async().await?)
    }
}

// Implement embedded-hal digital traits for IoPin
impl<I2c: embedded_hal_async::i2c::I2c, M: embassy_sync::blocking_mutex::raw::RawMutex> embedded_hal::digital::ErrorType
    for IoPin<'_, I2c, M>
{
    type Error = Pcal6416aError<I2c::Error>;
}

impl<I2c: embedded_hal_async::i2c::I2c, M: embassy_sync::blocking_mutex::raw::RawMutex>
    embedded_hal_async::digital::InputPin for IoPin<'_, I2c, M>
{
    async fn is_high(&mut self) -> Result<bool, Self::Error> {
        IoPin::is_high_async(self).await
    }

    async fn is_low(&mut self) -> Result<bool, Self::Error> {
        IoPin::is_low_async(self).await
    }
}

impl<I2c: embedded_hal_async::i2c::I2c, M: embassy_sync::blocking_mutex::raw::RawMutex>
    embedded_hal_async::digital::OutputPin for IoPin<'_, I2c, M>
{
    async fn set_low(&mut self) -> Result<(), Self::Error> {
        IoPin::set_low_async(self).await
    }

    async fn set_high(&mut self) -> Result<(), Self::Error> {
        IoPin::set_high_async(self).await
    }
}

impl<I2c: embedded_hal_async::i2c::I2c, M: embassy_sync::blocking_mutex::raw::RawMutex>
    embedded_hal_async::digital::StatefulOutputPin for IoPin<'_, I2c, M>
{
    async fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        IoPin::is_set_high_async(self).await
    }

    async fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        IoPin::is_set_low_async(self).await
    }

    async fn toggle(&mut self) -> Result<(), Self::Error> {
        IoPin::toggle_async(self).await
    }
}

impl<I2c: embedded_hal::i2c::I2c> Device<Pcal6416aDevice<I2c>> {
    /// Read the state of an input pin
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub fn is_pin_high(&mut self, port: Port, pin: Pin) -> Result<bool, Pcal6416aError<I2c::Error>> {
        let bit = pin.bit();

        let value = match port {
            Port::Port0 => {
                let reg = self.input_port_0().read()?;
                let reg: [u8; 1] = reg.into();
                reg[0] & (1 << bit) != 0
            }
            Port::Port1 => {
                let reg = self.input_port_1().read()?;
                let reg: [u8; 1] = reg.into();
                reg[0] & (1 << bit) != 0
            }
        };

        Ok(value)
    }

    /// Read the state of an input pin
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub fn is_pin_low(&mut self, port: Port, pin: Pin) -> Result<bool, Pcal6416aError<I2c::Error>> {
        Ok(!self.is_pin_high(port, pin)?)
    }

    /// Set an output pin to high state
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub fn set_pin_high(&mut self, port: Port, pin: Pin) -> Result<(), Pcal6416aError<I2c::Error>> {
        let bit = pin.bit();

        match port {
            Port::Port0 => self.output_port_0().modify(|r| match bit {
                0 => r.set_o_0_0(true),
                1 => r.set_o_0_1(true),
                2 => r.set_o_0_2(true),
                3 => r.set_o_0_3(true),
                4 => r.set_o_0_4(true),
                5 => r.set_o_0_5(true),
                6 => r.set_o_0_6(true),
                7 => r.set_o_0_7(true),
                _ => unreachable!(),
            }),
            Port::Port1 => self.output_port_1().modify(|r| match bit {
                0 => r.set_o_1_0(true),
                1 => r.set_o_1_1(true),
                2 => r.set_o_1_2(true),
                3 => r.set_o_1_3(true),
                4 => r.set_o_1_4(true),
                5 => r.set_o_1_5(true),
                6 => r.set_o_1_6(true),
                7 => r.set_o_1_7(true),
                _ => unreachable!(),
            }),
        }
    }

    /// Set an output pin to low state
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub fn set_pin_low(&mut self, port: Port, pin: Pin) -> Result<(), Pcal6416aError<I2c::Error>> {
        let bit = pin.bit();

        match port {
            Port::Port0 => self.output_port_0().modify(|r| match bit {
                0 => r.set_o_0_0(false),
                1 => r.set_o_0_1(false),
                2 => r.set_o_0_2(false),
                3 => r.set_o_0_3(false),
                4 => r.set_o_0_4(false),
                5 => r.set_o_0_5(false),
                6 => r.set_o_0_6(false),
                7 => r.set_o_0_7(false),
                _ => unreachable!(),
            }),
            Port::Port1 => self.output_port_1().modify(|r| match bit {
                0 => r.set_o_1_0(false),
                1 => r.set_o_1_1(false),
                2 => r.set_o_1_2(false),
                3 => r.set_o_1_3(false),
                4 => r.set_o_1_4(false),
                5 => r.set_o_1_5(false),
                6 => r.set_o_1_6(false),
                7 => r.set_o_1_7(false),
                _ => unreachable!(),
            }),
        }
    }

    /// Toggle an output pin state
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub fn toggle_pin(&mut self, port: Port, pin: Pin) -> Result<(), Pcal6416aError<I2c::Error>> {
        let bit = pin.bit();

        match port {
            Port::Port0 => self.output_port_0().modify(|r| match bit {
                0 => r.set_o_0_0(!r.o_0_0()),
                1 => r.set_o_0_1(!r.o_0_1()),
                2 => r.set_o_0_2(!r.o_0_2()),
                3 => r.set_o_0_3(!r.o_0_3()),
                4 => r.set_o_0_4(!r.o_0_4()),
                5 => r.set_o_0_5(!r.o_0_5()),
                6 => r.set_o_0_6(!r.o_0_6()),
                7 => r.set_o_0_7(!r.o_0_7()),
                _ => unreachable!(),
            }),
            Port::Port1 => self.output_port_1().modify(|r| match bit {
                0 => r.set_o_1_0(!r.o_1_0()),
                1 => r.set_o_1_1(!r.o_1_1()),
                2 => r.set_o_1_2(!r.o_1_2()),
                3 => r.set_o_1_3(!r.o_1_3()),
                4 => r.set_o_1_4(!r.o_1_4()),
                5 => r.set_o_1_5(!r.o_1_5()),
                6 => r.set_o_1_6(!r.o_1_6()),
                7 => r.set_o_1_7(!r.o_1_7()),
                _ => unreachable!(),
            }),
        }
    }

    /// Read the current state of an output pin
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub fn is_pin_set_high(&mut self, port: Port, pin: Pin) -> Result<bool, Pcal6416aError<I2c::Error>> {
        let bit = pin.bit();

        let value = match port {
            Port::Port0 => {
                let reg = self.output_port_0().read()?;
                let reg: [u8; 1] = reg.into();
                reg[0] & (1 << bit) != 0
            }
            Port::Port1 => {
                let reg = self.output_port_1().read()?;
                let reg: [u8; 1] = reg.into();
                reg[0] & (1 << bit) != 0
            }
        };

        Ok(value)
    }

    /// Read the current state of an output pin
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub fn is_pin_set_low(&mut self, port: Port, pin: Pin) -> Result<bool, Pcal6416aError<I2c::Error>> {
        Ok(!self.is_pin_set_high(port, pin)?)
    }
}

impl<I2c: embedded_hal_async::i2c::I2c> Device<Pcal6416aDevice<I2c>> {
    /// Read the state of an input pin (async version)
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub async fn is_pin_high_async(&mut self, port: Port, pin: Pin) -> Result<bool, Pcal6416aError<I2c::Error>> {
        let bit = pin.bit();

        let value = match port {
            Port::Port0 => {
                let reg = self.input_port_0().read_async().await?;
                let reg: [u8; 1] = reg.into();
                reg[0] & (1 << bit) != 0
            }
            Port::Port1 => {
                let reg = self.input_port_1().read_async().await?;
                let reg: [u8; 1] = reg.into();
                reg[0] & (1 << bit) != 0
            }
        };

        Ok(value)
    }

    /// Read the state of an input pin (async version)
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub async fn is_pin_low_async(&mut self, port: Port, pin: Pin) -> Result<bool, Pcal6416aError<I2c::Error>> {
        Ok(!self.is_pin_high_async(port, pin).await?)
    }

    /// Set an output pin to high state (async version)
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub async fn set_pin_high_async(&mut self, port: Port, pin: Pin) -> Result<(), Pcal6416aError<I2c::Error>> {
        let bit = pin.bit();

        match port {
            Port::Port0 => {
                self.output_port_0()
                    .modify_async(|r| match bit {
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
                    .await
            }
            Port::Port1 => {
                self.output_port_1()
                    .modify_async(|r| match bit {
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
                    .await
            }
        }
    }

    /// Set an output pin to low state (async version)
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub async fn set_pin_low_async(&mut self, port: Port, pin: Pin) -> Result<(), Pcal6416aError<I2c::Error>> {
        let bit = pin.bit();

        match port {
            Port::Port0 => {
                self.output_port_0()
                    .modify_async(|r| match bit {
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
                    .await
            }
            Port::Port1 => {
                self.output_port_1()
                    .modify_async(|r| match bit {
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
                    .await
            }
        }
    }

    /// Toggle an output pin state (async version)
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub async fn toggle_pin_async(&mut self, port: Port, pin: Pin) -> Result<(), Pcal6416aError<I2c::Error>> {
        let bit = pin.bit();

        match port {
            Port::Port0 => {
                self.output_port_0()
                    .modify_async(|r| match bit {
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
                    .await
            }
            Port::Port1 => {
                self.output_port_1()
                    .modify_async(|r| match bit {
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
                    .await
            }
        }
    }

    /// Read the current state of an output pin (async version)
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub async fn is_pin_set_high_async(&mut self, port: Port, pin: Pin) -> Result<bool, Pcal6416aError<I2c::Error>> {
        let bit = pin.bit();

        let value = match port {
            Port::Port0 => {
                let reg = self.output_port_0().read_async().await?;
                let reg: [u8; 1] = reg.into();
                reg[0] & (1 << bit) != 0
            }
            Port::Port1 => {
                let reg = self.output_port_1().read_async().await?;
                let reg: [u8; 1] = reg.into();
                reg[0] & (1 << bit) != 0
            }
        };

        Ok(value)
    }

    /// Read the current state of an output pin (async version)
    /// # Errors
    ///
    /// Will return `Err` if underlying I2C bus operation fails
    pub async fn is_pin_set_low_async(&mut self, port: Port, pin: Pin) -> Result<bool, Pcal6416aError<I2c::Error>> {
        Ok(!self.is_pin_set_high_async(port, pin).await?)
    }
}

impl<I2c: embedded_hal_async::i2c::I2c, M: embassy_sync::blocking_mutex::raw::RawMutex> SharedDevice<I2c, M> {
    pub fn new(device: Device<Pcal6416aDevice<I2c>>) -> Self {
        Self {
            device: embassy_sync::mutex::Mutex::new(device),
        }
    }

    /// Split the driver into an array of individual pin instances
    ///
    /// This borrows the shared device mutably and returns an array of 16 `IoPin` instances,
    /// one for each GPIO pin. The pins can be passed individually to different functions.
    /// Each pin uses the shared mutex to safely access the underlying device.
    ///
    /// # Example
    /// ```ignore
    /// let device = Device::new(Pcal6416aDevice { addr_pin, i2cbus });
    /// let mut shared = SharedDevice::new(device);
    /// let pins = shared.split();
    ///
    /// // Pass individual pins to different functions
    /// use_led(&pins[0]).await;
    /// use_button(&pins[1]).await;
    ///
    /// // Or access by index
    /// pins[2].set_high_async().await?;
    /// pins[3].set_low_async().await?;
    ///
    /// // Iterate over pins
    /// for (i, pin) in pins.iter().enumerate() {
    ///     println!("Pin {} number: {}", i, pin.number());
    /// }
    /// ```
    pub fn split(&mut self) -> [IoPin<'_, I2c, M>; 16] {
        [
            IoPin::new(Port::Port0, Pin::Pin0, &self.device),
            IoPin::new(Port::Port0, Pin::Pin1, &self.device),
            IoPin::new(Port::Port0, Pin::Pin2, &self.device),
            IoPin::new(Port::Port0, Pin::Pin3, &self.device),
            IoPin::new(Port::Port0, Pin::Pin4, &self.device),
            IoPin::new(Port::Port0, Pin::Pin5, &self.device),
            IoPin::new(Port::Port0, Pin::Pin6, &self.device),
            IoPin::new(Port::Port0, Pin::Pin7, &self.device),
            IoPin::new(Port::Port1, Pin::Pin0, &self.device),
            IoPin::new(Port::Port1, Pin::Pin1, &self.device),
            IoPin::new(Port::Port1, Pin::Pin2, &self.device),
            IoPin::new(Port::Port1, Pin::Pin3, &self.device),
            IoPin::new(Port::Port1, Pin::Pin4, &self.device),
            IoPin::new(Port::Port1, Pin::Pin5, &self.device),
            IoPin::new(Port::Port1, Pin::Pin6, &self.device),
            IoPin::new(Port::Port1, Pin::Pin7, &self.device),
        ]
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
        let expectations = vec![
            // Port 0 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x00], vec![0b0000_0001]),
            // Port 1 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x01], vec![0b1000_0000]),
        ];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        assert!(dev.is_pin_high(Port::Port0, Pin::Pin0).unwrap());
        assert!(dev.is_pin_high(Port::Port1, Pin::Pin7).unwrap());
        dev.interface.i2cbus.done();
    }

    #[test]
    fn input_pin_is_low() {
        let expectations = vec![
            // Port 0 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x00], vec![0b0000_0000]),
            // Port 1 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x01], vec![0b0000_0000]),
        ];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        assert!(dev.is_pin_low(Port::Port0, Pin::Pin0).unwrap());
        assert!(dev.is_pin_low(Port::Port1, Pin::Pin7).unwrap());
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
        assert!(dev.is_pin_high(Port::Port1, Pin::Pin7).unwrap());
        dev.interface.i2cbus.done();
    }

    #[test]
    fn output_pin_set_high() {
        let expectations = vec![
            // Port 0 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0000]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b0000_0001]),
            // Port 1 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x03], vec![0b0000_0000]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x03, 0b1000_0000]),
        ];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.set_pin_high(Port::Port0, Pin::Pin0).unwrap();
        dev.set_pin_high(Port::Port1, Pin::Pin7).unwrap();
        dev.interface.i2cbus.done();
    }

    #[test]
    fn output_pin_set_low() {
        let expectations = vec![
            // Port 0 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b1111_1111]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b1111_1110]),
            // Port 1 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x03], vec![0b1111_1111]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x03, 0b0111_1111]),
        ];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.set_pin_low(Port::Port0, Pin::Pin0).unwrap();
        dev.set_pin_low(Port::Port1, Pin::Pin7).unwrap();
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
        dev.set_pin_high(Port::Port1, Pin::Pin7).unwrap();
        dev.interface.i2cbus.done();
    }

    #[test]
    fn toggle_pin() {
        let expectations = vec![
            // Port 0 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0001]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b0000_0000]),
            // Port 1 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x03], vec![0b1000_0000]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x03, 0b0000_0000]),
        ];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        dev.toggle_pin(Port::Port0, Pin::Pin0).unwrap();
        dev.toggle_pin(Port::Port1, Pin::Pin7).unwrap();
        dev.interface.i2cbus.done();
    }

    #[test]
    fn is_pin_set_high() {
        let expectations = vec![
            // Port 0 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0001]),
            // Port 1 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x03], vec![0b1000_0000]),
        ];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        assert!(dev.is_pin_set_high(Port::Port0, Pin::Pin0).unwrap());
        assert!(dev.is_pin_set_high(Port::Port1, Pin::Pin7).unwrap());
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
        dev.set_pin_high(Port::Port0, Pin::Pin0).unwrap();
        dev.set_pin_high(Port::Port0, Pin::Pin1).unwrap();
        assert!(dev.is_pin_high(Port::Port0, Pin::Pin7).unwrap());
        dev.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn split_pins() {
        let expectations = vec![
            // Set pin 0 high
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0000]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b0000_0001]),
            // Set pin 1 high
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0001]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b0000_0011]),
            // Read pin 0
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x00], vec![0b0000_0011]),
            // Toggle pin 1
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0011]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b0000_0001]),
        ];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let mut dev = SharedDevice::new(dev);

        {
            let mut pins: [IoPin<
                '_,
                embedded_hal_mock::common::Generic<Transaction>,
                embassy_sync::blocking_mutex::raw::NoopRawMutex,
            >; 16] = dev.split();

            // Use individual pins independently
            pins[0].set_high_async().await.unwrap();
            pins[1].set_high_async().await.unwrap();
            assert!(pins[0].is_high_async().await.unwrap());
            pins[1].toggle_async().await.unwrap();
        }

        // Verify mock expectations
        dev.device.lock().await.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn split_pin_numbers() {
        let expectations = vec![];
        let i2cbus = Mock::new(&expectations);
        let dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let mut dev = SharedDevice::new(dev);

        {
            let pins: [IoPin<
                '_,
                embedded_hal_mock::common::Generic<Transaction>,
                embassy_sync::blocking_mutex::raw::NoopRawMutex,
            >; 16] = dev.split();

            // Verify all 16 pins have correct numbers (0-7 per port)
            for i in 0..8 {
                assert_eq!(pins[i].number(), i as u8, "Pin at index {} should have number {}", i, i);
                assert_eq!(pins[i].port(), Port::Port0, "Pin at index {} should be on Port 0", i);
            }
            for i in 8..16 {
                assert_eq!(
                    pins[i].number(),
                    (i - 8) as u8,
                    "Pin at index {} should have number {}",
                    i,
                    i - 8
                );
                assert_eq!(pins[i].port(), Port::Port1, "Pin at index {} should be on Port 1", i);
            }

            // Verify Pin enum values for all pins
            assert_eq!(pins[0].pin(), Pin::Pin0);
            assert_eq!(pins[1].pin(), Pin::Pin1);
            assert_eq!(pins[2].pin(), Pin::Pin2);
            assert_eq!(pins[3].pin(), Pin::Pin3);
            assert_eq!(pins[4].pin(), Pin::Pin4);
            assert_eq!(pins[5].pin(), Pin::Pin5);
            assert_eq!(pins[6].pin(), Pin::Pin6);
            assert_eq!(pins[7].pin(), Pin::Pin7);
            assert_eq!(pins[8].pin(), Pin::Pin0);
            assert_eq!(pins[9].pin(), Pin::Pin1);
            assert_eq!(pins[10].pin(), Pin::Pin2);
            assert_eq!(pins[11].pin(), Pin::Pin3);
            assert_eq!(pins[12].pin(), Pin::Pin4);
            assert_eq!(pins[13].pin(), Pin::Pin5);
            assert_eq!(pins[14].pin(), Pin::Pin6);
            assert_eq!(pins[15].pin(), Pin::Pin7);

            // Verify Port enum values
            assert_eq!(pins[0].port(), Port::Port0);
            assert_eq!(pins[7].port(), Port::Port0);
            assert_eq!(pins[8].port(), Port::Port1);
            assert_eq!(pins[15].port(), Port::Port1);
        }

        dev.device.lock().await.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn async_pin_set_high() {
        let expectations = vec![
            // Port 0 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0000]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b0000_0001]),
            // Port 1 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x03], vec![0b0000_0000]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x03, 0b1000_0000]),
        ];
        let i2cbus = Mock::new(&expectations);
        let dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let mut dev = SharedDevice::new(dev);

        {
            let pins: [IoPin<
                '_,
                embedded_hal_mock::common::Generic<Transaction>,
                embassy_sync::blocking_mutex::raw::NoopRawMutex,
            >; 16] = dev.split();
            pins[0].set_high_async().await.unwrap();
            pins[15].set_high_async().await.unwrap();
        }

        dev.device.lock().await.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn async_pin_set_low() {
        let expectations = vec![
            // Port 0 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0001]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b0000_0000]),
            // Port 1 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x03], vec![0b1000_0000]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x03, 0b0000_0000]),
        ];
        let i2cbus = Mock::new(&expectations);
        let dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let mut dev = SharedDevice::new(dev);

        {
            let pins: [IoPin<
                '_,
                embedded_hal_mock::common::Generic<Transaction>,
                embassy_sync::blocking_mutex::raw::NoopRawMutex,
            >; 16] = dev.split();
            pins[0].set_low_async().await.unwrap();
            pins[15].set_low_async().await.unwrap();
        }

        dev.device.lock().await.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn async_pin_is_high() {
        let expectations = vec![
            // Port 0 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x00], vec![0b0000_0001]),
            // Port 1 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x01], vec![0b1000_0000]),
        ];
        let i2cbus = Mock::new(&expectations);
        let dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let mut dev = SharedDevice::new(dev);

        {
            let pins: [IoPin<
                '_,
                embedded_hal_mock::common::Generic<Transaction>,
                embassy_sync::blocking_mutex::raw::NoopRawMutex,
            >; 16] = dev.split();
            assert!(pins[0].is_high_async().await.unwrap());
            assert!(pins[15].is_high_async().await.unwrap());
        }

        dev.device.lock().await.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn async_pin_is_low() {
        let expectations = vec![
            // Port 0 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x00], vec![0b0000_0000]),
            // Port 1 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x01], vec![0b0000_0000]),
        ];
        let i2cbus = Mock::new(&expectations);
        let dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let mut dev = SharedDevice::new(dev);

        {
            let pins: [IoPin<
                '_,
                embedded_hal_mock::common::Generic<Transaction>,
                embassy_sync::blocking_mutex::raw::NoopRawMutex,
            >; 16] = dev.split();
            assert!(pins[0].is_low_async().await.unwrap());
            assert!(pins[15].is_low_async().await.unwrap());
        }

        dev.device.lock().await.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn async_pin_toggle() {
        let expectations = vec![
            // Port 0 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0001]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b0000_0000]),
            // Port 1 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x03], vec![0b1000_0000]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x03, 0b0000_0000]),
        ];
        let i2cbus = Mock::new(&expectations);
        let dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let mut dev = SharedDevice::new(dev);

        {
            let pins: [IoPin<
                '_,
                embedded_hal_mock::common::Generic<Transaction>,
                embassy_sync::blocking_mutex::raw::NoopRawMutex,
            >; 16] = dev.split();
            pins[0].toggle_async().await.unwrap();
            pins[15].toggle_async().await.unwrap();
        }

        dev.device.lock().await.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn async_pin_is_set_high() {
        let expectations = vec![
            // Port 0 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0001]),
            // Port 1 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x03], vec![0b1000_0000]),
        ];
        let i2cbus = Mock::new(&expectations);
        let dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let mut dev = SharedDevice::new(dev);

        {
            let pins: [IoPin<
                '_,
                embedded_hal_mock::common::Generic<Transaction>,
                embassy_sync::blocking_mutex::raw::NoopRawMutex,
            >; 16] = dev.split();
            assert!(pins[0].is_set_high_async().await.unwrap());
            assert!(pins[15].is_set_high_async().await.unwrap());
        }

        dev.device.lock().await.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn async_pin_is_set_low() {
        let expectations = vec![
            // Port 0 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0000]),
            // Port 1 pin
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x03], vec![0b0000_0000]),
        ];
        let i2cbus = Mock::new(&expectations);
        let dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let mut dev = SharedDevice::new(dev);

        {
            let pins: [IoPin<
                '_,
                embedded_hal_mock::common::Generic<Transaction>,
                embassy_sync::blocking_mutex::raw::NoopRawMutex,
            >; 16] = dev.split();
            assert!(pins[0].is_set_low_async().await.unwrap());
            assert!(pins[15].is_set_low_async().await.unwrap());
        }

        dev.device.lock().await.interface.i2cbus.done();
    }

    #[tokio::test]
    async fn embedded_hal_async_traits() {
        use embedded_hal_async::digital::{InputPin, OutputPin, StatefulOutputPin};

        let expectations = vec![
            // OutputPin::set_high
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0000]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b0000_0001]),
            // OutputPin::set_low
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0001]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b0000_0000]),
            // InputPin::is_high (when high)
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x00], vec![0b0000_0001]),
            // InputPin::is_low (when high)
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x00], vec![0b0000_0001]),
            // InputPin::is_high (when low)
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x00], vec![0b0000_0000]),
            // InputPin::is_low (when low)
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x00], vec![0b0000_0000]),
            // StatefulOutputPin::is_set_high (when low)
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0000]),
            // StatefulOutputPin::is_set_low (when low)
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0000]),
            // StatefulOutputPin::toggle (low to high)
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0000]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b0000_0001]),
            // StatefulOutputPin::is_set_high (when high)
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0001]),
            // StatefulOutputPin::is_set_low (when high)
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0001]),
            // StatefulOutputPin::toggle (high to low)
            Transaction::write_read(IOEXP_ADDR_LOW, vec![0x02], vec![0b0000_0001]),
            Transaction::write(IOEXP_ADDR_LOW, vec![0x02, 0b0000_0000]),
        ];
        let i2cbus = Mock::new(&expectations);
        let mut dev = Device::new(Pcal6416aDevice {
            addr_pin: AddrPinState::Low,
            i2cbus,
        });
        let mut dev = SharedDevice::new(dev);

        {
            let mut pins: [IoPin<
                '_,
                embedded_hal_mock::common::Generic<Transaction>,
                embassy_sync::blocking_mutex::raw::NoopRawMutex,
            >; 16] = dev.split();

            // Test OutputPin trait - set_high
            OutputPin::set_high(&mut pins[0]).await.unwrap();

            // Test OutputPin trait - set_low
            OutputPin::set_low(&mut pins[0]).await.unwrap();

            // Test InputPin trait - is_high when pin is high
            assert!(InputPin::is_high(&mut pins[0]).await.unwrap());

            // Test InputPin trait - is_low when pin is high
            assert!(!InputPin::is_low(&mut pins[0]).await.unwrap());

            // Test InputPin trait - is_high when pin is low
            assert!(!InputPin::is_high(&mut pins[0]).await.unwrap());

            // Test InputPin trait - is_low when pin is low
            assert!(InputPin::is_low(&mut pins[0]).await.unwrap());

            // Test StatefulOutputPin trait - is_set_high when output is low
            assert!(!StatefulOutputPin::is_set_high(&mut pins[0]).await.unwrap());

            // Test StatefulOutputPin trait - is_set_low when output is low
            assert!(StatefulOutputPin::is_set_low(&mut pins[0]).await.unwrap());

            // Test StatefulOutputPin trait - toggle from low to high
            StatefulOutputPin::toggle(&mut pins[0]).await.unwrap();

            // Test StatefulOutputPin trait - is_set_high when output is high
            assert!(StatefulOutputPin::is_set_high(&mut pins[0]).await.unwrap());

            // Test StatefulOutputPin trait - is_set_low when output is high
            assert!(!StatefulOutputPin::is_set_low(&mut pins[0]).await.unwrap());

            // Test StatefulOutputPin trait - toggle from high to low
            StatefulOutputPin::toggle(&mut pins[0]).await.unwrap();
        }

        dev.device.lock().await.interface.i2cbus.done();
    }
}
