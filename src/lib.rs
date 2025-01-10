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
#![no_std]
#![allow(missing_docs)]

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Pcal6416aError<E> {
    /// I2C bus error
    I2c(E),
}
const IOEXP_ADDR: u8 = 0x20;
const LARGEST_REG_SIZE_BYTES: usize = 2;

pub struct Pcal6416aDevice<I2c: embedded_hal_async::i2c::I2c> {
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
            .write(IOEXP_ADDR, &buf[..=data.len()])
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
            .write_read(IOEXP_ADDR, &[address], data)
            .await
            .map_err(Pcal6416aError::I2c)
    }
}
