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

#[cfg(test)]
mod tests {
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    use super::*;

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
}
