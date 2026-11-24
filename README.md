# PCAL6416A Rust Device Driver

A `#[no_std]` platform-agnostic driver for the [PCAL6416A](https://www.nxp.com/docs/en/data-sheet/PCAL6416A.pdf) IO Expander

## MSRV

Currently, rust `1.88` and up is supported.

## HAL source

Git/path consumers use the paired `embedded-hal` and `embedded-hal-async`
revision `e6b10a6623db89406784044a4a432cd2fec66a65` for the unreleased async
digital traits. Consumers and their HAL implementations must use the same Git
URL and revision selector as this crate; registry or default-branch Git trait
types are not interchangeable with these pinned types. Registry publication
still uses the version fallback, which does not supply those async traits.

## License

Licensed under the terms of the [MIT license](http://opensource.org/licenses/MIT).

## Contribution

Unless you explicitly state otherwise, any contribution submitted for
inclusion in the work by you shall be licensed under the terms of the
MIT license.

License: MIT
