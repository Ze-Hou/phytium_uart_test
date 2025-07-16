#![no_std]

extern crate alloc;

pub mod mutex;

mod pl011;
pub use pl011::Pl011Uart;
