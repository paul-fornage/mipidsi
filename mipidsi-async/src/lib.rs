#![no_std]

//! Async SPI drivers for ST7789 and ILI9341 displays in RGB565 mode.
//!
//! Uses the async `DrawTarget` from the embedded-graphics fork. Drawing writes
//! directly to the display; there is no framebuffer or separate flush step.
//! Adjacent pixels in row-major order share an address window and use buffered
//! SPI transfers. The `batch` feature also buffers up to 128 isolated pixels
//! to combine them after sorting. All pending pixels are written before
//! `draw_iter` returns. Rectangle fills always use buffered transfers.
//! SPI and delays use embedded-hal-async. D/C and reset use embedded-hal's
//! `OutputPin`, since embedded-hal-async doesn't have a digital pin trait.
//!
//! The SPI device should use mode 0 and handle chip select. For example, with
//! your SPI device, output pins, and async delay provider:
//!
//! ```ignore
//! use embedded_graphics::{pixelcolor::Rgb565, prelude::*};
//! use mipidsi_async::{Rotation, ST7789};
//!
//! let mut display = ST7789::new(spi, dc, reset, 240, 240);
//! // Set the panel's offset in the controller's 240 x 320 memory if needed.
//! display.set_offset(0, 80);
//! display.init(&mut delay).await?;
//! // Rotate clockwise; drawing coordinates and dimensions follow the rotation.
//! display.set_rotation(Rotation::Deg90).await?;
//! // Some panels require inversion to show the expected colors.
//! display.set_invert_colors(true).await?;
//! display.clear(Rgb565::BLACK).await?;
//! // Embedded-graphics draw calls work the same way:
//! // drawable.draw(&mut display).await?;
//! ```
//!
//! For an ILI9341, use `ILI9341::new(spi, dc, reset, 240, 320)` with the same
//! async initialization, rotation, inversion, and drawing methods.

mod display;
pub mod ili9341;
pub mod st7789;

pub use display::{Display, Error, Rotation};
pub use ili9341::ILI9341;
pub use st7789::ST7789;
