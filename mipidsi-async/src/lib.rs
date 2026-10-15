#![no_std]

//! A small async SPI driver for an ST7789 in portrait RGB565 mode.
//!
//! Uses the async `DrawTarget` from the embedded-graphics fork. Drawing writes
//! directly to the display; there is no framebuffer or separate flush step.
//! SPI and delays use embedded-hal-async. D/C and reset use embedded-hal's
//! `OutputPin`, since embedded-hal-async doesn't have a digital pin trait.
//!
//! The SPI device should use mode 0 and handle chip select. For example, with
//! your SPI device, output pins, and async delay provider:
//!
//! ```ignore
//! use embedded_graphics::{pixelcolor::Rgb565, prelude::*};
//! use mipidsi_async::ST7789;
//!
//! let mut display = ST7789::new(spi, dc, reset, 240, 240);
//! // Set the panel's offset in the controller's 240 x 320 memory if needed.
//! display.set_offset(0, 80);
//! display.init(&mut delay).await?;
//! // Some panels require inversion to show the expected colors.
//! display.set_invert_colors(true).await?;
//! display.clear(Rgb565::BLACK).await?;
//! // Embedded-graphics draw calls work the same way:
//! // drawable.draw(&mut display).await?;
//! ```

use embedded_graphics_core::{
    draw_target::DrawTarget,
    geometry::{Dimensions, OriginDimensions, Size},
    pixelcolor::{
        raw::{RawData, RawU16},
        Rgb565,
    },
    primitives::{PointsIter, Rectangle},
    Pixel,
};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};

const MEMORY_WIDTH: u16 = 240;
const MEMORY_HEIGHT: u16 = 320;
const BUFFER_BYTES: usize = 256;

/// An error from the SPI device or a control pin.
#[derive(Debug)]
pub enum Error<SpiError, DcError, ResetError> {
    /// SPI transfer failed.
    Spi(SpiError),
    /// Setting the D/C pin failed.
    Dc(DcError),
    /// Setting the reset pin failed.
    Reset(ResetError),
}

/// An ST7789 connected to an async 8-bit SPI device, D/C pin, and reset pin.
///
/// Coordinates start at the top left of the visible panel. Smaller panels can
/// use [`set_offset`](Self::set_offset) to select their region of display RAM.
pub struct ST7789<SPI, DC, RST> {
    spi: SPI,
    dc: DC,
    reset: RST,
    width: u16,
    height: u16,
    x_offset: u16,
    y_offset: u16,
}

impl<SPI, DC, RST> ST7789<SPI, DC, RST>
where
    SPI: SpiDevice<u8>,
    DC: OutputPin,
    RST: OutputPin,
{
    /// Creates a driver without touching the hardware.
    ///
    /// Call [`init`](Self::init) before drawing. Width and height describe the
    /// visible panel in portrait orientation.
    ///
    /// # Panics
    ///
    /// Panics if width is outside `1..=240` or height is outside `1..=320`.
    pub fn new(spi: SPI, dc: DC, reset: RST, width: u16, height: u16) -> Self {
        assert!(width > 0 && width <= MEMORY_WIDTH);
        assert!(height > 0 && height <= MEMORY_HEIGHT);

        Self {
            spi,
            dc,
            reset,
            width,
            height,
            x_offset: 0,
            y_offset: 0,
        }
    }

    /// Sets the visible panel's offset in the controller's display RAM.
    ///
    /// This only changes the addresses used by subsequent drawing operations.
    ///
    /// # Panics
    ///
    /// Panics if the panel would extend beyond the 240 x 320 display RAM.
    pub fn set_offset(&mut self, x: u16, y: u16) {
        assert!(x <= MEMORY_WIDTH - self.width);
        assert!(y <= MEMORY_HEIGHT - self.height);

        self.x_offset = x;
        self.y_offset = y;
    }

    /// Resets the controller and turns on the display in portrait RGB565 mode.
    ///
    /// Uses the same command sequence and post-reset delays as the synchronous
    /// ST7789 model. Initialization disables color inversion and doesn't clear
    /// display RAM.
    pub async fn init(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error<SPI::Error, DC::Error, RST::Error>> {
        self.reset.set_high().map_err(Error::Reset)?;
        delay.delay_ms(10).await;
        self.reset.set_low().map_err(Error::Reset)?;
        delay.delay_ms(10).await;
        self.reset.set_high().map_err(Error::Reset)?;
        delay.delay_ms(150).await;

        self.command(0x11, &[]).await?; // SLPOUT
        delay.delay_ms(10).await;
        self.command(0x36, &[0x00]).await?; // MADCTL: portrait, RGB order
        self.set_invert_colors(false).await?;
        self.command(0x3a, &[0x55]).await?; // COLMOD: 16-bit RGB565
        delay.delay_ms(10).await;
        self.command(0x13, &[]).await?; // NORON
        delay.delay_ms(10).await;
        self.command(0x29, &[]).await?; // DISPON
        delay.delay_ms(120).await;

        Ok(())
    }

    /// Enables or disables the panel's color inversion mode.
    pub async fn set_invert_colors(
        &mut self,
        invert: bool,
    ) -> Result<(), Error<SPI::Error, DC::Error, RST::Error>> {
        self.command(if invert { 0x21 } else { 0x20 }, &[]).await
    }

    /// Returns the SPI device, D/C pin, and reset pin.
    pub fn release(self) -> (SPI, DC, RST) {
        (self.spi, self.dc, self.reset)
    }

    async fn command(
        &mut self,
        command: u8,
        data: &[u8],
    ) -> Result<(), Error<SPI::Error, DC::Error, RST::Error>> {
        self.dc.set_low().map_err(Error::Dc)?;
        self.spi.write(&[command]).await.map_err(Error::Spi)?;
        if !data.is_empty() {
            self.write_data(data).await?;
        }
        Ok(())
    }

    async fn write_data(
        &mut self,
        data: &[u8],
    ) -> Result<(), Error<SPI::Error, DC::Error, RST::Error>> {
        self.dc.set_high().map_err(Error::Dc)?;
        self.spi.write(data).await.map_err(Error::Spi)
    }

    // Coordinates are inclusive, already clipped, and relative to the panel.
    async fn set_window(
        &mut self,
        x0: u16,
        y0: u16,
        x1: u16,
        y1: u16,
    ) -> Result<(), Error<SPI::Error, DC::Error, RST::Error>> {
        let x0 = (x0 + self.x_offset).to_be_bytes();
        let x1 = (x1 + self.x_offset).to_be_bytes();
        let y0 = (y0 + self.y_offset).to_be_bytes();
        let y1 = (y1 + self.y_offset).to_be_bytes();

        self.command(0x2a, &[x0[0], x0[1], x1[0], x1[1]])
            .await?; // CASET
        self.command(0x2b, &[y0[0], y0[1], y1[0], y1[1]])
            .await?; // RASET
        self.command(0x2c, &[]).await // RAMWR
    }

    async fn write_pixels(
        &mut self,
        colors: impl IntoIterator<Item = Rgb565>,
    ) -> Result<(), Error<SPI::Error, DC::Error, RST::Error>> {
        let mut buffer = [0u8; BUFFER_BYTES];
        let mut used = 0;

        for color in colors {
            let bytes = RawU16::from(color).into_inner().to_be_bytes();
            buffer[used] = bytes[0];
            buffer[used + 1] = bytes[1];
            used += 2;

            if used == buffer.len() {
                self.write_data(&buffer).await?;
                used = 0;
            }
        }

        if used != 0 {
            self.write_data(&buffer[..used]).await?;
        }
        Ok(())
    }
}

impl<SPI, DC, RST> OriginDimensions for ST7789<SPI, DC, RST> {
    fn size(&self) -> Size {
        Size::new(u32::from(self.width), u32::from(self.height))
    }
}

impl<SPI, DC, RST> DrawTarget for ST7789<SPI, DC, RST>
where
    SPI: SpiDevice<u8>,
    DC: OutputPin,
    RST: OutputPin,
{
    type Color = Rgb565;
    type Error = Error<SPI::Error, DC::Error, RST::Error>;

    async fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            if point.x < 0
                || point.y < 0
                || point.x >= i32::from(self.width)
                || point.y >= i32::from(self.height)
            {
                continue;
            }

            let x = point.x as u16;
            let y = point.y as u16;
            self.set_window(x, y, x, y).await?;
            self.write_data(&RawU16::from(color).into_inner().to_be_bytes())
                .await?;
        }
        Ok(())
    }

    async fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        let clipped = area.intersection(&self.bounding_box());
        let Some(bottom_right) = clipped.bottom_right() else {
            return Ok(());
        };

        self.set_window(
            clipped.top_left.x as u16,
            clipped.top_left.y as u16,
            bottom_right.x as u16,
            bottom_right.y as u16,
        )
        .await?;

        // Keep colors aligned with the original rectangle when clipping. Stop
        // after the last visible pixel, or earlier if the iterator runs out.
        let count = (clipped.size.width * clipped.size.height) as usize;
        let visible_colors = area
            .points()
            .zip(colors)
            .filter_map(|(point, color)| clipped.contains(point).then_some(color))
            .take(count);

        self.write_pixels(visible_colors).await
    }

    async fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        let clipped = area.intersection(&self.bounding_box());
        let Some(bottom_right) = clipped.bottom_right() else {
            return Ok(());
        };

        self.set_window(
            clipped.top_left.x as u16,
            clipped.top_left.y as u16,
            bottom_right.x as u16,
            bottom_right.y as u16,
        )
        .await?;

        let count = (clipped.size.width * clipped.size.height) as usize;
        self.write_pixels(core::iter::repeat(color).take(count)).await
    }
}
