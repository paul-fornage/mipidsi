#![no_std]

//! A small async SPI driver for an ST7789 in RGB565 mode.
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

use embedded_graphics_core::{
    draw_target::DrawTarget,
    geometry::{Dimensions, OriginDimensions, Size},
    pixelcolor::{
        raw::{RawData, RawU16},
        Rgb565,
    },
    primitives::Rectangle,
    Pixel,
};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};

const MEMORY_WIDTH: u16 = 240;
const MEMORY_HEIGHT: u16 = 320;
const BUFFER_BYTES: usize = 256;

#[cfg(feature = "batch")]
type PixelBatch = heapless::Vec<(usize, Pixel<Rgb565>), { BUFFER_BYTES / 2 }>;

/// Display content rotation relative to the panel's portrait orientation.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Rotation {
    /// No rotation.
    #[default]
    Deg0,
    /// 90° clockwise rotation.
    Deg90,
    /// 180° clockwise rotation.
    Deg180,
    /// 270° clockwise rotation.
    Deg270,
}

impl Rotation {
    // MADCTL: MY (bit 7), MX (bit 6), MV (bit 5), RGB color order (bit 3 clear).
    const fn madctl(self) -> u8 {
        match self {
            Self::Deg0 => 0x00,
            Self::Deg90 => 0x60,
            Self::Deg180 => 0xc0,
            Self::Deg270 => 0xa0,
        }
    }
}

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
    rotation: Rotation,
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
            rotation: Rotation::Deg0,
        }
    }

    /// Sets the visible panel's offset in the controller's display RAM.
    ///
    /// Coordinates are relative to portrait orientation, regardless of the
    /// current rotation. Drawing automatically transforms these offsets.
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

    /// Resets the controller and turns on the display in RGB565 mode.
    ///
    /// Uses the same command sequence and post-reset delays as the synchronous
    /// ST7789 model. Initialization disables color inversion and doesn't clear
    /// display RAM. The current rotation is reapplied (portrait by default).
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
        self.set_rotation(self.rotation).await?;
        self.set_invert_colors(false).await?;
        self.command(0x3a, &[0x55]).await?; // COLMOD: 16-bit RGB565
        delay.delay_ms(10).await;
        self.command(0x13, &[]).await?; // NORON
        delay.delay_ms(10).await;
        self.command(0x29, &[]).await?; // DISPON
        delay.delay_ms(120).await;

        Ok(())
    }

    /// Returns the current display rotation.
    pub fn rotation(&self) -> Rotation {
        self.rotation
    }

    /// Sets the clockwise rotation of subsequent drawing operations.
    ///
    /// Drawing coordinates start at the rotated panel's top left. At 90° and
    /// 270°, the dimensions reported to embedded-graphics are swapped. Panel
    /// dimensions and offsets remain defined in portrait orientation.
    ///
    /// This changes the controller's address mapping without clearing RAM or
    /// rotating existing content; redraw the display after changing rotation.
    pub async fn set_rotation(
        &mut self,
        rotation: Rotation,
    ) -> Result<(), Error<SPI::Error, DC::Error, RST::Error>> {
        self.command(0x36, &[rotation.madctl()]).await?; // MADCTL
        self.rotation = rotation;
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
        // Reverse native offsets before swapping axes, just like MADCTL.
        let (x_offset, y_offset) = match self.rotation {
            Rotation::Deg0 => (self.x_offset, self.y_offset),
            Rotation::Deg90 => (self.y_offset, MEMORY_WIDTH - self.width - self.x_offset),
            Rotation::Deg180 => (
                MEMORY_WIDTH - self.width - self.x_offset,
                MEMORY_HEIGHT - self.height - self.y_offset,
            ),
            Rotation::Deg270 => (MEMORY_HEIGHT - self.height - self.y_offset, self.x_offset),
        };
        let x0 = (x0 + x_offset).to_be_bytes();
        let x1 = (x1 + x_offset).to_be_bytes();
        let y0 = (y0 + y_offset).to_be_bytes();
        let y1 = (y1 + y_offset).to_be_bytes();

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
        let mut colors = colors.into_iter();
        let mut buffer = [0u8; BUFFER_BYTES];
        let mut data_mode = false;

        loop {
            let mut used = 0;
            for pixel in buffer.chunks_exact_mut(2) {
                let Some(color) = colors.next() else {
                    break;
                };
                pixel.copy_from_slice(&RawU16::from(color).into_inner().to_be_bytes());
                used += 2;
            }

            if used == 0 {
                break;
            }

            // D/C stays high for the entire stream, including across chunks.
            if !data_mode {
                self.dc.set_high().map_err(Error::Dc)?;
                data_mode = true;
            }
            self.spi.write(&buffer[..used]).await.map_err(Error::Spi)?;

            if used < buffer.len() {
                break;
            }
        }
        Ok(())
    }

    #[cfg(feature = "batch")]
    async fn flush_batch(
        &mut self,
        batch: &mut PixelBatch,
    ) -> Result<(), Error<SPI::Error, DC::Error, RST::Error>> {
        if batch.is_empty() {
            return Ok(());
        }

        // Keep repeated writes to the same pixel in their original order so
        // that the last color still wins, even though other pixels are sorted.
        batch
            .as_mut_slice()
            .sort_unstable_by_key(|(order, Pixel(point, _))| (point.y, point.x, *order));

        let mut first = 0;
        while first < batch.len() {
            let Pixel(start, _) = batch[first].1;
            let mut end = start;
            let mut last = first + 1;
            while let Some((_, Pixel(point, _))) = batch.get(last) {
                if point.y != end.y || point.x != end.x + 1 {
                    break;
                }
                end = *point;
                last += 1;
            }

            self.set_window(start.x as u16, start.y as u16, end.x as u16, end.y as u16)
                .await?;
            if last - first == 1 {
                let Pixel(_, color) = batch[first].1;
                self.write_data(&RawU16::from(color).into_inner().to_be_bytes())
                    .await?;
            } else {
                self.write_pixels(
                    batch.as_slice()[first..last]
                        .iter()
                        .map(|(_, Pixel(_, color))| *color),
                )
                .await?;
            }
            first = last;
        }
        batch.clear();
        Ok(())
    }
}

impl<SPI, DC, RST> OriginDimensions for ST7789<SPI, DC, RST> {
    fn size(&self) -> Size {
        let (width, height) = match self.rotation {
            Rotation::Deg0 | Rotation::Deg180 => (self.width, self.height),
            Rotation::Deg90 | Rotation::Deg270 => (self.height, self.width),
        };
        Size::new(u32::from(width), u32::from(height))
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
        let bounds = self.bounding_box();
        let right = bounds.size.width as i32 - 1;
        let bottom = bounds.size.height as i32 - 1;
        let mut pixels = pixels
            .into_iter()
            .filter(|Pixel(point, _)| bounds.contains(*point))
            .peekable();
        #[cfg(feature = "batch")]
        let mut batch = PixelBatch::new();

        while let Some(Pixel(start, color)) = pixels.next() {
            // A full-width window can stream across row boundaries. A run
            // starting partway along a row must finish that row first.
            let end_y = if start.x == 0 { bottom } else { start.y };

            let mut expected = start;
            let mut following_colors = core::iter::from_fn(|| {
                if expected.x == right {
                    expected.x = 0;
                    expected.y += 1;
                } else {
                    expected.x += 1;
                }

                match pixels.peek() {
                    Some(Pixel(point, _)) if *point == expected && expected.y <= end_y => {
                        pixels.next().map(|Pixel(_, color)| color)
                    }
                    _ => None,
                }
            });

            if let Some(next_color) = following_colors.next() {
                // Known contiguous runs stream immediately with either feature
                // setting. Older pending writes must reach the panel first.
                #[cfg(feature = "batch")]
                self.flush_batch(&mut batch).await?;
                self.set_window(start.x as u16, start.y as u16, right as u16, end_y as u16)
                    .await?;
                self.write_pixels(
                    core::iter::once(color)
                        .chain(core::iter::once(next_color))
                        .chain(following_colors),
                )
                .await?;
            } else {
                #[cfg(feature = "batch")]
                {
                    let order = batch.len();
                    batch
                        .push((order, Pixel(start, color)))
                        .expect("batch is flushed when full");
                    if batch.is_full() {
                        self.flush_batch(&mut batch).await?;
                    }
                }
                #[cfg(not(feature = "batch"))]
                {
                    // Isolated pixels don't need a full transfer buffer.
                    let x = start.x as u16;
                    let y = start.y as u16;
                    self.set_window(x, y, x, y).await?;
                    self.write_data(&RawU16::from(color).into_inner().to_be_bytes())
                        .await?;
                }
            }
        }
        #[cfg(feature = "batch")]
        self.flush_batch(&mut batch).await?;
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

        // Skip clipped rows and columns without constructing a point for every
        // input color. Use u64 for offsets: a large off-screen rectangle can
        // have more than usize::MAX pixels on a 32-bit target.
        let count = (clipped.size.width * clipped.size.height) as usize;
        let mut colors = colors.into_iter();
        let top = (i64::from(clipped.top_left.y) - i64::from(area.top_left.y)) as u64;
        let left = (i64::from(clipped.top_left.x) - i64::from(area.top_left.x)) as u64;
        let mut skip = top * u64::from(area.size.width) + left;

        while skip != 0 {
            let chunk = skip.min(usize::MAX as u64) as usize;
            if colors.nth(chunk - 1).is_none() {
                return Ok(());
            }
            skip -= chunk as u64;
        }

        let row_skip = (area.size.width - clipped.size.width) as usize;
        if row_skip == 0 {
            // The usual case: directly stream an unclipped rectangle or one
            // clipped only at the top and bottom.
            return self.write_pixels(colors.take(count)).await;
        }

        let mut remaining_in_row = clipped.size.width;
        let visible_colors = core::iter::from_fn(|| {
            if remaining_in_row == 0 {
                colors.nth(row_skip - 1)?;
                remaining_in_row = clipped.size.width;
            }
            let color = colors.next()?;
            remaining_in_row -= 1;
            Some(color)
        })
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

        // Encode the color once and reuse the same filled buffer for every
        // transfer instead of converting and packing each repeated pixel.
        let mut remaining = (clipped.size.width * clipped.size.height) as usize;
        let chunk_pixels = remaining.min(BUFFER_BYTES / 2);
        let bytes = RawU16::from(color).into_inner().to_be_bytes();
        let mut buffer = [0u8; BUFFER_BYTES];
        for pixel in buffer[..chunk_pixels * 2].chunks_exact_mut(2) {
            pixel.copy_from_slice(&bytes);
        }

        self.dc.set_high().map_err(Error::Dc)?;
        while remaining != 0 {
            let pixels = remaining.min(chunk_pixels);
            self.spi
                .write(&buffer[..pixels * 2])
                .await
                .map_err(Error::Spi)?;
            remaining -= pixels;
        }
        Ok(())
    }
}
