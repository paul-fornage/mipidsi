//! ILI9341 controller support in RGB565 mode.

use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};

use crate::{Display, Error};

/// An ILI9341 connected to an async 8-bit SPI device, D/C pin, and reset pin.
pub type ILI9341<SPI, DC, RST> = Display<SPI, DC, RST, true>;

impl<SPI, DC, RST> Display<SPI, DC, RST, true>
where
    SPI: SpiDevice<u8>,
    DC: OutputPin,
    RST: OutputPin,
{
    /// Resets the ILI9341 and turns on the display in RGB565 mode.
    ///
    /// Uses the synchronous ILI9341 model's command sequence and post-reset
    /// delays. Disables color inversion and reapplies the current orientation
    /// (unmirrored portrait by default), without clearing display RAM.
    pub async fn init(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error<SPI::Error, DC::Error, RST::Error>> {
        self.hardware_reset(delay).await?;
        delay.delay_ms(5).await;
        self.set_orientation(self.orientation()).await?;
        self.command(0xb4, &[0x00]).await?; // Display inversion control
        self.set_invert_colors(false).await?;
        self.command(0x3a, &[0x55]).await?; // COLMOD: 16-bit RGB565
        self.command(0x13, &[]).await?; // NORON
        // Reset can implicitly enter sleep; wait before leaving it.
        delay.delay_ms(120).await;
        self.command(0x11, &[]).await?; // SLPOUT
        delay.delay_ms(140).await;
        self.command(0x29, &[]).await?; // DISPON
        Ok(())
    }
}
