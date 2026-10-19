//! ST7789 controller support in RGB565 mode.

use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};

use crate::{Display, Error};

/// An ST7789 connected to an async 8-bit SPI device, D/C pin, and reset pin.
pub type ST7789<SPI, DC, RST> = Display<SPI, DC, RST, false>;

impl<SPI, DC, RST> Display<SPI, DC, RST, false>
where
    SPI: SpiDevice<u8>,
    DC: OutputPin,
    RST: OutputPin,
{
    /// Resets the ST7789 and turns on the display in RGB565 mode.
    ///
    /// Uses the synchronous ST7789 model's command sequence and post-reset
    /// delays. Disables color inversion and reapplies the current rotation
    /// (portrait by default), without clearing display RAM.
    pub async fn init(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error<SPI::Error, DC::Error, RST::Error>> {
        self.hardware_reset(delay).await?;
        delay.delay_ms(150).await;
        self.command(0x11, &[]).await?; // SLPOUT
        delay.delay_ms(10).await;
        self.set_rotation(self.rotation()).await?;
        self.set_invert_colors(false).await?;
        self.command(0x3a, &[0x55]).await?; // COLMOD: 16-bit RGB565
        delay.delay_ms(10).await;
        self.command(0x13, &[]).await?; // NORON
        delay.delay_ms(10).await;
        self.command(0x29, &[]).await?; // DISPON
        delay.delay_ms(120).await;
        Ok(())
    }
}
