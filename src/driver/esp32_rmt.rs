#![cfg_attr(not(target_vendor = "espressif"), allow(dead_code))]

use core::error::Error;
use core::fmt;
use core::time::Duration;

#[cfg(not(target_vendor = "espressif"))]
use core::marker::PhantomData;

#[cfg(not(target_vendor = "espressif"))]
use crate::mock::esp_idf_hal;
#[cfg(target_vendor = "espressif")]
use esp_idf_hal::rmt::{encoder::{BytesEncoder, BytesEncoderConfig}, PinState, Pulse, Symbol};
#[cfg(all(not(feature = "std"), feature = "alloc"))]
use alloc::vec::Vec;
use esp_idf_hal::{
    gpio::OutputPin,
    rmt::{config::TxChannelConfig, TxChannelDriver},
    units::Hertz,
};

#[cfg(target_vendor = "espressif")]
use esp_idf_hal::rmt::config::TransmitConfig;

#[cfg(not(target_vendor = "espressif"))]
use crate::mock::esp_idf_sys;
use esp_idf_sys::EspError;

/// T0H duration time (0 code, high voltage time)
const WS2812_T0H_NS: Duration = Duration::from_nanos(400);
/// T0L duration time (0 code, low voltage time)
const WS2812_T0L_NS: Duration = Duration::from_nanos(850);
/// T1H duration time (1 code, high voltage time)
const WS2812_T1H_NS: Duration = Duration::from_nanos(800);
/// T1L duration time (1 code, low voltage time)
const WS2812_T1L_NS: Duration = Duration::from_nanos(450);

/// Converter to a sequence of RMT items.
#[repr(C)]
struct Ws2812Esp32RmtItemEncoder {
    /// The RMT item that represents a 0 code.
    #[cfg(target_vendor = "espressif")]
    bit0: Symbol,
    /// The RMT item that represents a 1 code.
    #[cfg(target_vendor = "espressif")]
    bit1: Symbol,
}

impl Ws2812Esp32RmtItemEncoder {
    /// Creates a new `Ws2812Esp32RmtItemEncoder`.
    fn new(
        clock_hz: Hertz,
        t0h: &Duration,
        t0l: &Duration,
        t1h: &Duration,
        t1l: &Duration,
    ) -> Result<Self, Ws2812Esp32RmtDriverError> {
        #[cfg(target_vendor = "espressif")]
        {
            let (bit0, bit1) = (
                Symbol::new(
                    Pulse::new_with_duration(clock_hz, PinState::High, *t0h)?,
                    Pulse::new_with_duration(clock_hz, PinState::Low, *t0l)?,
                ),
                Symbol::new(
                    Pulse::new_with_duration(clock_hz, PinState::High, *t1h)?,
                    Pulse::new_with_duration(clock_hz, PinState::Low, *t1l)?,
                ),
            );
            Ok(Self { bit0, bit1 })
        }
        #[cfg(not(target_vendor = "espressif"))]
        {
            let _ = (clock_hz, t0h, t0l, t1h, t1l);
            Ok(Self {})
        }
    }
}

/// WS2812 ESP32 RMT Driver error.
#[derive(Debug)]
#[repr(transparent)]
pub struct Ws2812Esp32RmtDriverError {
    source: EspError,
}

#[cfg(not(feature = "std"))]
impl Ws2812Esp32RmtDriverError {
    /// The `EspError` source of this error, if any.
    ///
    /// This is a workaround function until `core::error::Error` added to `esp_sys::EspError`.
    pub fn source(&self) -> Option<&EspError> {
        Some(&self.source)
    }
}

impl Error for Ws2812Esp32RmtDriverError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        #[cfg(feature = "std")]
        {
            Some(&self.source)
        }
        #[cfg(not(feature = "std"))]
        {
            None
        }
    }
}

impl fmt::Display for Ws2812Esp32RmtDriverError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.source.fmt(f)
    }
}

impl From<EspError> for Ws2812Esp32RmtDriverError {
    fn from(source: EspError) -> Self {
        Self { source }
    }
}

/// Builder for `Ws2812Esp32RmtDriver`.
///
/// # Examples
///
///
/// ```
/// # #[cfg(not(target_vendor = "espressif"))]
/// # use ws2812_esp32_rmt_driver::mock::esp_idf_hal;
/// #
/// # use core::time::Duration;
/// # use esp_idf_hal::peripherals::Peripherals;
/// # use esp_idf_hal::rmt::config::{TransmitConfig, TxChannelConfig};
/// # use esp_idf_hal::rmt::TxChannelDriver;
/// # use esp_idf_hal::units::Hertz;
/// # use ws2812_esp32_rmt_driver::driver::Ws2812Esp32RmtDriverBuilder;
/// #
/// # let peripherals = Peripherals::take().unwrap();
/// # let led_pin = peripherals.pins.gpio27;
///
/// // WS2812B timing parameters.
/// const WS2812_T0H_NS: Duration = Duration::from_nanos(400);
/// const WS2812_T0L_NS: Duration = Duration::from_nanos(850);
/// const WS2812_T1H_NS: Duration = Duration::from_nanos(800);
/// const WS2812_T1L_NS: Duration = Duration::from_nanos(450);
///
/// let channel_config = TxChannelConfig { resolution: Hertz(80_000_000), ..Default::default() };
/// let tx_driver = TxChannelDriver::new(led_pin, &channel_config).unwrap();
/// let driver = Ws2812Esp32RmtDriverBuilder::new_with_rmt_driver(tx_driver).unwrap()
///    .encoder_duration(&WS2812_T0H_NS, &WS2812_T0L_NS, &WS2812_T1H_NS, &WS2812_T1L_NS).unwrap()
///    .build().unwrap();
/// ```
pub struct Ws2812Esp32RmtDriverBuilder<'d> {
    /// TxRMT driver.
    tx: TxChannelDriver<'d>,
    /// Resolution of the RMT channel clock (used for pulse timing).
    resolution: Hertz,
    /// `u8`-to-`rmt_item32_t` Encoder
    encoder: Option<Ws2812Esp32RmtItemEncoder>,
}

impl<'d> Ws2812Esp32RmtDriverBuilder<'d> {
    /// Creates a new `Ws2812Esp32RmtDriverBuilder`.
    pub fn new(
        pin: impl OutputPin + 'd,
    ) -> Result<Self, Ws2812Esp32RmtDriverError> {
        let resolution = Hertz(80_000_000);
        let config = TxChannelConfig { resolution, ..Default::default() };
        let tx = TxChannelDriver::new(pin, &config)?;
        Ok(Self { tx, resolution, encoder: None })
    }

    /// Creates a new `Ws2812Esp32RmtDriverBuilder` with `TxChannelDriver`.
    ///
    /// The resolution defaults to 80 MHz. If you configured a different resolution,
    /// call [`encoder_duration`](Self::encoder_duration) to set the correct pulse timings.
    pub fn new_with_rmt_driver(tx: TxChannelDriver<'d>) -> Result<Self, Ws2812Esp32RmtDriverError> {
        Ok(Self { tx, resolution: Hertz(80_000_000), encoder: None })
    }

    /// Sets the encoder duration times.
    ///
    /// # Arguments
    ///
    /// * `t0h` - T0H duration time (0 code, high voltage time)
    /// * `t0l` - T0L duration time (0 code, low voltage time)
    /// * `t1h` - T1H duration time (1 code, high voltage time)
    /// * `t1l` - T1L duration time (1 code, low voltage time)
    ///
    /// # Errors
    ///
    /// Returns an error if the encoder initialization failed.
    pub fn encoder_duration(
        mut self,
        t0h: &Duration,
        t0l: &Duration,
        t1h: &Duration,
        t1l: &Duration,
    ) -> Result<Self, Ws2812Esp32RmtDriverError> {
        self.encoder = Some(Ws2812Esp32RmtItemEncoder::new(
            self.resolution, t0h, t0l, t1h, t1l,
        )?);
        Ok(self)
    }

    /// Builds the `Ws2812Esp32RmtDriver`.
    pub fn build(self) -> Result<Ws2812Esp32RmtDriver<'d>, Ws2812Esp32RmtDriverError> {
        let encoder = if let Some(encoder) = self.encoder {
            encoder
        } else {
            Ws2812Esp32RmtItemEncoder::new(
                self.resolution,
                &WS2812_T0H_NS,
                &WS2812_T0L_NS,
                &WS2812_T1H_NS,
                &WS2812_T1L_NS,
            )?
        };

        Ok(Ws2812Esp32RmtDriver {
            tx: self.tx,
            encoder,
            #[cfg(not(target_vendor = "espressif"))]
            pixel_data: None,
            #[cfg(not(target_vendor = "espressif"))]
            phantom: Default::default(),
        })
    }
}

/// WS2812 ESP32 RMT driver wrapper.
///
/// # Examples
///
/// ```
/// # #[cfg(not(target_vendor = "espressif"))]
/// # use ws2812_esp32_rmt_driver::mock::esp_idf_hal;
/// #
/// use esp_idf_hal::peripherals::Peripherals;
/// use ws2812_esp32_rmt_driver::driver::Ws2812Esp32RmtDriver;
/// use ws2812_esp32_rmt_driver::driver::color::{LedPixelColor, LedPixelColorGrb24};
///
/// let peripherals = Peripherals::take().unwrap();
/// let led_pin = peripherals.pins.gpio27;
/// let mut driver = Ws2812Esp32RmtDriver::new(led_pin).unwrap();
///
/// // Single LED with RED color.
/// let red = LedPixelColorGrb24::new_with_rgb(30, 0, 0);
/// let pixel: [u8; 3] = red.as_ref().try_into().unwrap();
/// assert_eq!(pixel, [0, 30, 0]);
///
/// driver.write_blocking(pixel.clone().into_iter()).unwrap();
/// ```
pub struct Ws2812Esp32RmtDriver<'d> {
    /// TxRMT driver.
    tx: TxChannelDriver<'d>,
    /// `u8`-to-`rmt_item32_t` Encoder
    encoder: Ws2812Esp32RmtItemEncoder,

    /// Pixel binary array to be written
    ///
    /// If the target vendor does not equals to "espressif", pixel data is written into this
    /// instead of genuine encoder.
    #[cfg(not(target_vendor = "espressif"))]
    pub pixel_data: Option<Vec<u8>>,
    /// Dummy phantom to take care of lifetime for `pixel_data`.
    #[cfg(not(target_vendor = "espressif"))]
    phantom: PhantomData<&'d Option<Vec<u8>>>,
}

impl<'d> Ws2812Esp32RmtDriver<'d> {
    /// Creates a WS2812 ESP32 RMT driver wrapper.
    ///
    /// RMT driver shall be initialized and installed for `pin`.
    ///
    /// # Errors
    ///
    /// Returns an error if the RMT driver initialization failed.
    pub fn new(
        pin: impl OutputPin + 'd,
    ) -> Result<Self, Ws2812Esp32RmtDriverError> {
        Ws2812Esp32RmtDriverBuilder::new(pin)?.build()
    }

    /// Creates a WS2812 ESP32 RMT driver wrapper with `TxChannelDriver`.
    ///
    /// The resolution defaults to 80 MHz. If you configured a different resolution,
    /// use [`Ws2812Esp32RmtDriverBuilder`] with [`encoder_duration`](Ws2812Esp32RmtDriverBuilder::encoder_duration)
    /// to set the correct pulse timings.
    ///
    /// ```
    /// # #[cfg(not(target_vendor = "espressif"))]
    /// # use ws2812_esp32_rmt_driver::mock::esp_idf_hal;
    /// #
    /// # use esp_idf_hal::peripherals::Peripherals;
    /// # use esp_idf_hal::rmt::config::TxChannelConfig;
    /// # use esp_idf_hal::rmt::TxChannelDriver;
    /// # use esp_idf_hal::units::Hertz;
    /// #
    /// # let peripherals = Peripherals::take().unwrap();
    /// # let led_pin = peripherals.pins.gpio27;
    /// #
    /// let channel_config = TxChannelConfig { resolution: Hertz(80_000_000), ..Default::default() };
    /// let driver = TxChannelDriver::new(led_pin, &channel_config).unwrap();
    /// ```
    ///
    /// # Errors
    ///
    /// Returns an error if the RMT driver initialization failed.
    pub fn new_with_rmt_driver(tx: TxChannelDriver<'d>) -> Result<Self, Ws2812Esp32RmtDriverError> {
        Ws2812Esp32RmtDriverBuilder::new_with_rmt_driver(tx)?.build()
    }

    /// Writes pixel data from a pixel-byte sequence to the IO pin.
    ///
    /// Byte count per LED pixel and channel order is not handled by this method.
    /// The pixel data sequence has to be correctly laid out depending on the LED strip model.
    ///
    /// # Errors
    ///
    /// Returns an error if an RMT driver error occurred.
    pub fn write_blocking<'a, 'b, T>(
        &'a mut self,
        pixel_sequence: T,
    ) -> Result<(), Ws2812Esp32RmtDriverError>
    where
        'b: 'a,
        T: Iterator<Item = u8> + Send + 'b,
    {
        #[cfg(target_vendor = "espressif")]
        {
            let encoder = BytesEncoder::with_config(&BytesEncoderConfig {
                bit0: self.encoder.bit0,
                bit1: self.encoder.bit1,
                msb_first: true,
                ..Default::default()
            })?;
            let data: Vec<u8> = pixel_sequence.collect();
            self.tx.send_iter([encoder], core::iter::once(data.as_slice()), &TransmitConfig::default())?;
        }
        #[cfg(not(target_vendor = "espressif"))]
        {
            self.pixel_data = Some(pixel_sequence.collect());
        }
        Ok(())
    }

    /// Writes pixel data from a pixel-byte sequence to the IO pin.
    ///
    /// Byte count per LED pixel and channel order is not handled by this method.
    /// The pixel data sequence has to be correctly laid out depending on the LED strip model.
    ///
    /// # Errors
    ///
    /// Returns an error if an RMT driver error occurred.
    #[cfg(feature = "alloc")]
    pub fn write<'b, T>(
        &'static mut self,
        pixel_sequence: T,
    ) -> Result<(), Ws2812Esp32RmtDriverError>
    where
        T: Iterator<Item = u8> + Send + 'static,
    {
        #[cfg(target_vendor = "espressif")]
        {
            let encoder = BytesEncoder::with_config(&BytesEncoderConfig {
                bit0: self.encoder.bit0,
                bit1: self.encoder.bit1,
                msb_first: true,
                ..Default::default()
            })?;
            let data: Vec<u8> = pixel_sequence.collect();
            self.tx.send_iter([encoder], core::iter::once(data.as_slice()), &TransmitConfig::default())?;
        }
        #[cfg(not(target_vendor = "espressif"))]
        {
            self.pixel_data = Some(pixel_sequence.collect());
        }
        Ok(())
    }
}
