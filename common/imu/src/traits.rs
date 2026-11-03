use crate::{ImuInfo, ImuSample};

/// A source of IMU samples: a hardware driver, a simulator or a placeholder.
pub trait Imu {
    type Error;

    /// What this device measures, its rates and its noise figures.
    fn info(&self) -> &ImuInfo;

    /// Non-blocking poll for the newest sample.
    ///
    /// `now_s` is the caller's clock; the driver stamps it into
    /// [`ImuSample::timestamp_s`] so every IMU shares the control loop's timebase.
    /// Returns `Ok(None)` when no new sample has arrived since the last call.
    fn read(&mut self, now_s: f64) -> Result<Option<ImuSample>, Self::Error>;
}
