//! Display rotation, mirroring, and controller memory mapping.

/// Display content rotation relative to the panel's portrait orientation.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
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
    /// Returns the rotation in degrees.
    pub const fn degree(self) -> i32 {
        match self {
            Self::Deg0 => 0,
            Self::Deg90 => 90,
            Self::Deg180 => 180,
            Self::Deg270 => 270,
        }
    }

    /// Converts an angle into a rotation.
    ///
    /// Returns an error if the angle isn't an integer multiple of 90°.
    pub const fn try_from_degree(mut angle: i32) -> Result<Self, InvalidAngleError> {
        if angle < 0 || angle > 270 {
            angle = angle.rem_euclid(360);
        }

        Ok(match angle {
            0 => Self::Deg0,
            90 => Self::Deg90,
            180 => Self::Deg180,
            270 => Self::Deg270,
            _ => return Err(InvalidAngleError),
        })
    }

    /// Rotates one rotation by another rotation.
    #[must_use]
    pub const fn rotate(self, other: Rotation) -> Self {
        match Self::try_from_degree(self.degree() + other.degree()) {
            Ok(rotation) => rotation,
            Err(_) => unreachable!(),
        }
    }

    /// Returns `true` for 0° and 180° rotations.
    pub const fn is_horizontal(self) -> bool {
        matches!(self, Self::Deg0 | Self::Deg180)
    }

    /// Returns `true` for 90° and 270° rotations.
    pub const fn is_vertical(self) -> bool {
        matches!(self, Self::Deg90 | Self::Deg270)
    }
}

/// Error returned when an angle isn't an integer multiple of 90°.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InvalidAngleError;

/// Display content orientation relative to the panel's portrait orientation.
///
/// Transformations use the same ordering as the synchronous driver:
///
/// ```
/// use mipidsi_async::{Orientation, Rotation};
///
/// let orientation = Orientation::new().rotate(Rotation::Deg270).flip_vertical();
/// assert_eq!(orientation, Orientation::new().flip_vertical().rotate(Rotation::Deg90));
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Orientation {
    /// Rotation.
    pub rotation: Rotation,
    /// Whether the native display columns are reversed after rotation mapping.
    pub mirrored: bool,
}

impl Orientation {
    /// Creates an unrotated, unmirrored orientation.
    pub const fn new() -> Self {
        Self {
            rotation: Rotation::Deg0,
            mirrored: false,
        }
    }

    /// Rotates the orientation.
    #[must_use]
    pub const fn rotate(self, rotation: Rotation) -> Self {
        Self {
            rotation: self.rotation.rotate(rotation),
            mirrored: self.mirrored,
        }
    }

    /// Flips the orientation across the horizontal axis.
    #[must_use]
    pub const fn flip_horizontal(self) -> Self {
        Self {
            rotation: if self.rotation.is_vertical() {
                self.rotation.rotate(Rotation::Deg180)
            } else {
                self.rotation
            },
            mirrored: !self.mirrored,
        }
    }

    /// Flips the orientation across the vertical axis.
    #[must_use]
    pub const fn flip_vertical(self) -> Self {
        Self {
            rotation: if self.rotation.is_vertical() {
                self.rotation
            } else {
                self.rotation.rotate(Rotation::Deg180)
            },
            mirrored: !self.mirrored,
        }
    }
}

impl Default for Orientation {
    fn default() -> Self {
        Self::new()
    }
}

/// Mapping from display coordinates to the controller's native rows and columns.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MemoryMapping {
    /// Rows and columns are swapped.
    pub swap_rows_and_columns: bool,
    /// Rows are reversed.
    pub reverse_rows: bool,
    /// Columns are reversed.
    pub reverse_columns: bool,
}

impl MemoryMapping {
    /// Creates a controller memory mapping from an orientation.
    pub const fn from_orientation(orientation: Orientation) -> Self {
        let (reverse_rows, reverse_columns) = match orientation.rotation {
            Rotation::Deg0 => (false, false),
            Rotation::Deg90 => (false, true),
            Rotation::Deg180 => (true, true),
            Rotation::Deg270 => (true, false),
        };

        Self {
            reverse_rows,
            reverse_columns: reverse_columns ^ orientation.mirrored,
            swap_rows_and_columns: orientation.rotation.is_vertical(),
        }
    }

    pub(crate) const fn madctl(self) -> u8 {
        // MADCTL: MY (bit 7), MX (bit 6), MV (bit 5), RGB order (bit 3 clear).
        ((self.reverse_rows as u8) << 7)
            | ((self.reverse_columns as u8) << 6)
            | ((self.swap_rows_and_columns as u8) << 5)
    }
}

impl From<Orientation> for MemoryMapping {
    fn from(orientation: Orientation) -> Self {
        Self::from_orientation(orientation)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn controller_address_modes() {
        for (rotation, normal, mirrored_mode) in [
            (Rotation::Deg0, 0x00, 0x40),
            (Rotation::Deg90, 0x60, 0x20),
            (Rotation::Deg180, 0xc0, 0x80),
            (Rotation::Deg270, 0xa0, 0xe0),
        ] {
            for (mirrored, expected) in [(false, normal), (true, mirrored_mode)] {
                let orientation = Orientation { rotation, mirrored };
                assert_eq!(MemoryMapping::from(orientation).madctl(), expected);
            }
        }
    }

    #[test]
    fn flips_match_sync_at_every_rotation() {
        use Rotation::*;

        for (rotation, horizontal, vertical) in [
            (Deg0, Deg0, Deg180),
            (Deg90, Deg270, Deg90),
            (Deg180, Deg180, Deg0),
            (Deg270, Deg90, Deg270),
        ] {
            for mirrored in [false, true] {
                let orientation = Orientation { rotation, mirrored };
                assert_eq!(
                    orientation.flip_horizontal(),
                    Orientation {
                        rotation: horizontal,
                        mirrored: !mirrored,
                    }
                );
                assert_eq!(
                    orientation.flip_vertical(),
                    Orientation {
                        rotation: vertical,
                        mirrored: !mirrored,
                    }
                );
                assert_eq!(orientation.flip_horizontal().flip_horizontal(), orientation);
                assert_eq!(orientation.flip_vertical().flip_vertical(), orientation);
            }
        }
    }

    #[test]
    fn rotation_and_flipping_preserve_sync_operation_order() {
        let orientation = Orientation::new().rotate(Rotation::Deg270).flip_vertical();
        assert_ne!(
            orientation,
            Orientation::new().flip_vertical().rotate(Rotation::Deg270)
        );
        assert_eq!(
            orientation,
            Orientation::new().flip_vertical().rotate(Rotation::Deg90)
        );
    }
}
