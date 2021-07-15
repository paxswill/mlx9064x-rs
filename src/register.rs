use num_enum::{IntoPrimitive, TryFromPrimitive};

/// Identify which subpage to access.
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord)]
#[derive(IntoPrimitive, TryFromPrimitive)]
#[repr(usize)]
pub enum Subpage {
    Zero = 0,
    One = 1,
}

/// The pixel access pattern used by a camera.
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord)]
#[derive(IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub enum AccessPattern {
    /// Pixels alternate between subpages, resulting in a chess or checker board pattern.
    ///
    /// This is the default (and strongly recommended value) for the MLX909640. While the MLX909641
    /// datasheet mentions this mode in the register section, no further mention is made in the
    /// datasheet, so I'm not sure it's actually supported.
    Chess,

    /// Each row of pixels is in the same subpage, with the rows alternating between subpages.
    ///
    /// This is the default mode for the MLX90641.
    Interleave,
}
