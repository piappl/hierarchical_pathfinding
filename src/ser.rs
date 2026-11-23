//! Versioned byte format for [`PathCache`](crate::PathCache) graphs.
//!
//! Enabled by the cargo feature `serialize`. A blob produced by
//! [`PathCache::to_bytes`](crate::PathCache::to_bytes) is
//! `b"HPAG"` (4 bytes) + a `u8` format version (currently `1`) + a
//! `bincode` payload, so it can be stored or transported by a component that
//! never deserializes it, and read back later by a planner that does.
//!
//! Note that the neighborhood type is part of the `PathCache` type:
//! `from_bytes` only succeeds if the receiver type matches the one that was
//! written. Also note that a cache created with
//! [`PathCacheConfig::cache_paths`](crate::PathCacheConfig::cache_paths) set to
//! `false` serializes its segments as `Unknown`, so resolving those segments on
//! the reading side requires the real cost function again.

use std::fmt;

/// Magic bytes at the start of every blob written by
/// [`PathCache::to_bytes`](crate::PathCache::to_bytes).
pub(crate) const HPA_MAGIC: &[u8; 4] = b"HPAG";

/// Format version written by [`PathCache::to_bytes`](crate::PathCache::to_bytes).
pub(crate) const HPA_VERSION: u8 = 1;

/// Total length of the header written by [`PathCache::to_bytes`](crate::PathCache::to_bytes).
pub(crate) const HPA_HEADER_LEN: usize = 5;

/// Errors that can occur when reading a `PathCache` from bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HpaSerError {
    /// Input shorter than the 5-byte header, so it cannot be a valid blob.
    TooShort,
    /// The first 4 bytes are not `HPAG` — the input is not a `PathCache` blob.
    BadMagic,
    /// The header names a format version this build cannot read; carries the version found.
    UnsupportedVersion(u8),
    /// Header is fine but the payload could not be deserialized (truncated or corrupt).
    Corrupt,
}

impl fmt::Display for HpaSerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooShort => write!(f, "input too short to contain a PathCache header"),
            Self::BadMagic => write!(f, "input does not start with the HPAG magic"),
            Self::UnsupportedVersion(v) => {
                write!(f, "unsupported PathCache format version {v}")
            }
            Self::Corrupt => write!(f, "PathCache payload could not be deserialized"),
        }
    }
}

impl std::error::Error for HpaSerError {}
