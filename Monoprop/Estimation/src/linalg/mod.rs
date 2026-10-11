//! Matrix helpers on nalgebra fixed-size types.
//!
//! - `basics`: diag and outer products
//! - `skew`: the cross-product matrix skew(v)
//! - `blocks`: reading and writing 3x3 blocks of larger matrices
//! - `spd`: symmetrising a matrix and checking positive definiteness
//! - `cholesky`: Cholesky factorisation L L^T
//! - `solve`: solving S x = b with Cholesky (no explicit inverse)

mod basics;
pub use basics::*;
mod skew;
pub use skew::*;
mod blocks;
pub use blocks::*;
mod spd;
pub use spd::*;
mod cholesky;
pub use cholesky::*;
mod solve;
pub use solve::*;
