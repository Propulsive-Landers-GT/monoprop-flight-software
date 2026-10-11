//! Quaternions and rotation matrices.
//!
//! - `product`: Hamilton product and the rotation matrix R(q)
//! - `quat_exp_log`: Exp and Log maps for quaternions
//! - `so3_exp_log`: Exp and Log maps for rotation matrices

mod product;
pub use product::*;
mod quat_exp_log;
pub use quat_exp_log::*;
mod so3_exp_log;
pub use so3_exp_log::*;
