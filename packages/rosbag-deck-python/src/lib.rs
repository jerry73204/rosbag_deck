// TODO: Implement PyO3 bindings for rosbag-deck
//
// Expose a thin Python wrapper around the Rust core library.

use pyo3::prelude::*;

#[pymodule]
fn rosbag_deck(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    // TODO: Add Python-visible classes and functions
    Ok(())
}
