use std::{env, path::PathBuf};

fn main() {
    // Use cortex-m-rt's linker script
    println!("cargo:rustc-link-arg=-Tlink.x");

    println!("cargo:rustc-link-arg=-Tdefmt.x");
    // ------------------------------------------

    // Make the *containing directory* of memory.x visible to the linker
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    println!("cargo:rustc-link-search={}", manifest_dir.display());

    // Rebuild if memory.x changes
    println!(
        "cargo:rerun-if-changed={}",
        manifest_dir.join("memory.x").display()
    );
}
