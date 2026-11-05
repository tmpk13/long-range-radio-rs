use std::env;
use std::fs;
use std::path::PathBuf;

use flash_layout::{ACTIVE_BASE, MAX_FW_SIZE};

fn main() {
    // cortex-m-rt's link.x says `INCLUDE memory.x`.  Generate this crate's
    // memory.x (the active app slot, where the bootloader jumps) from the
    // flash-layout crate into OUT_DIR, and put OUT_DIR on the linker search
    // path: the linker runs in the workspace root, not in this crate's
    // directory, so it finds memory.x only through the search path.  For the
    // same reason no memory.x may sit in the workspace root, where the linker
    // would find it first for every member.
    let mut memory_x = String::new();
    flash_layout::write_memory_x(&mut memory_x, ACTIVE_BASE, MAX_FW_SIZE).unwrap();
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(out.join("memory.x"), memory_x).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=build.rs");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
}
