use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Copy this crate's own memory.x into OUT_DIR and put OUT_DIR on the
    // linker search path.  cortex-m-rt's link.x says `INCLUDE memory.x`, and
    // the linker runs in the workspace root, not in this crate's directory,
    // so a memory.x beside Cargo.toml is not found on its own.  A memory.x
    // in the workspace root would be found first, and with the app's layout
    // in it the bootloader would link at 0x08004000 instead of 0x08000000.
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
}
