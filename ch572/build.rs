use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    // The linker script only applies to the firmware image.  Host test
    // builds of the library link normally.
    if env::var("CARGO_CFG_TARGET_ARCH").as_deref() != Ok("riscv32") {
        return;
    }
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(out.join("link.x"), include_bytes!("link.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=link.x");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
}
