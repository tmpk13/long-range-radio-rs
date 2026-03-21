use std::path::PathBuf;

fn main() {
    let rh_dir: PathBuf = "RadioHead".into();
    let csrc_dir: PathBuf = "csrc".into();

    cc::Build::new()
        .cpp(true)
        .std("c++11")
        // Our platform header must shadow RadioHead/RadioHead.h
        .include(&csrc_dir)
        .include(&rh_dir)
        // RadioHead routing stack sources
        .file(rh_dir.join("RHGenericDriver.cpp"))
        .file(rh_dir.join("RHDatagram.cpp"))
        .file(rh_dir.join("RHReliableDatagram.cpp"))
        .file(rh_dir.join("RHRouter.cpp"))
        .file(rh_dir.join("RHMesh.cpp"))
        // Our shim
        .file(csrc_dir.join("rh_shim.cpp"))
        // Bare-metal flags
        .flag("-fno-exceptions")
        .flag("-fno-rtti")
        .flag("-fno-threadsafe-statics")
        .flag("-fno-use-cxa-atexit")
        // Suppress warnings from RadioHead's C++ style
        .flag("-Wno-unused-variable")
        .flag("-Wno-unused-parameter")
        .compile("radiohead");

    // The cc crate emits -lstdc++ for C++ builds, but the bare-metal linker
    // doesn't know where to find it. Derive the search path from the CXX compiler.
    let target = std::env::var("TARGET").unwrap_or_default();
    let cxx_key = format!("CXX_{}", target.replace('-', "_"));
    if let Ok(cxx) = std::env::var(&cxx_key).or_else(|_| std::env::var("CXX")) {
        let cxx_path = PathBuf::from(&cxx);
        // Walk up from .../bin/riscv32-esp-elf-g++ to the toolchain root,
        // then find the matching libstdc++ directory.
        if let Some(bin_dir) = cxx_path.parent() {
            let toolchain_root = bin_dir.parent().unwrap_or(bin_dir);
            // Try target-specific multilib first, then generic
            let candidates = [
                toolchain_root.join("picolibc/riscv32-esp-elf/lib/rv32imc_zicsr_zifencei/ilp32/no-rtti"),
                toolchain_root.join("picolibc/riscv32-esp-elf/lib/no-rtti"),
                toolchain_root.join("picolibc/riscv32-esp-elf/lib"),
            ];
            for dir in &candidates {
                if dir.join("libstdc++.a").exists() {
                    println!("cargo:rustc-link-search=native={}", dir.display());
                    break;
                }
            }
        }
    }

    println!("cargo:rerun-if-changed=csrc/");
    println!("cargo:rerun-if-changed=RadioHead/RHGenericDriver.cpp");
    println!("cargo:rerun-if-changed=RadioHead/RHDatagram.cpp");
    println!("cargo:rerun-if-changed=RadioHead/RHReliableDatagram.cpp");
    println!("cargo:rerun-if-changed=RadioHead/RHRouter.cpp");
    println!("cargo:rerun-if-changed=RadioHead/RHMesh.cpp");
}
