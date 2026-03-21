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

    println!("cargo:rerun-if-changed=csrc/");
    println!("cargo:rerun-if-changed=RadioHead/RHGenericDriver.cpp");
    println!("cargo:rerun-if-changed=RadioHead/RHDatagram.cpp");
    println!("cargo:rerun-if-changed=RadioHead/RHReliableDatagram.cpp");
    println!("cargo:rerun-if-changed=RadioHead/RHRouter.cpp");
    println!("cargo:rerun-if-changed=RadioHead/RHMesh.cpp");
}
