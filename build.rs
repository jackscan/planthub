use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Put `memory.x` into output directory.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    // Add memory.x to linker search path.
    println!("cargo:rustc-link-search={}", out.display());
    // Rerun build script when memory.x is changed.
    println!("cargo:rerun-if-changed=memory.x");
}
