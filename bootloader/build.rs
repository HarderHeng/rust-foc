use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(out.join("memory.x"), include_bytes!("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_state_start=0x08006000");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_state_end=0x08007000");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_active_start=0x08009000");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_active_end=0x08014000");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_dfu_start=0x08014000");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_dfu_end=0x08020000");
    println!("cargo:rerun-if-changed=memory.x");
}
