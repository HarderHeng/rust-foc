use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(out.join("memory.x"), include_bytes!("memory.x")).unwrap();
    // Redirect stub: initial SP + reset vector at 0x08000000
    fs::write(out.join("redirect.x"),
        b"SECTIONS { .redirect 0x08000000 : { LONG(0x20008000) LONG(0x080071D9) } > FLASH }\nINSERT BEFORE .vector_table;\n"
    ).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg-bins=-Tredirect.x");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_state_start=0x08006000");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_state_end=0x08007000");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_active_start=0x08007000");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_active_end=0x08013000");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_dfu_start=0x08013000");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_dfu_end=0x0801F800");
    println!("cargo:rerun-if-changed=memory.x");
}
