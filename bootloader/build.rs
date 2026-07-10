use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(out.join("memory.x"), include_bytes!("memory.x")).unwrap();

    // redirect + fixup: the linker computes the reset vector from .text address
    fs::write(out.join("redirect.x"), concat!(
        "SECTIONS {\n",
        "  .redirect 0x08000000 : {\n",
        "    LONG(_stack_start)\n",
        "    LONG(ABSOLUTE(ADDR(.text)) | 1)\n",
        "  } > FLASH\n",
        "}\n",
        "INSERT BEFORE .vector_table;\n"
    ).as_bytes()).unwrap();

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg-bins=-Tredirect.x");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_state_start=0x08006000");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_state_end=0x08007000");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_active_start=0x08009000");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_active_end=0x08013000");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_dfu_start=0x08013000");
    println!("cargo:rustc-link-arg-bins=--defsym=__bootloader_dfu_end=0x0801F800");
    println!("cargo:rerun-if-changed=memory.x");
}
