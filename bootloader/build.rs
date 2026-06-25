fn main() {
    // Use the chip's own memory.x (via embassy-stm32's memory-x feature on the OUT_DIR),
    // plus cortex-m-rt's link.x for the standard linker flow.
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
