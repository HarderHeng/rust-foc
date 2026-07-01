// Bootloader-specific linker wiring.
//
// cortex-m-rt's `link.x` does `INCLUDE memory.x` (and `INCLUDE device.x`)
// near its start, looking for those files relative to its own OUT_DIR.
// We copy our `memory.x` — carves the OTA flag page out of code and
// asserts code never spills into it — into that directory so it
// wins the INCLUDE lookup.  We must NOT also emit our `memory.x` as
// a separate `-T` arg, because that would define MEMORY twice and
// fail to link.
//
// `embassy-stm32` `memory-x` feature is intentionally ON (Cargo.toml)
// because it provides the default `memory.x` containing the chip's
// FLASH/RAM regions — without it, cortex-m-rt's link.x would error
// with "memory region not defined: RAM".

use std::path::{Path, PathBuf};

fn main() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let src_memory = manifest_dir.join("memory.x");

    println!("cargo:rerun-if-changed=memory.x");

    // Locate cortex-m-rt's OUT_DIR by scanning the cargo target tree.
    let out_dir = std::env::var("OUT_DIR").unwrap();
    let profile_root = Path::new(&out_dir)
        .ancestors()
        .nth(3) // .../target/<triple>/<profile>
        .expect("derive profile root from OUT_DIR");
    let build_dir = profile_root.join("build");
    let cmrt_outdir = std::fs::read_dir(&build_dir)
        .expect("read target build dir")
        .flatten()
        .find(|e| {
            e.file_name()
                .to_string_lossy()
                .starts_with("cortex-m-rt-")
        })
        .map(|e| e.path().join("out"))
        .expect("could not locate cortex-m-rt OUT_DIR");
    std::fs::create_dir_all(&cmrt_outdir).expect("mkdir cortex-m-rt out");

    let dst = cmrt_outdir.join("memory.x");
    std::fs::copy(&src_memory, &dst)
        .unwrap_or_else(|e| panic!("copy {} -> {}: {e}", src_memory.display(), dst.display()));

    // Only emit the link.x flag — the INCLUDE inside link.x finds
    // our memory.x (because we copied it next to link.x) and that
    // single load defines both MEMORY and the OTA-flag ASSERT.
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
}
