use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    // Full layout with all partitions
    fs::write(out.join("app-layout.x"), fs::read("app-layout.x").unwrap()).unwrap();
    // Minimal memory.x for link.x INCLUDE compatibility
    fs::write(out.join("memory.x"), b"MEMORY { FLASH : ORIGIN = 0x08009000, LENGTH = 44K   RAM : ORIGIN = 0x20000000, LENGTH = 32K }\n").unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg-bins=-Tapp-layout.x");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo:rerun-if-changed=app-layout.x");
    println!("cargo:rustc-env=FOC_VERSION={}", env!("CARGO_PKG_VERSION"));
    let sha = std::process::Command::new("git").args(["rev-parse","--short","HEAD"]).output().ok()
        .and_then(|o| if o.status.success() { Some(String::from_utf8_lossy(&o.stdout).trim().to_string()) } else { None })
        .unwrap_or_else(|| "unknown".to_string());
    println!("cargo:rustc-env=FOC_GIT_SHA={}", sha);
    let ts = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    println!("cargo:rustc-env=FOC_BUILD_TIMESTAMP={}", ts);
    println!("cargo:rerun-if-changed=.git/HEAD");
}
