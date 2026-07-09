use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let slot = env::var("SLOT").unwrap_or_else(|_| "a".to_string());
    let memory_file = match slot.as_str() {
        "a" => "memory-slot-a.x",
        "b" => "memory-slot-b.x",
        other => panic!("SLOT must be 'a' or 'b', got '{other}'"),
    };
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(out.join("memory.x"), fs::read(memory_file).unwrap()).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo:rerun-if-changed=memory-slot-a.x");
    println!("cargo:rerun-if-changed=memory-slot-b.x");
    println!("cargo:rerun-if-env-changed=SLOT");

    println!("cargo:rustc-env=FOC_VERSION={}", env!("CARGO_PKG_VERSION"));
    let sha = std::process::Command::new("git").args(["rev-parse","--short","HEAD"]).output().ok()
        .and_then(|o| if o.status.success() { Some(String::from_utf8_lossy(&o.stdout).trim().to_string()) } else { None })
        .unwrap_or_else(|| "unknown".to_string());
    println!("cargo:rustc-env=FOC_GIT_SHA={}", sha);
    let ts = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    println!("cargo:rustc-env=FOC_BUILD_TIMESTAMP={}", ts);
    println!("cargo:rerun-if-changed=.git/HEAD");
}
