fn main() {
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    // Expose the crate version at compile time.
    println!("cargo:rustc-env=FOC_VERSION={}", env!("CARGO_PKG_VERSION"));

    // Git short SHA (e.g. "a1b2c3d").
    let sha = std::process::Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()
        .and_then(|o| {
            if o.status.success() {
                Some(String::from_utf8_lossy(&o.stdout).trim().to_string())
            } else {
                None
            }
        })
        .unwrap_or_else(|| "unknown".to_string());
    println!("cargo:rustc-env=FOC_GIT_SHA={}", sha);

    // Build timestamp (Unix seconds).
    let ts = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    println!("cargo:rustc-env=FOC_BUILD_TIMESTAMP={}", ts);

    println!("cargo:rerun-if-changed=build.rs");
}
