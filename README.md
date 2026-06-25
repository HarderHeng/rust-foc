# foc-rust

Field Oriented Control firmware in Rust on the ST B-G431B-ESC1 dev board (STM32G431CBU6).

## Status

Initial scaffold. USART2 + defmt + heartbeat. See `docs/superpowers/specs/2026-06-25-b-g431b-esc1-initialization-design.md` for the design.

## Toolchain

- `rustup target add thumbv7em-none-eabihf`
- `cargo install probe-rs --features=cli`
- `cargo install flip-link` (optional, for stack overflow detection — not used yet)

## Build

```bash
cargo build              # debug, target/thumbv7em-none-eabihf/debug/foc-rust.elf
cargo build --release    # release, size-optimized
```

## Flash + run

```bash
cargo run                # probe-rs runs + shows RTT
```

Open a second terminal to see USART2 output:

```bash
screen /dev/ttyUSB0 115200    # or whichever TTY your USB-TTL is on
```

## Architecture

```
src/
├── main.rs       # composition root
├── bsp.rs        # board constants + board_init()
├── drivers/
│   └── debug_uart.rs   # DebugShellSink trait + Uart2Sink
└── tasks/
    └── heartbeat.rs    # 500ms tick task
```

See `docs/superpowers/specs/` for the full design.

## License

TBD
