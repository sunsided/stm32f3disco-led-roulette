# STM32F3 Discovery

The LED Roulette example on the [STM32F3 Discovery](https://www.st.com/en/evaluation-tools/stm32f3discovery.html) board,
written in Rust. An experiment to get started with embedded development in Rust, as well as testing the IDE based development
and debugging experience in JetBrains CLion while using Rust.

<div align="center">
  <img src="docs/led-roulette.webp" alt="Moving LEDs on the STM32F3 Discovery board"/>
</div>

## Requirements

This project uses the `thumbv7em-none-eabihf` target. Prepare it using:

```shell
rustup target add thumbv7em-none-eabihf
```

Set up [probe-rs](https://probe.rs/), and follow the setup instructions (e.g. the [Linux udev rules](https://probe.rs/docs/getting-started/probe-setup/#linux%3A-udev-rules)).

### Troubleshooting

My STM32F3 Discovery had an outdated version of the ST-Link firmware, and probe-rs refused to work with it: 

> probe-rs failed to open the debug probe
> Error processing command: The firmware on the probe is outdated.

Go to [ST-LINK, ST-LINK/V2, ST-LINK/V2-1, STLINK-V3 boards firmware upgrade](https://www.st.com/en/development-tools/stsw-link007.html)
and download the latest version. Unzip, run the application and follow the instructions.

```shell
java -jar STLinkUpgrade.jar
```

## Quickstart

Start OpenOCD, e.g. by running [openocd.sh](openocd.sh):

```shell
./openocd.sh
```

In another shell, flash and run the program:

```shell
cargo run --bin stm32f3disco-led-roulette
```

In the GDB prompt, run `continue`.

```gdb
(gdb) continue
```

## Preparing the development environment

- For information on how to set up **Rust, OpenOCD and GDB** for the `thumbv7em-none-eabihf` target,
see [docs/SETUP.md](docs/SETUP.md).
- For a **JetBrains CLion** specific setup, see [docs/CLION.md](docs/CLION.md).
