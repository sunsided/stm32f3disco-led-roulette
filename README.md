# STM32F3 Discovery

## Setup

### Installation

From [here](https://docs.rust-embedded.org/discovery/f3discovery/03-setup/linux.html):

```shell
sudo apt-get install \
  gdb-multiarch \
  minicom \
  openocd
```

Install [cargo-binutils](https://github.com/rust-embedded/cargo-binutils):

```shell
cargo install cargo-binutils
```

## udev rules

```shell
lsusb | grep ST-LINK
```

gives

```
Bus 003 Device 026: ID 0483:3748 STMicroelectronics ST-LINK/V2
```

In other words, the `idVendor` is `0483` and `idProduct` is `3748`.

```shell
sudo vi /etc/udev/rules.d/99-openocd.rules
```

And add:

```udev
# STM32F3DISCOVERY - ST-LINK/V2.1
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE:="0666"
```

Reload udev rules:

```shell
sudo udevadm control --reload-rules
```

Unplug and re-plug the board if it was connected.

## Verify udev rules

```shell
lsusb | grep -i stm
```

should give something like

```
Bus 003 Device 027: ID 0483:3748 STMicroelectronics ST-LINK/V2
```

In this case, we need to test bus #3 for device #27:

```shell
crw-rw-rw-+ 1 root plugdev 189, 282 Jun  2 18:53 /dev/bus/usb/003/027
```

If the permissions are not `crw-rw-rw-` then the udev rules setup has failed.

## Verify OpenOCD installation

```shell
openocd -f interface/stlink.cfg -f target/stm32f3x.cfg
```

This should give something along the lines of

```
Open On-Chip Debugger 0.11.0
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
Info : auto-selecting first available session transport "hla_swd". To override use 'transport select <transport>'.
Info : The selected transport took over low-level target control. The results might differ compared to plain JTAG/SWD
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : clock speed 1000 kHz
Info : STLINK V2J23S0 (API v2) VID:PID 0483:3748
Info : Target voltage: 2.904739
Info : stm32f3x.cpu: hardware has 6 breakpoints, 4 watchpoints
Info : starting gdb server for stm32f3x.cpu on 3333
Info : Listening on port 3333 for gdb connections
```

In this case, it connects to `STLINK V2J23S0 (API v2) VID:PID 0483:3748`, which
represents the USB vendor/device combination detected above,
and will flash the COM LED on the board in red and green.

Also note the GDB port is `3333`; we will use this later on when we connect GDB.

## Building

Add the `thumbv7em-none-eabihf` target:

```shell
rustup target add thumbv7em-none-eabihf
```

See also [.cargo/config.toml](.cargo/config.toml) for building configuration.

Build the project:

```shell
cargo build

# Since .cargo/config.toml contains a target specification,
# the above line is equivalent to the following command:
#
# cargo build --target thumbv7em-none-eabihf
```

Check the processed file:

```shell
cargo readobj --target thumbv7em-none-eabihf --bin stm32f3disco-led-roulette -- --file-header
```

This produces something like:

```
ELF Header:
  Magic:   7f 45 4c 46 01 01 01 00 00 00 00 00 00 00 00 00
  Class:                             ELF32
  Data:                              2's complement, little endian
  Version:                           1 (current)
  OS/ABI:                            UNIX - System V
  ABI Version:                       0
  Type:                              EXEC (Executable file)
  Machine:                           ARM
  Version:                           0x1
  Entry point address:               0x8000195
  Start of program headers:          52 (bytes into file)
  Start of section headers:          1133316 (bytes into file)
  Flags:                             0x5000400
  Size of this header:               52 (bytes)
  Size of program headers:           32 (bytes)
  Number of program headers:         4
  Size of section headers:           40 (bytes)
  Number of section headers:         22
  Section header string table index: 20
```

## Flashing the binary

### Start OpenOCD

```shell
cd /tmp
openocd -f interface/stlink.cfg -f target/stm32f3x.cfg
```

### Start GDB

```shell
arm-none-eabi-gdb -q -ex "target extended-remote :3333" target/thumbv7em-none-eabihf/debug/stm32f3disco-led-roulette
```

This should print something like

```gdb
Registered pretty printers for UE4 classes
Reading symbols from target/thumbv7em-none-eabihf/debug/stm32f3disco-led-roulette...done.
Remote debugging using :3333
r0::init_data (sdata=0x8003503, edata=0xfd0300, sidata=0x0) at /home/markus/.cargo/registry/src/index.crates.io-6f17d22bba15001f/r0-0.2.2/src/lib.rs:132
132         while sdata < edata {
(gdb)
```

Meanwhile, the OpenOCD terminal output adds something like the following:

```
Info : accepting 'gdb' connection on tcp/3333
Info : device id = 0x10036422
Info : flash size = 256kbytes
```

### Flash the program

Use the `load` command in `gdb` to flash the program:

```gdb
(gdb) load
Loading section .vector_table, size 0x194 lma 0x8000000
Loading section .text, size 0x1a5c lma 0x8000194
Loading section .rodata, size 0x668 lma 0x8001bf0
Start address 0x8000194, load size 8792
Transfer rate: 17 KB/sec, 2930 bytes/write.
```

OpenOCD now shows something like

```
Info : Unable to match requested speed 1000 kHz, using 950 kHz
Info : Unable to match requested speed 1000 kHz, using 950 kHz
target halted due to debug-request, current mode: Thread 
xPSR: 0x01000000 pc: 0x080002b8 msp: 0x2000a000
Info : Unable to match requested speed 8000 kHz, using 4000 kHz
Info : Unable to match requested speed 8000 kHz, using 4000 kHz
Info : Unable to match requested speed 1000 kHz, using 950 kHz
Info : Unable to match requested speed 1000 kHz, using 950 kHz
target halted due to debug-request, current mode: Thread 
xPSR: 0x01000000 pc: 0x08000194 msp: 0x2000a000
```

### Debug the application

In GDB, use `break main` to break at the `main()` function:

```
(gdb) break main
Breakpoint 1 at 0x80001e8: file src/main.rs, line 7.
```

Next, issue `continue`:

```gdb
(gdb) continue
Continuing.
Note: automatically using hardware breakpoints for read-only addresses.

Breakpoint 1, main () at src/main.rs:7
7       #[entry]
```

The program is stopped at `#[entry]` which sets up the jump to `main`.
We can show the disassembled code by issuing `disassembly /m`.

```gdb
(gdb) disassemble /m
Dump of assembler code for function main:
7       #[entry]
   0x080001e4 <+0>:     push    {r7, lr}
   0x080001e6 <+2>:     mov     r7, sp
=> 0x080001e8 <+4>:     bl      0x80001ee <stm32f3disco_led_roulette::__cortex_m_rt_main>
   0x080001ec <+8>:     udf     #254    ; 0xfe

End of assembler dump.
```

Step into the next instruction using `step`:

```
(gdb) step
halted: PC: 0x080001ee
stm32f3disco_led_roulette::__cortex_m_rt_main () at src/main.rs:10
10          let x = 42;
```

Run `step` again, then print the variables:

```gdb
(gdb) print x
$1 = 42
(gdb) print _y
$2 = 42
(gdb) print &x
$3 = (i32 *) 0x20009fe4
(gdb) print &_y
$4 = (i32 *) 0x20009fe4
```

To show all local variables, run `show locals`:

```gdb
(gdb) info locals
x = 42
_y = 42
```

To reset the program, run `monitor reset halt` and `continue`:

```gdb
(gdb) monitor reset halt
Unable to match requested speed 1000 kHz, using 950 kHz
Unable to match requested speed 1000 kHz, using 950 kHz
target halted due to debug-request, current mode: Thread 
xPSR: 0x01000000 pc: 0x08000194 msp: 0x2000a000
(gdb) continue
Continuing.

Breakpoint 1, main () at src/main.rs:7
7       #[entry]
```
