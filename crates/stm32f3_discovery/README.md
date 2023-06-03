# STM32F3 Discovery

This crate is a clone of the [stm32f3-discovery](https://github.com/rubberduck203/stm32f3-discovery) crate
with local adjustments.

## Re-exports

This crate re-exports the [`cortex_m`], [`cortex_m_rt`], [`panic_itm`],
[`stm32f3xx_hal`] and [`switch_hal`] crates.

## Memory Layout

The STM32F303VC6's memory layout is specified in [memory.x](memory.x). The accompanying [build.rs](build.rs)
build script presents it to the linker. See the [`cortex_m_rt`] crate
documentation for more information.

[`cortex_m`]: https://docs.rs/cortex-m
[`cortex_m_rt`]: https://docs.rs/cortex-m-rt
[`panic_itm`]: https://crates.io/crates/panic_itm
[`stm32f3xx_hal`]: https://docs.rs/stm32f3xx_hal
[`switch_hal`]: https://docs.rs/switch_hal
