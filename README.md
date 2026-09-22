# Embedded Linux BSP

A Linux system for the Raspberry Pi 4, built from source with Buildroot, running
an out-of-tree character device driver for the ST VL53L1X time-of-flight ranging
sensor. Distance measurements are delivered by a hardware interrupt from the
sensor.

## Overview

| Component | Description |
| --- | --- |
| Buildroot image | Linux 6.12.61 built from source for aarch64, no stock distribution |
| Device tree overlay | Declares the sensor on I2C1 at 0x29, binds GPIO17 as its interrupt |
| Kernel driver | I2C client driver, character device, threaded IRQ handler |
| Buildroot package | Cross-compiles the module, compiles the overlay, installs an init script |

## Kernel Driver

The VL53L1X uses 16-bit big-endian register addresses, with its own
`i2c_transfer()` calls and a kmalloc'd scratch buffer safe for DMA-capable
controllers.

`probe()` verifies the model ID (0xEACC), waits for sensor firmware boot, writes
a 91-byte configuration block to registers 0x2D-0x87, runs a throwaway
calibration measurement, then sets distance mode and timing budget before
starting continuous ranging. The register sequence follows ST's Ultra Lite
Driver (UM2356).

Interrupt handling is threaded, since servicing it requires sleeping I2C
transactions. Two design decisions follow from the sensor's `GPIO1` being a
latched output that stays asserted until the driver writes
`SYSTEM__INTERRUPT_CLEAR`:

Level-triggered, not edge-triggered. With edge triggering a single missed or
mishandled edge leaves the line stuck asserted, no further edges are generated,
and interrupts stop permanently. Level triggering re-runs the handler while the
line remains asserted, so the system always recovers. `IRQF_ONESHOT` keeps the
interrupt masked until the handler returns.

The handler fully consumes the interrupt. It reads the measurement and clears
the latch itself, caching the result for whichever reader wakes next. An earlier
version cleared only inside `read()`, which meant an idle system left the line
asserted and the handler spinning at bus speed.

`read()` sleeps on a wait queue and discards any cached sample first, so every
call returns a measurement taken after the call began. If the device tree
supplies no interrupt, the driver falls back to polling, so the same module works
with or without the overlay. `/sys/class/vl53l1x/vl53l1x0/irq_count` exposes a
running interrupt count.

## Engineering Notes

The overlay declares `IRQ_TYPE_LEVEL_HIGH`, arrived at empirically. Initial
assumption was active-low with a pull-up, the common convention for an
open-drain interrupt output. Interrupts either never fired or fired continuously
at bus speed, and the sensor's own status register never changed value under any
condition. Configuring the line with a pull-down instead and observing the pin
while driving the sensor manually over I2C settled it: the line reads high while
a measurement is pending and drops the instant the interrupt is cleared. A
pull-down cannot be overcome by nothing, so that single reading proved the output
is active-high and push-pull, that clearing works as documented, and that the
wiring was sound.

The reason the above took as long as it did: the pull-up declared in the
overlay's pinctrl node was not being applied, leaving GPIO17 with no defined pull
state. A level-triggered interrupt on a floating pin produces a continuous storm
that looks exactly like sensor misbehaviour, and a floating input returns
arbitrary readings rather than wrong ones. The step that resolved it was checking
what the pin read with nothing connected at all, which with a pull-down
configured must read 0.

## Known Limitations

GPIO17 is hardcoded in the overlay rather than exposed as a device tree property.

Single sensor only: the VL53L1X defaults to address 0x29, so multiple units on
one bus collide. Supporting more would require driving `XSHUT` from a GPIO to
bring them up individually and reassign addresses.

## Hardware

- Raspberry Pi 4
- VL53L1X time-of-flight ranging sensor
- USB-to-UART adapter for the serial console

## Stack

Buildroot, Linux, device tree
