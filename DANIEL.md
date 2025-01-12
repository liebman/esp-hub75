
examples:
- `i2s_parallel` for esp32 uses 16bit parallel (fails)
- `i2s_parallel_latch` for esp32 uses 8bit parallel (fails)
- `lcd_cam` for esp32s3 (fails)
- `parl_io` for esp32c6 (works!)

The examples are mostly the same:

display_task:
- generates an image in a frame buffer and sends it via embassy Signal to hub75_task
- waits for return frame buffer from hub75_task
- repeats forever

hub75_task:
- loops forever sending its current frame buffer buffer out via DMA
- each iteration its checks a Signal for a new frame buffer
- if there is a buffer its swaps its with its current one and sends the old one back via signal

main task:
- increments a counter evert 100ms

The main branch works on the released esp-hal - the with-latch-next branch is updated for current git.

I use this `.cargo/config.toml`

```
[alias]
erase     = "espflash erase-parts --partition-table partitions.csv app0"
run-32    = "run   --release --target xtensa-esp32-none-elf        --example i2s_parallel       --features esp32,log -- -B 1152000 -p /dev/cu.usbserial-14244301"
run-32-l  = "run   --release --target xtensa-esp32-none-elf        --example i2s_parallel_latch --features esp32,log -- -B 115200  -p /dev/cu.usbserial-00000000"
run-s3    = "run   --release --target xtensa-esp32s3-none-elf      --example lcd_cam      --features esp32s3,defmt -- --probe 303a:1001:80:65:99:BA:7C:B0"
run-c6    = "run   --release --target riscv32imac-unknown-none-elf --example parl_io      --features esp32c6,defmt -- --probe 303a:1001:40:4C:CA:42:94:18"

# just build
build-32  = "build --release --target xtensa-esp32-none-elf        --example i2s_parallel --features esp32,log"
build-32-l= "build --release --target xtensa-esp32-none-elf        --example i2s_parallel_latch --features esp32,log"
build-s3  = "build --release --target xtensa-esp32s3-none-elf      --example lcd_cam      --features esp32s3,defmt"
build-c6  = "build --release --target riscv32imac-unknown-none-elf --example parl_io      --features esp32c6,defmt"

[build]
# target = "xtensa-esp32s3-none-elf"
# target = "riscv32imac-unknown-none-elf"

[target.xtensa-esp32-none-elf]
runner = "espflash flash --chip esp32 --monitor"
rustflags = [
  "-C", "link-arg=-Tlinkall.x",
  "-C", "link-arg=-nostartfiles",
  "-C", "link-arg=-Tdefmt.x",
]

[target.xtensa-esp32s3-none-elf]
runner = "probe-rs run --chip esp32s3 --no-location"
rustflags = [
  "-C", "link-arg=-Tlinkall.x",
  "-C", "link-arg=-nostartfiles",
  "-C", "link-arg=-Tdefmt.x",
]

[target.riscv32imac-unknown-none-elf]
runner = "probe-rs run --chip esp32c6 --no-location"
rustflags = [
  "-C", "link-arg=-Tlinkall.x",
  "-C", "link-arg=-Tdefmt.x",
]

[unstable]
build-std = ["alloc", "core"]

[env]
# log level
DEFMT_LOG = "info"
```
