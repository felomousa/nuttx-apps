# readylist: ready-to-run list insertion test

When an interrupt handler calls `sem_post()`, NuttX disables interrupts
(`enter_critical_section()` in `nxsem_post_slow()`) and inserts the woken
task into the ready-to-run list with `nxsched_add_prioritized()`. That
insertion walks the sorted list one entry at a time and stops at the first
task of *lower* priority. So the time interrupts stay disabled grows with
the number of ready tasks whose priority is at or above the woken task's.

This test reproduces that, counts the steps, and times them.

## How it works

| Thread | Priority | Role |
|---|---|---|
| main | 200 | The task the interrupt preempts. It busy-waits. |
| W | `-w` (100) | Blocked on a semaphore; the interrupt wakes it. |
| N fillers | `-f` (100) | Made ready before each wake-up. They cannot run, because main outranks them. |

For each wake-up, a watchdog (which runs in the timer interrupt) does this:

1. It scans the whole ready-to-run list and counts the tasks with priority
   ≥ W's. This is the oracle.
2. It calls `sem_post()` and times it with `perf_gettime()`.
3. It reads `g_readylist_visits`, the number of entries the kernel stepped
   over. This needs `CONFIG_SCHED_READYLIST_STATS`; without it the test
   only times, which is how to get numbers from an uninstrumented kernel.

Columns:

- `visits`: the expected step count, which is
  `(200 >= W ? 1 : 0) + (fillers ready && F >= W ? N : 0)`.
- `min/med/mean/max`: how long `sem_post()` took, over wake-ups that saw
  exactly the expected ready tasks. The unit is ns, or perf counter ticks
  with `-c` (CPU cycles on the ESP32).
- `wrong`: wake-ups where the kernel's count differed from the oracle, or
  that did not run in interrupt context. Any non-zero value fails the test.
- `other`: wake-ups where some other task (usually `hpwork`, woken by the
  same tick) was also ready. They are left out of the timing and do not
  fail the test.

## Running on the simulator

```
./tools/configure.sh sim:readylist
make -j
./nuttx
nsh> readylist -n 256 -s 32 -i 200          # fillers at W's priority: grows with N
nsh> readylist -n 256 -s 64 -i 200 -f 50    # fillers below W: flat, 1 visit
nsh> readylist -n 256 -s 64 -i 200 -b       # fillers blocked: flat, 1 visit
nsh> readylist -n 256 -s 64 -i 200 -w 210   # W above everything: flat, 0 visits
nsh> cat /proc/irqs                         # max time per IRQ (SCHED_IRQMONITOR)
```

On macOS, `CLOCK_MONOTONIC` and therefore the sim's `perf_gettime()` ticks
in 1 µs steps, so individual samples are quantized. Use the `mean` column and the
fitted cost per filler. Linux hosts have nanosecond resolution. Simulator
timings describe the host CPU, not a microcontroller; use real hardware for
the per-entry cost.

On macOS, if a rebuild seems to run old code, delete `apps/libapps.a` and
the `.built` files under `apps/`: the macOS `ar` appends duplicate members
and the linker keeps the first (stale) one.

## Running on an ESP32 (single core)

```
./tools/configure.sh esp32-devkitc:readylist
make -j
make flash ESPTOOL_PORT=/dev/cu.usbserial-XX ESPTOOL_BINDIR=./ ESPTOOL_BAUD=460800
nsh> readylist -n 224 -s 32 -i 200 -c
```

- Use `-c`. It reports `CCOUNT` cycles, while ns are currently wrong on
  xtensa because nothing calls `up_perf_init()`.
- A WROOM module without PSRAM fits about 240 fillers with 1 KiB stacks.
- Expected result: 324 cycles at N = 0, plus 14 cycles per ready task at
  W's priority or above.
- With CH340 adapters on macOS, flashing at the default 921600 baud can
  fail, so use `ESPTOOL_BAUD=460800`.

## Counting lost UART bytes (`-u`)

An external sender (for example `pico/pico_uart_sender`) streams
0x00..0xFF nonstop into ESP32 UART2 RX (GPIO 18) at 921600 baud. Use
`esp32-devkitc:readylist_uart`, which sets the RX FIFO interrupt threshold
to 127 (one byte of headroom) and enables a separate interrupt stack.

```
nsh> readylist -n 208 -s 16 -i 1000 -c -d -f 20 -w 20 -u /dev/ttyS1    # W below every filler
nsh> readylist -n 208 -s 16 -i 1000 -c -d -f 20 -w 210 -u /dev/ttyS1   # control: W above all
```

- `-d` spreads the fillers over priorities 20..199 and posts them lowest
  first. Without it, posting N equal-priority fillers is itself a long
  walk with interrupts disabled, and that causes loss of its own.
- Trust `ovf` (the hardware RX FIFO overflow flag), not `lost`. After an
  overflow the ESP32 FIFO is left inconsistent, because the driver does
  not reset it, so `lost` comes out in 128-byte steps.

## Notes

- `CONFIG_SCHED_CRITMONITOR` does **not** see this critical section:
  `enter_critical_section()` skips the monitor in interrupt context. For
  ISR-side cost, use this test or `CONFIG_SCHED_IRQMONITOR` (`/proc/irqs`).
- The test reads the kernel's `g_readytorun` directly, so it needs a flat
  build.
- `CONFIG_SCHED_READYLIST_STATS` adds a counter increment to every step of
  the walk. For final timing numbers, also run with it disabled.
