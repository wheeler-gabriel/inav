# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

INAV is an open-source navigation-capable flight controller firmware written in C, targeting STM32 F4/F7/H7 and AT32 microcontrollers. It supports multicopters, fixed-wing aircraft, rovers, and boats. The project is a fork of Cleanflight/Betaflight with heavy focus on autonomous navigation (GPS, waypoints, RTH, position hold).

## Build System

INAV uses CMake with a cross-compiler (`arm-none-eabi-gcc 13.2.1`). CMake downloads the toolchain automatically on first run.

### Setup (first time)
```sh
mkdir build && cd build
cmake ..
```

### Building firmware
```sh
# From the build/ directory
make MATEKF405SE          # single target
make -j$(nproc) MATEKF405SE MATEKF722  # parallel multi-target
make help | less          # list all ~200 targets
```

Output files: `build/inav_<version>_<TARGET>.hex` and `bin/<TARGET>.elf`

### Building with ninja (faster)
```sh
cmake -GNinja ..
ninja MATEKF405SE
```

### SITL (Software In The Loop) — runs on the host
```sh
mkdir build_SITL && cd build_SITL
cmake -DSITL=ON ..
make
```

### Running unit tests
```sh
mkdir testing && cd testing
cmake -DTOOLCHAIN= ..
make check
# Run a single test binary directly:
src/test/unit/time_unittest
```

### Docker build
```sh
./build.sh MATEKF405SE    # Linux/WSL
```

### Cleaning
```sh
make clean              # clean all
make clean_MATEKF405SE  # clean single target
```

## Code Architecture

### Directory Structure

- `src/main/` — all firmware source code
  - `fc/` — flight controller core: initialization (`fc_init.c`), main task loop (`fc_core.c`), CLI (`cli.c`), RC processing, MSP handling, and **`settings.yaml`** (the canonical definition of all user-configurable settings)
  - `flight/` — PID controller, IMU, mixer, failsafe, gyro filtering (dynamic notch, RPM filter, Kalman), servos, VTOL mixer profiles
  - `navigation/` — position/altitude estimator, multicopter nav, fixed-wing nav, launch detection, geozone enforcement, RTH trackback
  - `sensors/` — abstraction layer for gyro, accelerometer, barometer, compass, GPS, rangefinder, pitot, optical flow, ESC telemetry
  - `drivers/` — hardware abstraction: SPI/I2C buses, UART, DMA, timers, PWM output, specific sensor ICs under `accgyro/`, `barometer/`, `compass/`, etc.
  - `io/` — higher-level I/O: OSD, GPS parsing, serial communication, flashfs, beeper, LED strip, gimbal, VTX
  - `rx/` — RC receiver protocols: SBUS, CRSF, IBUS, GHST, FPORT, SRXL2, Spektrum, MAVLink
  - `msp/` — MultiWii Serial Protocol implementation
  - `telemetry/` — outbound telemetry: MAVLink, SmartPort, FrSky, LTM, HOTT, CRSF
  - `programming/` — Logic Conditions and Global Variables (in-firmware scripting)
  - `blackbox/` — flight data logger
  - `scheduler/` — cooperative task scheduler (`cfTask_t` in `fc/fc_tasks.c`)
  - `target/` — per-board definitions (~200 boards), each with `target.h` (pin/peripheral defines) and `target.c`
  - `cms/` — on-screen menu system
- `src/test/unit/` — Google Test unit tests (compiled natively, not for the target)
- `cmake/` — toolchain files, settings generation, target helper macros

### Key Architectural Concepts

**Settings system**: All user-configurable parameters are declared in `src/main/fc/settings.yaml`. A Ruby script (`src/utils/settings.rb`) processes this file at build time to generate `settings_generated.c/h`. Adding a new CLI parameter means editing `settings.yaml`, not writing C directly.

**Parameter Groups (PGs)**: Runtime configuration uses a registry-based system defined in `config/parameter_group.h`. Each subsystem declares its config struct via `PG_DECLARE`/`PG_REGISTER` macros, enabling automatic EEPROM save/load. Profile-scoped configs use `PG_SIZE_PROFILE_FLAG`.

**Scheduler**: The firmware runs a cooperative task scheduler (not an RTOS). Tasks are registered in `fc/fc_tasks.c` as `cfTask_t` entries with a desired period and callback. Tasks call `rescheduleTask()` to adjust their rate dynamically.

**Target definitions**: Each board lives in `src/main/target/<BOARDNAME>/`. The `target.h` defines hardware pins, bus assignments, and `USE_*` feature flags. `src/main/target/<BOARDNAME>/CMakeLists.txt` registers the target with a macro like `target_stm32f405xg(BOARDNAME)`. New source files must be added to `src/main/CMakeLists.txt`.

**Platform abstraction**: `src/main/platform.h` is the top-level include that pulls in the MCU-specific platform header based on `TOOLCHAIN`. SITL builds use a host-native platform (`src/main/target/SITL/`).

### Coding Conventions

- C99 standard (C11 on macOS SITL)
- Boolean-returning functions named as questions: `isOkToArm()`
- `find*()` functions may return NULL; `get*()` functions must not
- `USE_*` preprocessor defines gate hardware features; match guards in `target.h` with guards in driver code
- Tests are written in C++ and link against the C firmware code

## Navigation System

The navigation stack (`src/main/navigation/`) is the most complex subsystem:
- `navigation.c` — state machine, mode management, common entry points
- `navigation_pos_estimator.c` — sensor fusion for position/velocity estimation (GPS, optical flow, baro, rangefinder)
- `navigation_multicopter.c` — copter-specific position/altitude controllers
- `navigation_fixedwing.c` / `navigation_fw_launch.c` — fixed-wing nav and autolaunch
- `navigation_rover_boat.c` — ground/water vehicle nav
- `navigation_geozone.c` — geofencing

## Useful References

- Target list: `src/main/target/` (directory names are valid `make` targets)
- All CLI settings: `src/main/fc/settings.yaml`
- MSP protocol definitions: `src/main/msp/msp_protocol*.h`
- Supported MCU platforms: `cmake/arm-none-eabi-checks.cmake` (currently expects GCC 13.2.1)
- SITL documentation: `docs/development/Building SITL.md`
- Full build docs: `docs/development/Building in Linux.md`
