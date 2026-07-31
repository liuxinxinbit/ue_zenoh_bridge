# zenoh-c 1.9.0

This directory vendors the zenoh-c 1.9.0 release artifacts required by
`ue_zenoh_bridge`. The package does not search for or link against a system
zenoh-c installation.

Upstream:

- Project: https://github.com/eclipse-zenoh/zenoh-c
- Release: https://github.com/eclipse-zenoh/zenoh-c/releases/tag/1.9.0
- License: EPL-2.0 OR Apache-2.0

Bundled target:

- Linux x86_64: `lib/linux-x86_64/libzenohc.so`
- SHA-256:
  `79bd2c8ae6610778a625e84fbdcc0e1da99a5d42ac94f8dc4cf8b5c42a915cd1`

The shared library is installed next to the ROS 2 executable. The executable
uses an `$ORIGIN` install RPATH, so no system installation or
`LD_LIBRARY_PATH` setting is required.

To add another operating system or architecture, place the matching official
zenoh-c 1.9.0 headers and library in a platform-specific subdirectory and add
an explicit platform branch in the package `CMakeLists.txt`. Do not silently
fall back to a system library because mixing zenoh-c headers and binaries from
different releases is unsafe.
