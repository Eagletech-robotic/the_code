## Host Tools

This directory contains host-side tools that run on your development machine, not on the robot. These tools help with
simulation and debugging.

## thibault_debug.cpp

### First building

```bash
# From the project root
mkdir -p build && cd build
cmake ..
make thibault_debug
ln -sf host-tools/thibault_debug thibault_debug
```

### Subsequent building and running

```bash
# From the project root
cd build
make clean # If you want to force a full rebuild
make thibault_debug
./thibault_debug
```

## simulator_connector.cpp

This connector embeds the robot guidance code into a WASM binary used by our browser-based physics simulator.

### Requirements

Requires Emscripten to compile C/C++ to WebAssembly.

Install Emscripten by following the instructions on
the [official website](https://emscripten.org/docs/getting_started/downloads.html):

- Clone the git repository
- Follow the documentation, including sourcing `/var/lib/emsdk/emsdk_env.sh` in your `.bashrc` file

### First building

```bash
# From the project root
mkdir -p build-wasm && cd build-wasm
emcmake cmake ..
emmake make simulator_connector
cp host-tools/simulator-connector.wasm ../../simulator/public
```

### Subsequent building and running

```bash
# From the project root
cd build-wasm
make clean && \
  make simulator_connector && \
  cp host-tools/simulator-connector.wasm ../../simulator/public
```

### Usage

The WASM file is used by the browser-based simulator found in the `simulator` repository. Open the simulator in a web
browser to see the robot simulation in action.
