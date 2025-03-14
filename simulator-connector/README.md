## Installation

To compile our C code to WASM, we use Emscripten, a compiler that uses CLang.

To install Emscripten, follow the instructions on the [official website](https://emscripten.org/docs/getting_started/downloads.html):

- Clone the git repository.
- Follow instructions in the documentation, including sourcing `/var/lib/emsdk/emsdk_env.sh` in your `.bashrc` file.

Emscripten comes with various tools, including `emcc`.

## Compilation

To compile the simulator, move to the root folder, then run the following command:

```bash
emcc simulator-connector/simulator-connector.c -o3 -g -s ASSERTIONS=1 \
     -D THIBAULT_AUTOPILOT \
     helloworld2/Core/Src/eaglesteward/falcon.c \
     helloworld2/Core/Src/eaglesteward/thibault.cpp \
     helloworld2/Core/Src/utils/*.c \
     helloworld2/Core/Src/utils/*.cpp \
     -I helloworld2/Core/Inc/ \
     -o simulator-connector/simulator-connector.wasm &&
     mv simulator-connector/simulator-connector.wasm ../simulator/public/
```
