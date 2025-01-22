## Installation

To compile our C code to WASM, we use Emscripten, a compiler that uses CLang.

To install Emscripten, follow the instructions on the [official website](https://emscripten.org/docs/getting_started/downloads.html):

- Clone the git repository.
- Follow instructions in the documentation, including sourcing `/var/lib/emsdk/emsdk_env.sh` in your `.bashrc` file.

Emscripten comes with various tools, including `emcc`.

## Compilation

To compile the simulator, move to this folder, then use the following command:

```bash
emcc simulator-connector.c -o2 \
     -D MODE_THIBAULT \
     ../helloworld2/Core/Src/eaglesteward/*.c \
     ../helloworld2/Core/Src/eaglesteward/*.cpp \
     ../helloworld2/Core/Src/robotic/*.c \
     -I../helloworld2/Core/Inc/ \
     -lm \
     -o simulator-connector.wasm &&
     mv simulator-connector.wasm ../../simulator/public/
```
