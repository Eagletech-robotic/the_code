## Installation

To compile our C code to WASM, we use Emscripten, a compiler that uses CLang.

To install Emscripten, follow the instructions on the [official website](https://emscripten.org/docs/getting_started/downloads.html):

- Clone the git repository
- Follow instructions to configure the SDK

Emscripten comes with various tools, including `emcc`.

## Compilation

To compile the simulator, use the following command:

```bash
emcc simulator-connector.c \
     ../helloworld2/Core/Src/eaglesteward/falcon.c \
     ../helloworld2/Core/Src/robotic/carre.c \
     ../helloworld2/Core/Src/robotic/inertial.c \
     ../helloworld2/Core/Src/robotic/pid.c \
     -I../helloworld2/Core/Inc/ \
     -lm \
     -o simulator-connector.wasm && mv simulator-connector.wasm ../../simulator/public/
```
