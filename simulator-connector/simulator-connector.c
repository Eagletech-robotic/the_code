// Compile with:
// emcc simulator-connecter.c -o simulator-connecter.wasm && mv simulator-connecter.wasm
// ../../simulator/

#include <emscripten/emscripten.h>
#include <stdio.h>

EMSCRIPTEN_KEEPALIVE int exported_log_42() {
    puts("Hello, World!\n");
    return 53;
}

int main() { return 0; }
