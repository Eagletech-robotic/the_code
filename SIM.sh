source ../emsdk/emsdk_env.sh
./build.sh build --wasm && cp build-wasm/host-tools/simulator-connector.wasm ../simulator/public/
