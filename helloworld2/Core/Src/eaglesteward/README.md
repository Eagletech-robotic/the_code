```bash
g++ -Wall -Wextra -o3 \
     -D STANDALONE \
     helloworld2/Core/Src/eaglesteward/thibault.cpp \
     helloworld2/Core/Src/utils/*.cpp \
     -Ihelloworld2/Core/Inc/ \
     -o build/thibault \
     && ./build/thibault
```
