```bash
g++ -Wall -Wextra -o3 \
     -D STANDALONE \
     helloworld2/Core/Src/eaglesteward/thibault.cpp \
     helloworld2/Core/Src/utils/*.c \
     helloworld2/Core/Src/utils/*.cpp \
     -Ihelloworld2/Core/Inc/ \
     -lm \
     -o build/thibault
```
