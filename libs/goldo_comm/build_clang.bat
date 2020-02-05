mkdir build
mkdir build\clang
cd build\clang
cmake -G Ninja -DCMAKE_CONFIG=Release -DCMAKE_TOOLCHAIN_FILE=..\..\clang-toolchain.cmake ..\..
cmake --build .
cd ../..