# 删除CMake缓存文件和临时目录（关键）
rm -rf CMakeCache.txt CMakeFiles/

# 重新执行cmake
mkdir -p build && cd build
cmake ..

# 编译
make -j8

