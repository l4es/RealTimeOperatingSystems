Building with cake
==================

For genuine build, omit -DCMAKE_TOOLCHAIN_FILE=YYY
Use XX=32 for mingw32 and XX=64 for mingw64 builds.

Example setup:

cd nutconf
mkdir winXX
cd winXX
cmake -DCMAKE_TOOLCHAIN_FILE=../../Toolchain-mingwXX.cmake ..
cd ../..

cd qnutconf
mkdir winXX
cd winXX
cmake -DCMAKE_TOOLCHAIN_FILE=../../Toolchain-mingwXX.cmake ..
cd ../..
