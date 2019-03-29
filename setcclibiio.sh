
#Install arm-linux-gnueabihf
sudo apt install gcc-arm-linux-gnueabihf

#Cross-compilation libxml2
wget http://www.xmlsoft.org/sources/libxml2-2.9.3.tar.gz
tar zxvf libxml2-2.9.3.tar.gz

cd libxml2-2.9.3

./configure --host=arm-linux --build=amd-linux --target=arm --prefix=/usr/local/arm/libxml2_arm CC=arm-linux-gnueabihf-gcc  --without-zlib  --without-python

make 

sudo make install

#Cross-compilation zlib

wget http://zlib.net/zlib-1.2.11.tar.gz

tar zxvf zlib-1.2.11.tar.gz

cd zlib-1.2.11.tar.gz

export CC=arm-linux-gnueabihf-gcc

./configure --prefix=/usr/local/arm/zlib_arm

make

sudo make install

#Cross-compilation libiio

git clone -b 2018_R1 https://github.com/analogdevicesinc/libiio

cd libiio

touch toolchainfile.cmake
echo "# this one is important
SET(CMAKE_SYSTEM_NAME Linux)

#this one not so much
SET(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
SET(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)

# where is the target environment
SET(CMAKE_FIND_ROOT_PATH $PWD)

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
#SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)

# for libraries and headers in the target directories
#SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
#SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)" > toolchainfile.cmake

sed -i 's/CMAKE_EXE_LINKER_FLAGS:STRING=/CMAKE_EXE_LINKER_FLAGS:STRING= -L /usr/local/arm/libxml2_arm/lib -lxml2 -L /usr/local/arm/zlib_arm/lib -lz/g' CMakeCache.txt

touch config.sh
echo "#!/bin/bash
TOOLCHAIN_PATH=$PWD
cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_PATH}/toolchainfile.cmake" > config.sh


source config.sh

make 

sudo make install













