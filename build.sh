## prepare environment
export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu-
export LANG=en_US.UTF-8

## build image
# make defconfig
make -j4 HOSTCFLAGS="-I./macos/include -I/opt/homebrew/Cellar/openssl@3/3.2.1/include -L/opt/homebrew/Cellar/openssl@3/3.2.1/lib -I/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/usr/include" V=1

