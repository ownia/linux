## prepare environment
export ARCH=arm64
export CROSS_COMPILE=aarch64-unknown-linux-gnu-

## build image
# make defconfig
# make -j8 HOSTCFLAGS="-I./macos/include -I/opt/homebrew/Cellar/openssl@3/3.0.5/include -I/Library/Developer/CommandLineTools/SDKs/MacOSX13.0.sdk/usr/include/machine"
make -j8 HOSTCFLAGS="-I./macos/include -I/opt/homebrew/Cellar/openssl@3/3.0.5/include -L/opt/homebrew/Cellar/openssl@3/3.0.5/lib -I/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/usr/include"

