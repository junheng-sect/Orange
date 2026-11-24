# Noetic/Focal amd64 build and test environment. The workspace is supplied at run time.
FROM osrf/ros:noetic-desktop-full@sha256:7dbfb9576d8e6d226c31e06129a82aaab8702695f38eca2116918cb9b9308797

# Focal's default C++ library lacks <format>. Use GCC 13 for ZXing only.
RUN set -eux; \
    fingerprint=C8EC952E2A0E1FBDC5090F6A2C277A0A352154E5; \
    curl -fsSL --retry 3 "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x${fingerprint}" -o /tmp/toolchain.asc; \
    test "$(gpg --show-keys --with-colons /tmp/toolchain.asc | awk -F: '$1=="fpr"{print $10; exit}')" = "$fingerprint"; \
    gpg --batch --dearmor -o /usr/share/keyrings/ubuntu-toolchain-test.gpg /tmp/toolchain.asc; \
    echo 'deb [signed-by=/usr/share/keyrings/ubuntu-toolchain-test.gpg] https://ppa.launchpadcontent.net/ubuntu-toolchain-r/test/ubuntu focal main' > /etc/apt/sources.list.d/ubuntu-toolchain-test.list; \
    apt-get -o Acquire::Retries=3 update; \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        g++-13 python3-catkin-tools ros-noetic-mavros ros-noetic-mavros-msgs \
        ros-noetic-camera-calibration libceres-dev libssl-dev libeigen3-dev libyaml-cpp-dev \
        libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
        libavcodec-dev libavformat-dev libavutil-dev libv4l-dev v4l-utils ffmpeg \
        python3-dev python3-nose python3-numpy python3-opencv python3-matplotlib \
        python3-websocket python3-yaml poppler-utils; \
    rm -rf /var/lib/apt/lists/* /tmp/toolchain.asc

RUN set -eux; \
    curl -fsSL --retry 3 https://codeload.github.com/zxing-cpp/zxing-cpp/tar.gz/refs/tags/v3.0.0 -o /tmp/zxing.tar.gz; \
    echo 'a1b387f9968a26a1821e7d11d7021ed71e5316fac2487b2a65f1a53a5cde5efc  /tmp/zxing.tar.gz' | sha256sum -c -; \
    mkdir /tmp/zxing-src; tar -xzf /tmp/zxing.tar.gz --strip-components=1 -C /tmp/zxing-src; \
    cmake -S /tmp/zxing-src -B /tmp/zxing-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++-13 \
        -DZXING_WRITERS=OFF -DZXING_EXAMPLES=OFF -DZXING_C_API=OFF -DZXING_DEPENDENCIES=LOCAL; \
    cmake --build /tmp/zxing-build -j4; cmake --install /tmp/zxing-build; ldconfig; \
    rm -rf /tmp/zxing-src /tmp/zxing-build /tmp/zxing.tar.gz

# Match the board detector; Focal and the Noetic snapshot ship older versions.
RUN set -eux; \
    curl -fsSL --retry 3 https://codeload.github.com/AprilRobotics/apriltag/tar.gz/refs/tags/v3.4.5 -o /tmp/tag.tar.gz; \
    echo '3749f48220db78c29c375a373861f4e1f4138e5dbae3c298584e96cfcc556515  /tmp/tag.tar.gz' | sha256sum -c -; \
    mkdir /tmp/tag-src; tar -xzf /tmp/tag.tar.gz --strip-components=1 -C /tmp/tag-src; \
    cmake -S /tmp/tag-src -B /tmp/tag-build -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=OFF -DBUILD_PYTHON_WRAPPER=OFF; \
    cmake --build /tmp/tag-build -j4; cmake --install /tmp/tag-build; ldconfig; \
    rm -rf /tmp/tag-src /tmp/tag-build /tmp/tag.tar.gz
