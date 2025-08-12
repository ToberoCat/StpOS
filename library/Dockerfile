FROM arm64v8/python:3.11-bookworm
LABEL authors="Tobias Madlberger"

RUN apt-get update && apt-get install -y git wget libx11-dev zlib1g-dev cmake swig ninja-build \
                          build-essential libc-dev libasio-dev libc6-dev libpthread-stubs0-dev libssl-dev  \
                          libzbar-dev libopencv-dev libjpeg-dev python3-dev doxygen yasm git build-essential  \
                          cmake libzbar-dev libopencv-dev libjpeg-dev python3-dev doxygen swig yasm \
                          libcurl4-openssl-dev ninja-build

RUN git clone https://github.com/gabime/spdlog.git && \
    cd spdlog && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON .. && make -j && make install

COPY . /tmp/app
RUN pip install ninja --break-system-packages
WORKDIR /app
ENV CMAKE_GENERATOR=Ninja
ENV CMAKE_BUILD_PARALLEL_LEVEL=12