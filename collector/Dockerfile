FROM ubuntu:20.04 as build
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    build-essential cmake libbluetooth-dev libcurl4-openssl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
COPY CMakeLists.txt main.cpp /work/
RUN cmake -S . -B build && cmake --build build

FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    bluez bluetooth usbutils libbluetooth-dev libcurl4-openssl-dev \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build /work/build/nrf52_sht30_collector /nrf52_sht30_collector
COPY startup.sh /startup.sh
CMD ["/startup.sh"]