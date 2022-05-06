FROM ros:foxy as hakoniwa-ros-simulator

ARG DEBIAN_FRONTEND=noninteractive

ENV CMAKE_INSTALL_DIR /local
ENV PATH $PATH:/grpc/cmake/build
ENV PATH $PATH:/grpc/cmake/build/third_party/protobuf
ENV PATH $PATH:$CMAKE_INSTALL_DIR/bin

RUN apt-get update && apt-get install -y \
	git	\
	build-essential	\
	wget	\
	gcc	\
	g++	\
	ruby	\
	vim	\
	libssl-dev \
	make	\
	autoconf \
	automake \
	pkg-config \
	curl \
	&& apt-get clean \
	&& rm -rf /var/lib/apt/lists/*

RUN	wget -q -O cmake-linux.sh https://github.com/Kitware/CMake/releases/download/v3.17.0/cmake-3.17.0-Linux-x86_64.sh && \
	mkdir -p $CMAKE_INSTALL_DIR && \
	sh cmake-linux.sh --skip-license --prefix=$CMAKE_INSTALL_DIR && \
	rm cmake-linux.sh

# Install grpc
RUN mkdir -p /root/grpc-build && \
	cd /root/grpc-build && \
	git clone --recurse-submodules -b v1.35.0 https://github.com/grpc/grpc.git && \
	cd grpc && \
	git submodule update --init && \
	mkdir -p cmake/build && \
	cd cmake/build && \
	cmake ../.. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local/grpc -DgRPC_BUILD_CSHARP_EXT=OFF -DOPENSSL_ROOT_DIR=/usr/local/boringssl -DCMAKE_PREFIX_PATH="/usr/local/cares;/usr/local/protobuf;/usr/local/zlib" && \
	make -j4 && \
	make install && \
	cd /root && \
	rm -rf grpc-build
ENV PATH /usr/local/grpc/bin:${PATH}

# Install grpc for Ruby
RUN	gem install grpc grpc-tools

RUN mkdir /root/workspace
WORKDIR /root/workspace

#hakoniwa-core
RUN git clone --recurse-submodules https://github.com/toppers/hakoniwa-core.git

#install hakoniwa command
WORKDIR /root/workspace/hakoniwa-core/impl/asset/client/build
RUN bash build.bash cpp
RUN git pull && cp -rp ../cpp/bin/* /root/workspace/hakoniwa-core/impl/asset/client/build/cpp/

ENV PATH /root/workspace/hakoniwa-core/impl/asset/client/build/cpp/:${PATH}

WORKDIR /root/workspace

#install python
RUN apt update &&  apt install -y python && \
	curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py && \
	python get-pip.py && \
	pip install jinja2

WORKDIR /root/workspace/hakoniwa-ros2sim/ros2/workspace
RUN chmod +x /root/workspace/hakoniwa-core/impl/asset/client/build/cpp/hako-*
ENV CORE_PORTNO 50051
