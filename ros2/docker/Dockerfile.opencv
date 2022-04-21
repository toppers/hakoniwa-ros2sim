FROM ros:foxy as hakoniwa-ros2-builder

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

ENV PATH /root/workspace/hakoniwa-core/impl/asset/client/build/cpp/:${PATH}

WORKDIR /root/workspace

#install python
RUN apt update &&  apt install -y python && \
	curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py && \
	python get-pip.py && \
	pip install jinja2

#install opencv
RUN apt update -y && apt upgrade -y
RUN apt-get install -y libjpeg-dev 
RUN apt-get install -y libpng-dev 
RUN apt-get install -y libtiff-dev
RUN apt-get install -y build-essential cmake pkg-config unzip yasm git checkinstall
RUN apt-get install -y libjpeg-dev libpng-dev libtiff-dev
RUN apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libavresample-dev
RUN apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
RUN apt-get install -y libxvidcore-dev x264 libx264-dev libfaac-dev libmp3lame-dev libtheora-dev
RUN apt-get install -y libfaac-dev libmp3lame-dev libvorbis-dev
RUN apt-get install -y libopencore-amrnb-dev libopencore-amrwb-dev
RUN apt-get install -y libdc1394-22 libdc1394-22-dev libxine2-dev libv4l-dev v4l-utils
RUN cd /usr/include/linux \
	&& ln -s -f ../libv4l1-videodev.h videodev.h
RUN apt-get install -y libgtk-3-dev
RUN apt-get install -y python3-dev python3-pip
RUN sudo -H pip3 install -U pip numpy
RUN apt-get install -y python3-testresources
RUN apt-get install -y libtbb-dev
RUN apt-get install -y libatlas-base-dev gfortran
RUN apt-get install -y libprotobuf-dev protobuf-compiler libgoogle-glog-dev libgflags-dev libgphoto2-dev libeigen3-dev libhdf5-dev doxygen
RUN mkdir /root/workspace/opencv

WORKDIR  /root/workspace/opencv
RUN mkdir opencv_build && cd opencv_build \
	&& git clone https://github.com/opencv/opencv  \
	&& git clone https://github.com/opencv/opencv_contrib
WORKDIR /root/workspace/opencv/opencv_build/opencv_contrib
WORKDIR /root/workspace/opencv/opencv_build/opencv
RUN mkdir -p build && cd build \
	&& cmake \
	-D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_C_COMPILER=/usr/bin/gcc \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D INSTALL_PYTHON_EXAMPLES=ON \
	-D INSTALL_C_EXAMPLES=OFF \
	-D WITH_TBB=ON \
	-D WITH_CUDA=OFF \
	-D BUILD_opencv_cudacodec=OFF \
	-D ENABLE_FAST_MATH=1 \
	-D CUDA_FAST_MATH=1 \
	-D WITH_V4L=ON \
	-D WITH_QT=OFF \
	-D WITH_OPENGL=ON \
	-D WITH_GSTREAMER=ON \
	-D OPENCV_GENERATE_PKGCONFIG=ON \
	-D OPENCV_PC_FILE_NAME=opencv.pc \
	-D OPENCV_EXTRA_MODULES_PATH=/root/workspace/opencv/opencv_build/opencv_contrib/modules \
	-D BUILD_EXAMPLES=ON ..
WORKDIR /root/workspace/opencv/opencv_build/opencv/build
RUN make -j8
RUN make install

#install mqtt
RUN apt-get install -y software-properties-common
RUN add-apt-repository ppa:mosquitto-dev/mosquitto-ppa
RUN apt-get install -y libmosquitto-dev

WORKDIR /root/workspace/hakoniwa-ros-samples/ros2/workspace
