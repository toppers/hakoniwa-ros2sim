#!/bin/bash


cd tb3/Assets

if [ ${OS_TYPE} = "Mac" ]
then
	LIBNAME=libshakoc.dylib
else
	LIBNAME=libshakoc.so
fi

if [ -d Plugin ]
then
	:
else
	mkdir Plugin
fi

wget https://github.com/toppers/hakoniwa-core/releases/download/v2.0.0/Hakoniwa_NoGRPC.dll
echo ARCH_TYPE = ${ARCH_TYPE}
if [ ${ARCH_TYPE} = "arm64" ]
then
	wget https://github.com/toppers/hakoniwa-core-cpp-client/releases/download/v1.0.1/libshakoc.${ARCH_TYPE}.dylib
	mv libshakoc.${ARCH_TYPE}.dylib ${LIBNAME}
else
	wget https://github.com/toppers/hakoniwa-core/releases/download/v2.0.0/${LIBNAME}
fi
mv Hakoniwa_NoGRPC.dll Plugin/
mv ${LIBNAME} Plugin/

if [ -d Model ]
then
	:
	echo "Model is already installed."
else
	wget https://github.com/toppers/hakoniwa-ros2sim/releases/download/v1.0.0/Model.tar.gz
	tar xzvf Model.tar.gz
	rm -f Model.tar.gz
fi

if [ -d Resources/Hakoniwa/Robots ]
then
	echo "Resources is already installed."
else
	mkdir -p Resources/Hakoniwa/Robots
	cp -p Prefabs/Robots/TB3RoboModel.prefab*  Resources/Hakoniwa/Robots/
fi
