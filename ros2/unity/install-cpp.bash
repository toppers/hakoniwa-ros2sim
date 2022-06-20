#!/bin/bash


cd tb3/Assets

if [ ${OS_TYPE} = "Mac" ]
then
	LIBNAME=libshakoc.dylib
else
	LIBNAME=libshakoc.so
fi

if [ -f Plugin/${LIBNAME} ]
then
	:
else
	wget https://github.com/toppers/hakoniwa-core/releases/download/v1.1.0/${LIBNAME}
	mv ${LIBNAME} Plugin/
fi

