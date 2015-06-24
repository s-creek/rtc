#!/bin/sh
cd `dirname $0`

. ./config.sh
UNAME=`uname`
#CMAKE_OPT="-DCMAKE_INSTALL_PREFIX:STRING=/opt/grx $@  -DCOMP_NAME=${COMP_NAME}"
CMAKE_OPT="-DCMAKE_INSTALL_PREFIX:STRING=/opt/grx $@  -DCOMP_NAME=${COMP_NAME} -DROBOT_NAME=${ROBOT_NAME}"
MAKE_OPT="VERBOSE=1 $@"

# for logging
DATE=`/bin/date '+%Y%m%d%H%M'`
mkdir -p log


# cmake
if [ "$UNAME" = "QNX" ]; then
    CMAKE_OPT="$CMAKE_OPT -DBOOST_ROOT=/usr/pkg"
    export CXX=QCC
    export CC=qcc
fi
cmake . ${CMAKE_OPT} 2>&1 | tee log/build_${DATE}.log


# make
if [ "$UNAME" = "QNX" ]; then
    for F in `find . -name link.txt -or -name relink.txt`;
    do 
	sed -e "s/-export-dynamic/-Wl,-export-dynamic/g " $F > $F.new;
	mv $F.new $F;
    done
fi
make ${MAKE_OPT}  2>&1 | tee -a log/build_${DATE}.log

cd log
ln -sfv build_${DATE}.log build.log