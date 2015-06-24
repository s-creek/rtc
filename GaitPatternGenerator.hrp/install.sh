#!/bin/sh
cd `dirname $0`

if [ ! -e Makefile ]; then
    exit
fi

# for logging
DATE=`/bin/date '+%Y%m%d%H%M'`
mkdir -p log

make install 2>&1 | tee log/install_${DATE}.log
cd log
ln -sfv install_${DATE}.log install.log