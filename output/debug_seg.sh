#!/bin/bash

BIN=./cw_DealRCF_nebulalink
ROOT=$(cd "$(dirname "$0")/.." && pwd)
LIB=$ROOT/lib

echo "============================"
echo "1. 系统信息"
echo "============================"
uname -a
echo

echo "============================"
echo "2. 程序架构"
echo "============================"
file $BIN
echo

echo "============================"
echo "3. 程序依赖库 (ldd)"
echo "============================"
ldd $BIN
echo

echo "============================"
echo "4. ELF NEEDED"
echo "============================"
readelf -d $BIN | grep NEEDED
echo

echo "============================"
echo "5. 库架构检查"
echo "============================"
file $LIB/*.so
echo

echo "============================"
echo "6. 每个库的依赖"
echo "============================"
for f in $LIB/*.so
do
    echo "----- $f -----"
    ldd $f
    echo
done

echo "============================"
echo "7. ELF Header 检查"
echo "============================"
for f in $LIB/*.so
do
    echo "----- $f -----"
    readelf -h $f | head
    echo
done

echo "============================"
echo "8. 动态库加载过程"
echo "============================"
LD_DEBUG=libs $BIN 2>&1 | tail -n 200
echo

echo "============================"
echo "9. GDB backtrace"
echo "============================"
gdb -batch -ex run -ex bt $BIN
echo

echo "============================"
echo "诊断完成"
echo "============================"
