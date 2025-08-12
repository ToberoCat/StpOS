#!/bin/bash

# Do not run this script directly. It is called from scripts/compile-wheel.sh
set -e

LIBWALLABY=false
DEBUG_Building=""

for arg in "$@"
do
    case $arg in
        --libwallaby)
        LIBWALLABY=true
        shift
        ;;
        --debug)
        DEBUG_Building="-DCMAKE_BUILD_TYPE=Debug"
        export DEBUG=1
        shift
        ;;
    esac
done

export CMAKE_BUILD_PARALLEL_LEVEL=8

# Removed CREATE3-related logic

if $LIBWALLABY ; then
  echo "Building libwallaby"
  cd /app/lib/libwallaby || exit 1
  cmake -Bbuild -Dwith_tello=OFF -Dwith_python_binding=OFF -Dcreate3_build_local=OFF $DEBUG_Building .
  cd build || exit 1
  ninja
  cpack
  mv kipr-1.2.0-Linux.deb /app/binaries
fi

dpkg -i /app/binaries/kipr-1.2.0-Linux.deb

echo "Building python library"
cd /app || exit 1
python3 setup.py bdist_wheel
