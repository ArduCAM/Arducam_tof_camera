#!/bin/sh
# compile script
workpath=${PWD##*/}
execfunc=cpp/preview_depth
if [ ! "$workpath" = "Arducam_tof_camera" ] && [ ! "$workpath" = "arducam_tof_camera" ] ; then
  echo "The compiled script is moved and cannot be executed normally!"
else
  if [ ! -d "example" ]; then
    echo "source code file does not exist!"
  else
    cd example || exit

    if [ ! -d "build" ]; then
      mkdir build
    fi

    cd build || exit
    cmake .. && make
    if [ ! -x "$execfunc" ]; then
      echo "Compilation error, please check whether the opencvand arducam-tof-sdk are installed successfully!"
    else
      $execfunc
    fi
  fi
fi