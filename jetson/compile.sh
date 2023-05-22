#!/bin/sh
# compile script
# echo $(dirname $(pwd))
workpath=${PWD##*/}
execfunc=./jetson_preview

if [ ! "$workpath" = "jetson" ]; then
  echo "The compiled script is moved and cannot be executed normally!"
else
   
  if [ ! -d "build" ]; then
    mkdir build
  fi

  cd build || exit
  cmake .. && make
  if [ ! -x "$execfunc" ];then
    echo "Compilation error, please check whether the driver and sdk are installed successfully!"
  else
    $execfunc
  fi
fi
