
# compile
cd `find ~ -name Arducam_tof_camera`
cd jetson

mkdir build && cd build
cmake ..
make 
./jetson_preview 