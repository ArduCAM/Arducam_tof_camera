# compile
cd `find ~ -name Arducam_tof_camera`
mkdir build && cd build
cmake ..
make 
./example/cpp/preview_depth