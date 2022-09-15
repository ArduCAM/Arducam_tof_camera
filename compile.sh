# compile
cd `find ~ -name Arducam_tof_camera`
cd `find -name example`

mkdir build && cd build
cmake ..
make 
./cpp/preview_depth