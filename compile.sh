# compile
cd `find ~ -name Arducam_tof_camera`
cd example
mkdir build
cd build
cmake ..
sudo make install
./example_cpp/test_cpp