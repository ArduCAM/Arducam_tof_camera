cmake_minimum_required(VERSION 3.4)

find_package (Python3 COMPONENTS Interpreter Development)

set(PYTHON_LIB_PATH lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages)
message("${PYTHON_LIB_PATH},${Python3_LIBRARY_DIRS}")

INSTALL(FILES ArduCamDepthCamera_Py.cpython-39-arm-linux-gnueabihf.so DESTINATION ${PYTHON_LIB_PATH})

