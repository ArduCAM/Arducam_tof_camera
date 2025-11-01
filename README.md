# Arducam TOF Camera Application

This repository contains a modernized C++ application for interacting with the Arducam TOF camera. It provides a simple, object-oriented interface for capturing and visualizing depth and amplitude data from the camera.

## Project Structure

The repository is organized into the following directories:

- `apps/`: Contains the main application entry point.
- `include/`: Contains the header files for the application's classes.
- `src/`: Contains the source files for the application's classes.
- `scripts/`: Contains build scripts.

## Getting Started

### Prerequisites

- A compatible Arducam TOF camera
- A C++ compiler that supports C++11
- CMake 3.10 or higher
- OpenCV 4.0 or higher
- The Arducam TOF SDK

### Building the Application

1. **Clone the repository:**

   ```bash
   git clone https://github.com/your-username/arducam-tof-camera-app.git
   cd arducam-tof-camera-app
   ```

2. **Run the compilation script:**

   ```bash
   ./scripts/compile.sh
   ```

### Running the Application

The compiled executable will be located in the `build/` directory. To run the application, execute the following command:

```bash
./build/tof_camera_app
```

## Usage

The application will open a window and display the live feed from the TOF camera. The window will show either the depth or amplitude data, depending on the configuration. To exit the application, press the 'q' key.
