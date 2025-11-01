#include "TOFApplication.h"
#include <cstdlib>

/**
 * @brief Main entry point for the TOF camera application.
 *
 * This function initializes and runs the TOFApplication, which encapsulates the
 * main logic for camera interaction and data visualization.
 *
 * @return int Returns EXIT_SUCCESS on successful execution, EXIT_FAILURE otherwise.
 */
int main() {
    TOFApplication app;
    if (app.initialize() != 0) {
        return EXIT_FAILURE;
    }
    app.run();
    return EXIT_SUCCESS;
}
