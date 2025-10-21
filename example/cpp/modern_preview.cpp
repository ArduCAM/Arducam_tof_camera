#include "TOFApplication.h"
#include <cstdlib>

int main() {
    TOFApplication app;
    if (app.initialize() != 0) {
        return EXIT_FAILURE;
    }
    app.run();
    return EXIT_SUCCESS;
}
