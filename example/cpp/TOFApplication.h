#ifndef TOFAPPLICATION_H
#define TOFAPPLICATION_H

#include "Camera.h"
#include "Viewer.h"
#include <memory>

class TOFApplication {
public:
    TOFApplication();
    ~TOFApplication();

    int initialize();
    void run();

private:
    std::unique_ptr<Camera> camera_;
    std::unique_ptr<Viewer> viewer_;
};

#endif // TOFAPPLICATION_H
