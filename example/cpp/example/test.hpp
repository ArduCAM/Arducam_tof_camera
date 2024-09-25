#pragma once

#include "pid.h"
#include <opencv2/core.hpp>

struct opt_data {
    bool raw = false;
    bool no_preview = false;
    bool no_confidence = false;
    bool no_amplitude = false;
    int device = 0;
    int mode = -1;
    const char* cfg = nullptr;
    int fps = -1;
    int min_range = 0;
    int max_range = 0;
    int confidence_value = 30;

    cv::Rect seletRect{0, 0, 0, 0};
    cv::Rect followRect{0, 0, 0, 0};
    int sel_range = 4;
    int max_width = 240;
    int max_height = 180;
    double gain = 0;
    double gamma = 1;
    cv::Mat gamma_lut;
    double gain_val = 1, gain_offset_val = 0;
    pid_ref gain_pid, gain_offset_pid;
};

bool parse_opt(int argc, char* argv[], opt_data& opt);
