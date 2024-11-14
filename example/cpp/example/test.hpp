#pragma once

#include "ArducamTOFCamera.hpp"
#include "pid.h"
#include <functional>
#include <mutex>
#include <opencv2/core.hpp>
#include <vector>

struct opt_data {
    bool raw = false;
    bool no_preview = false;
    bool no_confidence = false;
    bool no_amplitude = false;
    bool no_load_cali = false;
    int device = 0;
    int mode = -2;
    const char* cfg = nullptr;
    int fps = -1;
    int min_range = 0;
    int max_range = 0;
    int confidence_value = 30;
    int exp_time = 0;
    bool h_flip = false, v_flip = false;

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

    std::mutex process_mtx;
    std::vector<std::function<void(Arducam::ArducamTOFCamera&)>> process;
};

bool parse_opt(int argc, char* argv[], opt_data& opt);
