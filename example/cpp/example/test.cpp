#include "test.hpp"

#include "ArducamTOFCamera.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

#define LOCAL       static inline
#define WITH_VT_100 1

#if WITH_VT_100
#define ESC(x) "\033" x
#define NL     ESC("[K\n")
#else
#define ESC(x) ""
#define NL     "\n"
#endif

using namespace Arducam;
using namespace std::literals;

#define MAX_DISTANCE     4
#define SHOW_FRAME_DELAY 1
#define AVG_FPS          1

enum class DType {
    f32,
    u8,
    u16,
};

LOCAL cv::Mat gamma_table(float gamma);

#define DO_LATER(opt, expr)                                                                                            \
    do {                                                                                                               \
        std::unique_lock<std::mutex> lk(opt.process_mtx);                                                              \
        opt.process.push_back([=](Arducam::ArducamTOFCamera& tof) { expr; });                                          \
    } while (false)

void on_confidence_changed(int pos, void* userdata)
{
    opt_data& opt = *(opt_data*)userdata;
    opt.confidence_value = pos;
}
void on_amplitude_changed(int pos, void* userdata)
{
    opt_data& opt = *(opt_data*)userdata;
    opt.amplitude_value = opt_data::amplitude_value_step(pos);
}
void on_exposure_changed(int pos, void* userdata)
{
    opt_data& opt = *(opt_data*)userdata;
    opt.exp_time = pos;
    DO_LATER(opt, tof.setControl(Control::EXPOSURE, pos));
}
void on_hflip_changed(int pos, void* userdata)
{
    opt_data& opt = *(opt_data*)userdata;
    opt.h_flip = pos != 0;
    DO_LATER(opt, tof.setControl(Control::HFLIP, pos));
}
void on_vflip_changed(int pos, void* userdata)
{
    opt_data& opt = *(opt_data*)userdata;
    opt.v_flip = pos != 0;
    DO_LATER(opt, tof.setControl(Control::VFLIP, pos));
}
void on_min_range_changed(int pos, void* userdata)
{
    opt_data& opt = *(opt_data*)userdata;
    opt.min_range = pos;
    if (opt.min_range > opt.max_range) {
        opt.max_range = opt.min_range;
        cv::setTrackbarPos("max-range", "preview", opt.max_range);
    }
}
void on_max_range_changed(int pos, void* userdata)
{
    opt_data& opt = *(opt_data*)userdata;
    opt.max_range = pos;
    if (opt.max_range < opt.min_range) {
        opt.min_range = opt.max_range;
        cv::setTrackbarPos("min-range", "preview", opt.min_range);
    }
}
void on_gain_changed(int pos, void* userdata)
{
    opt_data& opt = *(opt_data*)userdata;
    // if (pos == 0) {
    //     opt.gain_val = opt.gain;
    // }
    opt.gain = pos / 10.0;
}
void on_gamma_changed(int pos, void* userdata)
{
    opt_data& opt = *(opt_data*)userdata;
    auto tmp = pos / 10.0;
    if (tmp != opt.gamma) {
        opt.gamma = tmp;
        opt.gamma_lut = gamma_table((float)opt.gamma);
    }
}

void onMouse(int event, int x, int y, int flags, void* userdata)
{
    (void)flags;

    opt_data& opt = *(opt_data*)userdata;
    const auto r = opt.sel_range;
    if (x < r || x > (opt.max_width - r) || y < r || y > (opt.max_height - r))
        return;
    switch (event) {
    case cv::EVENT_LBUTTONDOWN:
        break;

    case cv::EVENT_LBUTTONUP:
        opt.seletRect.x = x - r;
        opt.seletRect.y = y - r;
        opt.seletRect.width = 2 * r;
        opt.seletRect.height = 2 * r;
        break;
    default:
        opt.followRect.x = x - r;
        opt.followRect.y = y - r;
        opt.followRect.width = 2 * r;
        opt.followRect.height = 2 * r;
        break;
    }
}

LOCAL const char* to_str(DType type)
{
    switch (type) {
    case DType::f32:
        return "f32";
    case DType::u8:
        return "u8";
    case DType::u16:
        return "u16";
    default:
        return "unknown";
    }
}

LOCAL int to_size(DType type)
{
    switch (type) {
    case DType::f32:
        return sizeof(float);
    case DType::u8:
        return sizeof(uint8_t);
    case DType::u16:
        return sizeof(uint16_t);
    default:
        return 0;
    }
}

#if AVG_FPS
LOCAL void display_fps(void)
{
    using std::chrono::high_resolution_clock;
    static auto start = high_resolution_clock::now();
    static float avg_duration = 0;
    static float alpha = 1.F / 10;
    static int count = 0;

    auto now = high_resolution_clock::now();
    auto cost_ms = (now - start) / 1ms;
    start = now;

    if (count == 0) {
        ++count;
        return;
    } else if (count == 1) {
        avg_duration = (float)cost_ms;
    } else {
        float t = avg_duration * (1 - alpha) + cost_ms * alpha;
        if (std::isfinite(t)) {
            avg_duration = t;
            alpha = 1.F / (2000.F / t);
            alpha = std::min(1.F / 30, std::max(1.F / 1, alpha));
        }
    }

    ++count;

    std::cout << "fps:" << (1000. / avg_duration) << NL;
}
#else
LOCAL void display_fps(void)
{
    using std::chrono::high_resolution_clock;
    using namespace std::literals;
    static int count = 0;
    static auto time_beg = high_resolution_clock::now();
    auto time_end = high_resolution_clock::now();
    ++count;
    auto duration_ms = (time_end - time_beg) / 1ms;
    if (duration_ms >= 1000) {
        std::cout << "fps:" << count << NL;
        count = 0;
        time_beg = time_end;
    }
}
#endif

#if 0
LOCAL void getPreview(cv::Mat preview_ptr, cv::Mat confidence_image_ptr, int confidence_value)
{
    preview_ptr.setTo(255, confidence_image_ptr < confidence_value);
}
LOCAL void getPreviewRGB(cv::Mat preview_ptr, cv::Mat confidence_image_ptr, int confidence_value)
{
    preview_ptr.setTo(cv::Scalar(0, 0, 0), confidence_image_ptr < confidence_value);
    // cv::GaussianBlur(preview_ptr, preview_ptr, cv::Size(7, 7), 0);
}
#endif

LOCAL std::string saveData(void* data, unsigned int width, unsigned int height, const char* prefix = "depth",
                           DType d_type = DType::f32)
{
    using std::to_string;
    using std::chrono::system_clock;
    using namespace std::literals;
    // get filename
    // "depth_$width_$height_f32_$timestamp.raw"
    std::string filename = prefix + "_"s + to_string(width) + "_" + to_string(height) + "_" + to_str(d_type) + "_" +
                           to_string(system_clock::now().time_since_epoch() / 1ms) + ".raw";

    // save data
    auto file = std::ofstream(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "save data failed" << NL;
        return "";
    }
    file.write(reinterpret_cast<char*>(data), width * height * to_size(d_type));
    file.close();

    std::cout << "save data to " << filename << NL;
    return filename;
}

LOCAL std::string appendData(void* data, unsigned int width, unsigned int height, const std::string& filename,
                             DType d_type = DType::f32)
{
    using std::to_string;
    using std::chrono::system_clock;
    using namespace std::literals;
    // save data
    auto file = std::ofstream(filename, std::ios::app | std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "append data failed" << NL;
        return "";
    }
    file.write(reinterpret_cast<char*>(data), width * height * to_size(d_type));
    file.close();

    std::cout << "append data to " << filename << NL;
    return filename;
}

LOCAL bool checkExit(const opt_data& opt)
{
    (void)opt;

    int key = cv::waitKey(500);
    switch (key) {
    case 27:
    case 'q':
        return false;
    }
    return true;
}

LOCAL bool processKey(const opt_data& opt, void* data, void* data2 = nullptr, void* data3 = nullptr)
{
    static std::string last_filename;
    static int need_save_cnt = 0, need_save_input = 0;
    int key = 0;
    if (opt.fps <= 0) {
        key = cv::waitKey(1);
    } else {
        key = cv::waitKey(500 / opt.fps);
    }
    if (key == -1 && need_save_cnt > 0) {
        key = 'r';
    }
    switch (key) {
    case 27:
    case 'q':
        return false;
    case 's':
        if (data2 == nullptr) {
            last_filename = saveData(data, opt.max_width, opt.max_height, "raw", DType::u16);
        } else {
            saveData(data, opt.max_width, opt.max_height, "depth", DType::f32);
            saveData(data2, opt.max_width, opt.max_height, "confidence", DType::f32);
            saveData(data3, opt.max_width, opt.max_height, "gray", DType::f32);
        }
        break;
    case 'r':
        if (need_save_input != 0) {
            need_save_cnt = need_save_input;
            need_save_input = 0;
        }
        if (data2 == nullptr) {
            if (last_filename.empty()) {
                last_filename = saveData(data, opt.max_width, opt.max_height, "raw", DType::u16);
            } else {
                last_filename = appendData(data, opt.max_width, opt.max_height, last_filename, DType::u16);
            }
            need_save_cnt--;
            if (need_save_cnt == 0) {
                std::cout << "save done to " << last_filename << NL;
                last_filename.clear();
            }
        }
        break;
    case 'c':
        need_save_input = 0;
        std::cout << "input: " << need_save_input << NL;

        break;
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9': {
        need_save_input *= 10;
        need_save_input += key - '0';
        std::cout << "input: " << need_save_input << NL;
    }
    default:
        break;
    }
    return true;
}

LOCAL bool raw_loop(Arducam::ArducamTOFCamera& tof, opt_data& data)
{
    Arducam::ArducamFrameBuffer* frame = tof.requestFrame(2000);
    if (frame == nullptr) {
        return checkExit(data);
    }
    Arducam::FrameFormat format;
    frame->getFormat(FrameType::RAW_FRAME, format);
    std::cout << "frame: (" << format.width << "x" << format.height << "), time: " << format.timestamp << NL;
    data.max_height = format.height;
    data.max_width = format.width;
#if SHOW_FRAME_DELAY
    if (1000000000000UL <= format.timestamp && format.timestamp <= 9000000000000UL) {
        // a timestamp in milliseconds (13 digits)
        uint64_t now = std::chrono::system_clock::now().time_since_epoch() / 1ms;
        std::cout << "timestamp: " << format.timestamp << ", now: " << now << ", diff: " << (now - format.timestamp)
                  << "ms" << NL;
    } else if (1000000000UL <= format.timestamp && format.timestamp <= 9000000000UL) {
        // a timestamp in seconds (10 digits)
        uint64_t now = std::chrono::system_clock::now().time_since_epoch() / 1s;
        std::cout << "timestamp: " << format.timestamp << ", now: " << now << ", diff: " << (now - format.timestamp)
                  << "s" << NL;
    } else {
        // invalid timestamp from epoch
        // with mono timestamp
        uint64_t now = std::chrono::steady_clock::now().time_since_epoch() / 1ms;
        std::cout << "timestamp: " << format.timestamp << ", now: " << now << ", diff: " << (now - format.timestamp)
                  << "ms" << NL;
    }
#endif

    // depth_ptr = (float*)frame->getData(FrameType::DEPTH_FRAME);
    uint16_t* raw_data = (uint16_t*)frame->getData(FrameType::RAW_FRAME);
    if (raw_data == nullptr) {
        tof.releaseFrame(frame);
        return checkExit(data);
    }

    cv::Mat result_frame(format.height, format.width, CV_8U);
    cv::Mat raw_frame(format.height, format.width, CV_16U, raw_data);

    raw_frame.convertTo(result_frame, CV_8U, 1. / (1 << 4), 0);
    if (!data.no_preview) {
        cv::imshow("preview", result_frame);
    }
    display_fps();
    if (!processKey(data, raw_data)) {
        return false;
    }
    tof.releaseFrame(frame);
    return true;
}

LOCAL bool depth_loop(Arducam::ArducamTOFCamera& tof, opt_data& data)
{
    // auto info = tof.getCameraInfo();
    Arducam::ArducamFrameBuffer* frame = tof.requestFrame(2000);
    if (frame == nullptr) {
        return checkExit(data);
    }
    Arducam::FrameFormat format;
    frame->getFormat(FrameType::DEPTH_FRAME, format);
    std::cout << "frame: (" << format.width << "x" << format.height << "), time: " << format.timestamp << "  " << NL;
    data.max_height = format.height;
    data.max_width = format.width;
    const int min_range = data.min_range, max_range = data.max_range;
#if SHOW_FRAME_DELAY
    if (1000000000000UL <= format.timestamp && format.timestamp <= 9000000000000UL) {
        // a timestamp in milliseconds (13 digits)
        uint64_t now = std::chrono::system_clock::now().time_since_epoch() / 1ms;
        std::cout << "timestamp: " << format.timestamp << ", now: " << now << ", diff: " << (now - format.timestamp)
                  << "ms" << NL;
    } else if (1000000000UL <= format.timestamp && format.timestamp <= 9000000000UL) {
        // a timestamp in seconds (10 digits)
        uint64_t now = std::chrono::system_clock::now().time_since_epoch() / 1s;
        std::cout << "timestamp: " << format.timestamp << ", now: " << now << ", diff: " << (now - format.timestamp)
                  << "s" << NL;
    } else {
        // invalid timestamp from epoch
        // with mono timestamp
        uint64_t now = std::chrono::steady_clock::now().time_since_epoch() / 1ms;
        std::cout << "timestamp: " << format.timestamp << ", now: " << now << ", diff: " << (now - format.timestamp)
                  << "ms" << NL;
    }
#endif
    float* depth_ptr = (float*)frame->getData(FrameType::DEPTH_FRAME);
    float* amplitude_ptr = (float*)frame->getData(FrameType::AMPLITUDE_FRAME);
    float* confidence_ptr = (float*)frame->getData(FrameType::CONFIDENCE_FRAME);

    cv::Mat result_frame(format.height, format.width, CV_8U);
    cv::Mat depth_frame(format.height, format.width, CV_32F, depth_ptr);
    cv::Mat amplitude_frame_ori(format.height, format.width, CV_32F, amplitude_ptr);
    cv::Mat amplitude_frame(format.height, format.width, CV_8U);
    cv::Mat confidence_frame_ori(format.height, format.width, CV_32F, confidence_ptr);
    cv::Mat confidence_frame(format.height, format.width, CV_32F);

    depth_frame.convertTo(result_frame, CV_8U, 255.0 / (max_range - min_range),
                          (-min_range * 255.0) / (max_range - min_range));
    // getPreview(result_frame, confidence_frame);
    cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_RAINBOW);
    result_frame.setTo(cv::Scalar(0, 0, 0), (depth_frame < min_range) | (depth_frame > max_range));
    result_frame.setTo(cv::Scalar(0, 0, 0),
                       (confidence_frame_ori < data.confidence_value) | (amplitude_frame_ori < data.amplitude_value));

    confidence_frame_ori.convertTo(confidence_frame, CV_32F, 1 / 255.0, 0);
    // cv::normalize(amplitude_frame_ori, amplitude_frame, 255, 0, cv::NORM_MINMAX, CV_8U);
    if (data.gain == 0) {
        double min, max;
        // cv::minMaxLoc(amplitude_frame_ori, &min, &max);
        std::vector<float> tmp;
        amplitude_frame_ori.reshape(1, 1).copyTo(tmp);
        auto min_it = tmp.begin() + tmp.size() * 0.02;
        std::nth_element(tmp.begin(), min_it, tmp.end());
        min = *min_it;
        auto max_it = tmp.begin() + tmp.size() * 0.98;
        std::nth_element(tmp.begin(), max_it, tmp.end());
        max = *max_it;
        // printf("= min: %f, max: %f\n", min, max);
        constexpr double gain_min = .05 / (1 << (12 - 8));
        constexpr double gain_max = 80. / (1 << (12 - 8));
        double exp_gain = 255.0 / (max - min);
        exp_gain = std::min(std::max(exp_gain, gain_min), gain_max);
        double exp_offset = -min * exp_gain;
        double inc_gain = pid_calculate(data.gain_pid, exp_gain, data.gain_val);
        double inc_offset = pid_calculate(data.gain_offset_pid, exp_offset, data.gain_offset_val);
        data.gain_val = data.gain_val + inc_gain;
        data.gain_offset_val = data.gain_offset_val + inc_offset;
        // printf("exp: (x%f+%f), inc: (%f, %f), real: (x%f+%f)\n", exp_gain * (1 << (12 - 8)), exp_offset, inc_gain,
        //        inc_offset, data.gain_val * (1 << (12 - 8)), data.gain_offset_val);
        amplitude_frame_ori.convertTo(amplitude_frame, CV_8U, data.gain_val, data.gain_offset_val);
    } else {
        amplitude_frame_ori.convertTo(amplitude_frame, CV_8U, data.gain / (1 << (12 - 8)), 0);
    }
    if (data.gamma != 1) {
        cv::LUT(amplitude_frame, data.gamma_lut, amplitude_frame);
    }

    cv::rectangle(result_frame, data.seletRect, cv::Scalar(0, 0, 0), 2);
    cv::rectangle(result_frame, data.followRect, cv::Scalar(255, 255, 255), 1);

    if (!data.no_preview) {
        cv::imshow("preview", result_frame);
        std::cout << "select Rect distance: " << cv::mean(depth_frame(data.seletRect)).val[0]
                  << "mm, pos: " << data.seletRect << NL;
    }
    if (!data.no_confidence) {
        cv::imshow("confidence", confidence_frame);
    }
    if (!data.no_amplitude) {
        cv::imshow("amplitude", amplitude_frame);
    }

    display_fps();
    if (!processKey(data, depth_ptr, confidence_ptr, amplitude_ptr)) {
        return false;
    }
    tof.releaseFrame(frame);
    return true;
}
LOCAL void process_later(Arducam::ArducamTOFCamera& tof, opt_data& data)
{
    std::unique_lock<std::mutex> lk(data.process_mtx);
    for (auto& proc : data.process) {
        proc(tof);
    }
    data.process.clear();
}

#define check(expr, err_msg)                                                                                           \
    do {                                                                                                               \
        Arducam::TofErrorCode ret = (expr);                                                                            \
        if (ret != Arducam::TofErrorCode::ArducamSuccess) {                                                            \
            std::cerr << err_msg << ": " << to_str(ret) << std::endl;                                                  \
            return -1;                                                                                                 \
        }                                                                                                              \
    } while (0)
#define check_nonret(expr, err_msg)                                                                                    \
    do {                                                                                                               \
        Arducam::TofErrorCode ret = (expr);                                                                            \
        if (ret != Arducam::TofErrorCode::ArducamSuccess) {                                                            \
            std::cerr << err_msg << ": " << to_str(ret) << std::endl;                                                  \
        }                                                                                                              \
    } while (0)

#define set_ctl_(tof, ctrl, val)                                                                                       \
    check_nonret(tof.setControl(Control::ctrl, val), "set control(" #ctrl ", " #val ") failed")
#define set_ctl(ctrl, val) set_ctl_(tof, ctrl, val)

int main(int argc, char* argv[])
{
    opt_data opt;

    if (!parse_opt(argc, argv, opt)) {
        return -1;
    }

    opt.gamma_lut = gamma_table((float)opt.gamma);
    if (!pid_init(&opt.gain_pid, 1. / 4, 1, -1, 0.4, 0.01, 0) ||
        !pid_init(&opt.gain_offset_pid, 1. / 4, 5, -5, 0.4, 0.01, 0.3)) {
        std::cerr << "pid init failed" << std::endl;
        return -1;
    }

    Arducam::ArducamTOFCamera tof;
    // if (tof.openWithFile(Arducam::DEPTH_FRAME, "imx316_mcu.cfg", 375e5))
    if (opt.cfg) {
        check(tof.openWithFile(opt.cfg, opt.device), "open camera failed");
    } else {
        check(tof.open(Connection::CSI, opt.device), "open camera failed");
    }

    int option = opt.mode;
    auto info = tof.getCameraInfo();
    if (option == -2) {
        // hqvga device does not support the other resolutions
        option = info.device_type == Arducam::DeviceType::DEVICE_VGA ? 1 : -1;
    }

    switch (option) {
    case -1:
        std::cout << "keep default\n";
        break;
    case 0:
        std::cout << "640 * 480 with single frequency\n";
        break;
    case 1:
        std::cout << "640 * 480 with dual frequency\n";
        break;
    default:
        return -1;
    }

    if (option == -1) {
        // do nothing
    } else {
        if (option & 1) {
            set_ctl(MODE, to_int(TofWorkMode::DOUBLE_FREQ));
        } else {
            set_ctl(MODE, to_int(TofWorkMode::SINGLE_FREQ));
        }
    }

    if (opt.no_load_cali) {
        set_ctl(LOAD_CALI_DATA, 0);
    }

    set_ctl(HFLIP, opt.h_flip);
    set_ctl(VFLIP, opt.v_flip);

    if (opt.raw) {
        check(tof.start(FrameType::RAW_FRAME), "start camera failed");
    } else {
        check(tof.start(FrameType::DEPTH_FRAME), "start camera failed");
    }

    if (opt.fps != -1) {
        if (opt.fps != 0) {
            int max_fps = 0;
            tof.getControl(Control::FRAME_RATE, &max_fps);
            if (opt.fps > max_fps) {
                std::cerr << "Invalid fps: " << opt.fps << ", max fps: " << max_fps << std::endl;
                return -1;
            }
            set_ctl(FRAME_RATE, opt.fps);
        }
        // disable auto frame rate if the camera has the control
        (void)tof.setControl(Control::AUTO_FRAME_RATE, 0);
    }
    tof.getControl(Control::FRAME_RATE, &opt.fps);

    if (opt.exp_time == 0) {
        tof.getControl(Control::EXPOSURE, &opt.exp_time);
    } else {
        set_ctl(EXPOSURE, opt.exp_time);
    }

    // int rang = 0;
    // tof.setControl(Control::RANGE, MAX_DISTANCE);
    // tof.getControl(Control::RANGE, &rang);
    // std::cout << rang << std::endl;

    info = tof.getCameraInfo();
    int max_range;
    tof.getControl(Control::RANGE, &max_range);
    std::cout << "open camera with (" << info.width << "x" << info.height << ") with range " << max_range << std::endl;
    // tof.writeSensor(0x0402, 0x01301e00);
    tof.writeSensor(0x0402, 0x01300a00);

    if (opt.max_range == 0) {
        opt.max_range = max_range;
    }

    if (!opt.no_preview) {
        cv::namedWindow("preview", cv::WINDOW_NORMAL);
        cv::setMouseCallback("preview", onMouse, &opt);

        cv::createTrackbar("min-range", "preview", NULL, 6000, on_min_range_changed, &opt);
        cv::createTrackbar("max-range", "preview", NULL, 6000, on_max_range_changed, &opt);
        cv::createTrackbar("exposure", "preview", NULL, 0xffff, on_exposure_changed, &opt);
        // cv::createTrackbar("hflip", "preview", NULL, 1, on_hflip_changed, &opt);
        // cv::createTrackbar("vflip", "preview", NULL, 1, on_vflip_changed, &opt);

        cv::setTrackbarPos("min-range", "preview", opt.min_range);
        cv::setTrackbarPos("max-range", "preview", opt.max_range);
        cv::setTrackbarPos("exposure", "preview", opt.exp_time);
        // cv::setTrackbarPos("hflip", "preview", opt.h_flip);
        // cv::setTrackbarPos("vflip", "preview", opt.v_flip);

        if (info.device_type == Arducam::DeviceType::DEVICE_VGA) {
            // only vga support confidence
            cv::createTrackbar("confidence", "preview", NULL, 255, on_confidence_changed, &opt);
            cv::setTrackbarPos("confidence", "preview", opt.confidence_value);
            cv::createTrackbar("amplitude", "preview", NULL, opt_data::amplitude_value_range, on_amplitude_changed,
                               &opt);
            cv::setTrackbarPos("amplitude", "preview", 0);
        }
    }

    std::cout << ESC("[?25l");                     // hide cursor
    std::cout << ESC("7");                         // save cursor position
    std::cout << NL << NL << NL << NL << NL << NL; // move cursor down 6 lines
    std::cout << ESC("8");                         // restore cursor position
    if (opt.raw) {
        for (; raw_loop(tof, opt);) {
            process_later(tof, opt);
            std::cout << ESC("8"); // restore cursor position
        }
    } else {
        if (!opt.no_confidence) {
            cv::namedWindow("confidence", cv::WINDOW_NORMAL);
        }
        if (!opt.no_amplitude) {
            cv::namedWindow("amplitude", cv::WINDOW_NORMAL);

            cv::createTrackbar("gain x10", "amplitude", NULL, static_cast<int>(80.0 * 10), on_gain_changed, &opt);
            cv::createTrackbar("gamma x10", "amplitude", NULL, static_cast<int>(2.5 * 10), on_gamma_changed, &opt);
            cv::setTrackbarPos("gain x10", "amplitude", static_cast<int>(opt.gain * 10));
            cv::setTrackbarPos("gamma x10", "amplitude", static_cast<int>(opt.gamma * 10));
        }
        for (; depth_loop(tof, opt);) {
            process_later(tof, opt);
            std::cout << ESC("8"); // restore cursor position
        }
    }
    std::cout << ESC("[?25h");     // show cursor
    std::cout << ESC("[6B") << NL; // move cursor down 6 lines

    if (tof.stop()) {
        std::cerr << "stop camera failed\n";
    }
    if (tof.close()) {
        std::cerr << "close camera failed\n";
    }

    pid_destroy(opt.gain_pid);
    pid_destroy(opt.gain_offset_pid);
    return 0;
}

LOCAL cv::Mat gamma_table(float gamma)
{
    cv::Mat lut(1, 256, CV_8U);
    float inv_gamma = 1.F / gamma;
    for (int i = 0; i < 256; i++) {
        lut.at<uint8_t>(i) = cv::saturate_cast<uint8_t>(pow(i / 255.0, inv_gamma) * 255.0);
    }
    return lut;
}
