#include "ArducamTOFCamera.hpp"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

#define LOCAL static inline

using namespace Arducam;
using namespace std::literals;

#define MAX_DISTANCE 4
#define AVG_FPS      1

cv::Rect seletRect(0, 0, 0, 0);
cv::Rect followRect(0, 0, 0, 0);
int sel_range = 4;
int max_width = 240;
int max_height = 180;
int confidence_value = 30;

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
};

enum class DType {
    f32,
    u8,
    u16,
};

void on_confidence_changed(int pos, void* userdata)
{
    //
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
    static float alpha = 1. / 10;
    static int count = 0;

    auto now = high_resolution_clock::now();
    auto cost_ms = (now - start) / 1ms;
    start = now;

    if (count == 0) {
        ++count;
        return;
    } else if (count == 1) {
        avg_duration = cost_ms;
    } else {
        float t = avg_duration * (1 - alpha) + cost_ms * alpha;
        if (std::isfinite(t)) {
            avg_duration = t;
            alpha = 1. / (2000. / t);
            alpha = std::min(1.F / 30, std::max(1.F / 1, alpha));
        }
    }

    ++count;

    std::cout << "fps:" << (1000. / avg_duration) << std::endl;
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
        std::cout << "fps:" << count << std::endl;
        count = 0;
        time_beg = time_end;
    }
}
#endif

LOCAL void getPreview(uint8_t* preview_ptr, float* phase_image_ptr, float* confidence_image_ptr)
{
    auto len = 240 * 180;
    for (int i = 0; i < len; i++) {
        uint8_t mask = *(confidence_image_ptr + i) > 30 ? 254 : 0;
        float depth = ((1 - (*(phase_image_ptr + i) / MAX_DISTANCE)) * 255);
        uint8_t pixel = depth > 255 ? 255 : depth;
        *(preview_ptr + i) = pixel & mask;
    }
}

LOCAL void getPreview(cv::Mat preview_ptr, cv::Mat confidence_image_ptr)
{
    for (int line = 0; line < preview_ptr.rows; line++) {
        for (int col = 0; col < preview_ptr.cols; col++) {
            if (confidence_image_ptr.at<float>(line, col) < confidence_value)
                preview_ptr.at<uint8_t>(line, col) = 255;
        }
    }
}

LOCAL void getPreviewRGB(cv::Mat preview_ptr, cv::Mat confidence_image_ptr)
{
    preview_ptr.setTo(cv::Scalar(0, 0, 0), confidence_image_ptr < confidence_value);
    // cv::GaussianBlur(preview_ptr, preview_ptr, cv::Size(7, 7), 0);
}

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
        std::cerr << "save data failed" << std::endl;
        return "";
    }
    file.write(reinterpret_cast<char*>(data), width * height * to_size(d_type));
    file.close();

    std::cout << "save data to " << filename << std::endl;
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
        std::cerr << "append data failed" << std::endl;
        return "";
    }
    file.write(reinterpret_cast<char*>(data), width * height * to_size(d_type));
    file.close();

    std::cout << "append data to " << filename << std::endl;
    return filename;
}

LOCAL bool checkExit()
{
    int key = cv::waitKey(500);
    switch (key) {
    case 27:
    case 'q':
        return false;
    }
    return true;
}

LOCAL bool processKey(void* data, void* data2 = nullptr)
{
    static std::string last_filename;
    int key = cv::waitKey(1);
    switch (key) {
    case 27:
    case 'q':
        return false;
    case 's':
        if (data2 == nullptr) {
            last_filename = saveData(data, max_width, max_height, "raw", DType::u16);
        } else {
            saveData(data, max_width, max_height, "depth", DType::f32);
            saveData(data2, max_width, max_height, "confidence", DType::f32);
        }
        break;
    case 'r':
        if (data2 == nullptr) {
            if (last_filename.empty()) {
                last_filename = saveData(data, max_width, max_height, "raw", DType::u16);
            } else {
                last_filename = appendData(data, max_width, max_height, last_filename, DType::u16);
            }
        }
    default:
        break;
    }
    return true;
}

void onMouse(int event, int x, int y, int flags, void* param)
{
    const auto r = sel_range;
    if (x < r || x > (max_width - r) || y < r || y > (max_height - r))
        return;
    switch (event) {
    case cv::EVENT_LBUTTONDOWN:
        break;

    case cv::EVENT_LBUTTONUP:
        seletRect.x = x - r;
        seletRect.y = y - r;
        seletRect.width = 2 * r;
        seletRect.height = 2 * r;
        break;
    default:
        followRect.x = x - r;
        followRect.y = y - r;
        followRect.width = 2 * r;
        followRect.height = 2 * r;
        break;
    }
}

LOCAL bool raw_loop(Arducam::ArducamTOFCamera& tof, const opt_data& data)
{
    Arducam::ArducamFrameBuffer* frame = tof.requestFrame(2000);
    if (frame == nullptr) {
        return checkExit();
    }
    Arducam::FrameDataFormat format;
    frame->getFrameDataFormat(FrameType::RAW_FRAME, format);
    std::cout << "frame: (" << format.width << "x" << format.height << "), time: " << format.timestamp << std::endl;
    max_height = format.height;
    max_width = format.width;

    // depth_ptr = (float*)frame->getData(FrameType::DEPTH_FRAME);
    uint16_t* raw_data = (uint16_t*)frame->getData(FrameType::RAW_FRAME);
    if (raw_data == nullptr) {
        tof.releaseFrame(frame);
        return checkExit();
    }

    cv::Mat result_frame(format.height, format.width, CV_8U);
    cv::Mat raw_frame(format.height, format.width, CV_16U, raw_data);

    raw_frame.convertTo(result_frame, CV_8U, 1. / (1 << 4), 0);
    if (!data.no_preview) {
        cv::imshow("preview", result_frame);
    }
    if (!processKey(raw_data)) {
        return false;
    }
    display_fps();
    tof.releaseFrame(frame);
    return true;
}

LOCAL bool depth_loop(Arducam::ArducamTOFCamera& tof, const opt_data& data)
{
    // auto info = tof.getCameraInfo();
    Arducam::ArducamFrameBuffer* frame = tof.requestFrame(2000);
    if (frame == nullptr) {
        return checkExit();
    }
    Arducam::FrameDataFormat format;
    frame->getFrameDataFormat(FrameType::DEPTH_FRAME, format);
    std::cout << "frame: (" << format.width << "x" << format.height << "), time: " << format.timestamp << std::endl;
    max_height = format.height;
    max_width = format.width;
    const int min_range = data.min_range, max_range = data.max_range;
    if (1000000000000UL <= format.timestamp && format.timestamp <= 9000000000000UL) {
        // a timestamp in milliseconds (13 digits)
        uint64_t now = std::chrono::system_clock::now().time_since_epoch() / 1ms;
        printf("timestamp: %llu, now: %llu, diff: %llums\n", (long long unsigned)format.timestamp,
               (long long unsigned)now, (long long unsigned)(now - format.timestamp));
    } else if (1000000000UL <= format.timestamp && format.timestamp <= 9000000000UL) {
        // a timestamp in seconds (10 digits)
        uint64_t now = std::chrono::system_clock::now().time_since_epoch() / 1s;
        printf("timestamp: %llu, now: %llu, diff: %llus\n", (long long unsigned)format.timestamp,
               (long long unsigned)now, (long long unsigned)(now - format.timestamp));
    } else {
        // invalid timestamp from epoch
        // with mono timestamp
        uint64_t now = std::chrono::steady_clock::now().time_since_epoch() / 1ms;
        printf("timestamp: %llu, now: %llu, diff: %llums\n", (long long unsigned)format.timestamp,
               (long long unsigned)now, (long long unsigned)(now - format.timestamp));
    }
    float* depth_ptr = (float*)frame->getData(FrameType::DEPTH_FRAME);
    float* amplitude_ptr = (float*)frame->getData(FrameType::AMPLITUDE_FRAME);
    float* confidence_ptr = (float*)frame->getData(FrameType::CONFIDENCE_FRAME);

    cv::Mat result_frame(format.height, format.width, CV_8U);
    cv::Mat depth_frame(format.height, format.width, CV_32F, depth_ptr);
    cv::Mat amplitude_frame(format.height, format.width, CV_32F, amplitude_ptr);
    cv::Mat confidence_frame(format.height, format.width, CV_32F, confidence_ptr);

    depth_frame.convertTo(result_frame, CV_8U, 255.0 / (max_range - min_range),
                          (-min_range * 255.0) / (max_range - min_range));
    // getPreview(result_frame, confidence_frame);
    cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_RAINBOW);
    result_frame.setTo(cv::Scalar(0, 0, 0), depth_frame < min_range);
    getPreviewRGB(result_frame, confidence_frame);

    confidence_frame.convertTo(confidence_frame, CV_32F, 1 / 255.0, 0);
    // cv::normalize(confidence_frame, confidence_frame, 1, 0, cv::NORM_MINMAX);
    cv::normalize(amplitude_frame, amplitude_frame, 1, 0, cv::NORM_MINMAX);

    cv::rectangle(result_frame, seletRect, cv::Scalar(0, 0, 0), 2);
    cv::rectangle(result_frame, followRect, cv::Scalar(255, 255, 255), 1);

    if (!data.no_preview) {
        cv::imshow("preview", result_frame);
        std::cout << "select Rect distance: " << cv::mean(depth_frame(seletRect)).val[0] << "mm, pos: " << seletRect
                  << std::endl;
    }
    if (!data.no_confidence) {
        cv::imshow("confidence", confidence_frame);
    }
    if (!data.no_amplitude) {
        cv::imshow("amplitude", amplitude_frame);
    }

    if (!processKey(depth_ptr, confidence_ptr)) {
        return false;
    }
    display_fps();
    tof.releaseFrame(frame);
    return true;
}

#define check(expr, err_msg)                                                                                           \
    do {                                                                                                               \
        Arducam::TofErrorCode ret = (expr);                                                                            \
        if (ret != Arducam::TofErrorCode::ArducamSuccess) {                                                            \
            std::cerr << err_msg << ": " << to_str(ret) << std::endl;                                                  \
            return -1;                                                                                                 \
        }                                                                                                              \
    } while (0)

#define set_ctl_(tof, ctrl, val) check(tof.setControl(CameraCtrl::ctrl, val), "set control(" #ctrl ", " #val ") failed")
#define set_ctl(ctrl, val)       set_ctl_(tof, ctrl, val)

LOCAL bool parse_opt(int argc, char* argv[], opt_data& opt);

int main(int argc, char* argv[])
{
    opt_data opt;

    if (!parse_opt(argc, argv, opt)) {
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
    if (option == -1) {
        // hqvga device does not support the other resolutions
        option = info.device_type == Arducam::DeviceType::DEVICE_VGA ? 3 : 4;
    }

    switch (option) {
    case 0:
        printf("320 * 240 with single frequency\n");
        break;
    case 1:
        printf("320 * 240 with double frequency\n");
        break;
    case 2:
        printf("640 * 480 with single frequency\n");
        break;
    case 3:
        printf("640 * 480 with double frequency\n");
        break;
    case 4:
        printf("keep default\n");
        break;
    default:
        return -1;
    }

    if (option == 4) {
        // do nothing
    } else {
        if (option & 2) {
            set_ctl(FMT_WIDTH, 640);
            set_ctl(FMT_HEIGHT, 480);
        } else {
            set_ctl(FMT_WIDTH, 640 / 2);
            set_ctl(FMT_HEIGHT, 480 / 2);
        }

        if (option & 1) {
            set_ctl(MODE, to_int(TofWorkMode::DOUBLE_FREQ));
        } else {
            set_ctl(MODE, to_int(TofWorkMode::SINGLE_FREQ));
        }
    }

    if (opt.raw) {
        check(tof.start(FrameType::RAW_FRAME), "start camera failed");
    } else {
        check(tof.start(FrameType::DEPTH_FRAME), "start camera failed");
    }

    if (opt.fps != -1) {
        if (opt.fps != 0) {
            int max_fps = 0;
            tof.getControl(CameraCtrl::FRAME_RATE, &max_fps);
            if (opt.fps > max_fps) {
                std::cerr << "Invalid fps: " << opt.fps << ", max fps: " << max_fps << std::endl;
                return -1;
            }
            set_ctl(FRAME_RATE, opt.fps);
        }
        // disable auto frame rate if the camera has the control
        (void)tof.setControl(CameraCtrl::AUTO_FRAME_RATE, 0);
    }

    // int rang = 0;
    // tof.setControl(CameraCtrl::RANGE, MAX_DISTANCE);
    // tof.getControl(CameraCtrl::RANGE, &rang);
    // std::cout << rang << std::endl;

    info = tof.getCameraInfo();
    int max_range;
    tof.getControl(CameraCtrl::RANGE, &max_range);
    std::cout << "open camera with (" << info.width << "x" << info.height << ") with range " << max_range << std::endl;

    if (opt.max_range == 0) {
        opt.max_range = max_range;
    }

    if (!opt.no_preview) {
        cv::namedWindow("preview", cv::WINDOW_NORMAL);
        cv::setMouseCallback("preview", onMouse);

        cv::createTrackbar("min-range", "preview", NULL, 6000, on_min_range_changed, &opt);
        cv::createTrackbar("max-range", "preview", NULL, 6000, on_max_range_changed, &opt);

        if (info.device_type == Arducam::DeviceType::DEVICE_VGA) {
            // only vga support confidence
            cv::createTrackbar("confidence", "preview", &confidence_value, 255, on_confidence_changed);
        }
    }

    if (opt.raw) {
        for (; raw_loop(tof, opt);) {
        }
    } else {
        if (!opt.no_confidence) {
            cv::namedWindow("confidence", cv::WINDOW_NORMAL);
        }
        if (!opt.no_amplitude) {
            cv::namedWindow("amplitude", cv::WINDOW_NORMAL);
        }
        for (; depth_loop(tof, opt);) {
        }
    }

    if (tof.stop()) {
        fprintf(stderr, "stop camera failed\n");
    }
    if (tof.close()) {
        fprintf(stderr, "close camera failed\n");
    }
    return 0;
}

/* #region parse_opt */

void help(int argc, char* argv[])
{
    std::cout << "Usage: " << argv[0] << " [OPTION]" << std::endl;
    std::cout << "  -h,--help           Display this information" << std::endl;
    std::cout << "  -v,--version        Display the version of the program" << std::endl;
    std::cout << "  -d,--device NUM     Set the device number" << std::endl;
    std::cout << "  --raw/--depth       Display the raw or depth frame" << std::endl;
    std::cout << "  -P,--no-preview     Do not display the preview" << std::endl;
    std::cout << "  -C,--no-confidence  Do not display the confidence" << std::endl;
    std::cout << "  -A,--no-amplitude   Do not display the amplitude" << std::endl;
    std::cout << "  --fps FPS           Set the fps of the camera" << std::endl;
    std::cout << "  --mode MODE         Set the mode of the camera" << std::endl;
    std::cout << "      0               320 * 240 with single frequency" << std::endl;
    std::cout << "      1               320 * 240 with double frequency" << std::endl;
    std::cout << "      2               640 * 480 with single frequency" << std::endl;
    std::cout << "      3(*)            640 * 480 with double frequency (default)" << std::endl;
    std::cout << "  --cfg PATH          The usb camera config file path" << std::endl;
    std::cout << "  -m,--min-range NUM  Set the min range of the camera (mm)" << std::endl;
    std::cout << "  -M,--max-range NUM  Set the max range of the camera (mm)" << std::endl;
}

enum class ArgEnum {
    none,
    help,
    version,
    device,
    raw,
    depth,
    no_preview,
    no_confidence,
    no_amplitude,
    fps,
    mode,
    cfg,
    min_range,
    max_range,
};

template <ArgEnum arg_enum> LOCAL const char* to_str()
{
    switch (arg_enum) {
    case ArgEnum::help:
        return "help";
    case ArgEnum::version:
        return "version";
    case ArgEnum::device:
        return "device";
    case ArgEnum::raw:
        return "raw";
    case ArgEnum::depth:
        return "depth";
    case ArgEnum::no_preview:
        return "no-preview";
    case ArgEnum::no_confidence:
        return "no-confidence";
    case ArgEnum::no_amplitude:
        return "no-amplitude";
    case ArgEnum::fps:
        return "fps";
    case ArgEnum::mode:
        return "mode";
    case ArgEnum::cfg:
        return "cfg";
    case ArgEnum::min_range:
        return "min_range";
    case ArgEnum::max_range:
        return "max_range";
    default:
        return "unknown";
    }
}

LOCAL bool __parse_cfg(opt_data& data, const char* path)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        return false;
    }
    data.cfg = path;
    return true;
}

template <ArgEnum arg_enum> LOCAL int __parse_opt(int argc, char* argv[], int curr, opt_data& data)
{
    std::cout << "[info] process " << to_str<arg_enum>() << std::endl;
    switch (arg_enum) {
    case ArgEnum::device: {
        if (curr + 1 >= argc) {
            std::cerr << "Invalid device number" << std::endl;
            return 0;
        }
        data.device = atoi(argv[curr + 1]);
        return 2;
    } break;
    case ArgEnum::cfg: {
        if (curr + 1 >= argc) {
            std::cerr << "Invalid config file path" << std::endl;
            return 0;
        }
        if (!__parse_cfg(data, argv[curr + 1])) {
            std::cerr << "Invalid config file path" << std::endl;
            return 0;
        }
        return 2;
    } break;
    case ArgEnum::raw: {
        data.raw = true;
    } break;
    case ArgEnum::depth: {
        data.raw = false;
    } break;
    case ArgEnum::no_preview: {
        data.no_preview = true;
    } break;
    case ArgEnum::no_confidence: {
        data.no_confidence = true;
    } break;
    case ArgEnum::no_amplitude: {
        data.no_amplitude = true;
    } break;
    case ArgEnum::min_range: {
        if (curr + 1 >= argc) {
            std::cerr << "Invalid min range" << std::endl;
            return 0;
        }
        data.min_range = atoi(argv[curr + 1]);
        if (data.min_range < 0) {
            std::cerr << "Invalid min range" << std::endl;
            return 0;
        }
        return 2;
    } break;
    case ArgEnum::max_range: {
        if (curr + 1 >= argc) {
            std::cerr << "Invalid max range" << std::endl;
            return 0;
        }
        data.max_range = atoi(argv[curr + 1]);
        if (data.max_range < 0) {
            std::cerr << "Invalid max range" << std::endl;
            return 0;
        }
        return 2;
    } break;
    case ArgEnum::mode: {
        if (curr + 1 >= argc) {
            std::cerr << "Invalid mode" << std::endl;
            return 0;
        }
        data.mode = atoi(argv[curr + 1]);
        if (data.mode < 0 || data.mode > 4) {
            std::cerr << "Invalid mode" << std::endl;
            return 0;
        }
        return 2;
    } break;
    case ArgEnum::fps: {
        if (curr + 1 >= argc) {
            std::cerr << "Invalid fps" << std::endl;
            return 0;
        }
        data.fps = atoi(argv[curr + 1]);
        if (data.fps < 0) {
            std::cerr << "Invalid fps" << std::endl;
            return 0;
        }
        return 2;
    } break;
    case ArgEnum::help: {
        help(argc, argv);
        exit(0);
    } break;
    case ArgEnum::version: {
        std::cout << "Version: 1.0.0" << std::endl;
        exit(0);
    } break;
    }
    return 1;
}

LOCAL int __parse_opt(int argc, char* argv[], int curr, opt_data& data)
{
    bool is_opt;
    const char* arg = argv[curr];
    // if start with "--"
    if (arg[0] != '-')
        return 0;
    if (arg[1] == '-') {
        is_opt = true;
        arg += 2;
    } else {
        is_opt = false;
        arg += 1;
    }

#define SHORT(chr, type)                                                                                               \
    case chr:                                                                                                          \
        if (1 != __parse_opt<type>(argc, argv, curr, data)) {                                                          \
            return 0;                                                                                                  \
        }                                                                                                              \
        break;
#define SHORT_END(chr, type)                                                                                           \
    case chr:                                                                                                          \
        if (arg[index + 1] != '\0') {                                                                                  \
            return 0;                                                                                                  \
        }                                                                                                              \
        return __parse_opt<type>(argc, argv, curr, data);                                                              \
        break;
#define LONG(chr, type)                                                                                                \
    else if (!strcmp(arg, chr))                                                                                        \
    {                                                                                                                  \
        return __parse_opt<type>(argc, argv, curr, data);                                                              \
    }

    if (!is_opt) {
        for (int index = 0; arg[index] != '\0'; index++) {
            switch (arg[index]) {
                // SHORT('r', ArgEnum::raw)
                SHORT('h', ArgEnum::help)
                SHORT('v', ArgEnum::version)
                SHORT('P', ArgEnum::no_preview)
                SHORT('C', ArgEnum::no_confidence)
                SHORT('A', ArgEnum::no_amplitude)
                SHORT_END('d', ArgEnum::device)
                SHORT_END('m', ArgEnum::min_range)
                SHORT_END('M', ArgEnum::max_range)
            default:
                return 0;
            }
        }
    } else {
        if (!strcmp(arg, "")) {
            return 0;
        }
        LONG("cfg", ArgEnum::cfg)
        LONG("raw", ArgEnum::raw)
        LONG("depth", ArgEnum::depth)
        LONG("no-preview", ArgEnum::no_preview)
        LONG("no-confidence", ArgEnum::no_confidence)
        LONG("no-amplitude", ArgEnum::no_amplitude)
        LONG("min-range", ArgEnum::min_range)
        LONG("max-range", ArgEnum::max_range)
        LONG("mode", ArgEnum::mode)
        LONG("fps", ArgEnum::fps)
        LONG("help", ArgEnum::help)
        LONG("version", ArgEnum::version)
        else
        {
            return 0;
        }
    }

#undef SHORT
#undef LONG

    return 1;
}

LOCAL bool parse_opt(int argc, char* argv[], opt_data& opt)
{
    for (int i = 1; i < argc;) {
        int tmp = __parse_opt(argc, argv, i, opt);
        if (tmp == 0) {
            std::cerr << "Invalid option: " << argv[i] << std::endl;
            return false;
        }
        i += tmp;
    }

    // check
    if (opt.no_confidence && opt.raw) {
        std::cerr << "Invalid option: --no-confidence and --raw are exclusive" << std::endl;
        return false;
    }
    if (opt.no_amplitude && opt.raw) {
        std::cerr << "Invalid option: --no-amplitude and --raw are exclusive" << std::endl;
        return false;
    }

    return true;
}

/* #endregion */
