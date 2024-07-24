#include "ArducamTOFCamera.hpp"
#include <chrono>
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
int max_width = 240;
int max_height = 180;
int max_range = 0;

enum class DType {
    f32,
    u8,
    u16,
};

int confidence_value = 30;
void on_confidence_changed(int pos, void* userdata)
{
    //
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
        avg_duration = avg_duration * (1 - alpha) + cost_ms * alpha;
        alpha = 1. / (2000. / avg_duration);
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

LOCAL void getPreview(cv::Mat preview_ptr, cv::Mat amplitude_image_ptr)
{
    auto len = preview_ptr.rows * preview_ptr.cols;
    for (int line = 0; line < preview_ptr.rows; line++) {
        for (int col = 0; col < preview_ptr.cols; col++) {
            if (amplitude_image_ptr.at<float>(line, col) < confidence_value)
                preview_ptr.at<uint8_t>(line, col) = 255;
        }
    }
}

LOCAL void getPreviewRGB(cv::Mat preview_ptr, cv::Mat amplitude_image_ptr)
{
    preview_ptr.setTo(cv::Scalar(0, 0, 0), amplitude_image_ptr < confidence_value);
    // cv::GaussianBlur(preview_ptr, preview_ptr, cv::Size(7, 7), 0);
}

LOCAL void saveData(void* data, unsigned int width, unsigned int height, const char* prefix = "depth",
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
        std::cerr << "save depth failed" << std::endl;
        return;
    }
    file.write(reinterpret_cast<char*>(data), width * height * to_size(d_type));
    file.close();

    std::cout << "save depth to " << filename << std::endl;
}

LOCAL bool checkExit()
{
    int key = cv::waitKey(1);
    switch (key) {
    case 27:
    case 'q':
        return false;
    }
    return true;
}

LOCAL bool processKey(void* data, bool depth = true)
{
    int key = cv::waitKey(1);
    switch (key) {
    case 27:
    case 'q':
        return false;
    case 's':
        if (depth) {
            saveData(data, max_width, max_height, "depth", DType::f32);
        } else {
            saveData(data, max_width, max_height, "raw", DType::u16);
        }
        break;
    default:
        break;
    }
    return true;
}

void onMouse(int event, int x, int y, int flags, void* param)
{
    if (x < 4 || x > (max_width - 4) || y < 4 || y > (max_height - 4))
        return;
    switch (event) {
    case cv::EVENT_LBUTTONDOWN:

        break;

    case cv::EVENT_LBUTTONUP:
        seletRect.x = x - 4;
        seletRect.y = y - 4;
        seletRect.width = 8;
        seletRect.height = 8;
        break;
    default:
        followRect.x = x - 4;
        followRect.y = y - 4;
        followRect.width = 8;
        followRect.height = 8;
        break;
    }
}

LOCAL bool raw_loop(Arducam::ArducamTOFCamera& tof, uint8_t* preview_ptr)
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
    cv::imshow("preview", result_frame);

    if (!processKey(raw_data, false)) {
        return false;
    }
    display_fps();
    tof.releaseFrame(frame);
    return true;
}

LOCAL bool depth_loop(Arducam::ArducamTOFCamera& tof, uint8_t* preview_ptr)
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

    float* depth_ptr = (float*)frame->getData(FrameType::DEPTH_FRAME);
    float* amplitude_ptr = (float*)frame->getData(FrameType::CONFIDENCE_FRAME);
    // getPreview(preview_ptr, depth_ptr, amplitude_ptr);

    cv::Mat result_frame(format.height, format.width, CV_8U);
    cv::Mat depth_frame(format.height, format.width, CV_32F, depth_ptr);
    cv::Mat amplitude_frame(format.height, format.width, CV_32F, amplitude_ptr);

    depth_frame.convertTo(result_frame, CV_8U, 255.0 / max_range, 0);
    // getPreview(result_frame, amplitude_frame);
    cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_RAINBOW);
    getPreviewRGB(result_frame, amplitude_frame);

    // amplitude_frame.convertTo(amplitude_frame, CV_8U, 255.0 / max_range, 0);
    cv::normalize(amplitude_frame, amplitude_frame, 1, 0, cv::NORM_MINMAX);

    // cv::imshow("result_frame", result_frame);
    cv::imshow("amplitude", amplitude_frame);

    cv::rectangle(result_frame, seletRect, cv::Scalar(0, 0, 0), 2);
    cv::rectangle(result_frame, followRect, cv::Scalar(255, 255, 255), 1);

    // std::cout << "select Rect distance: " << cv::mean(depth_frame(seletRect)).val[0] << std::endl;

    cv::imshow("preview", result_frame);
    cv::setMouseCallback("preview", onMouse);

    std::cout << "select Rect distance: " << cv::mean(depth_frame(seletRect)).val[0] << std::endl;

    if (!processKey(depth_ptr)) {
        return false;
    }
    display_fps();
    tof.releaseFrame(frame);
    return true;
}

#define check(expr, err_msg)                                                                                           \
    do {                                                                                                               \
        if (expr) {                                                                                                    \
            std::cerr << err_msg << std::endl;                                                                         \
            return -1;                                                                                                 \
        }                                                                                                              \
    } while (0)

#define set_ctl_(tof, ctrl, val) check(tof.setControl(CameraCtrl::ctrl, val), "set control(" #ctrl ", " #val ") failed")
#define set_ctl(ctrl, val)       set_ctl_(tof, ctrl, val)

void help(int argc, char* argv[])
{
    std::cout << "Usage: " << argv[0] << " [OPTION]" << std::endl;
    std::cout << "  -h,--help           Display this information" << std::endl;
    std::cout << "  -v,--version        Display the version of the program" << std::endl;
    std::cout << "  --device NUM        Set the device number" << std::endl;
    std::cout << "  --raw/--depth       Display the raw or depth frame" << std::endl;
    std::cout << "  --fps FPS           Set the fps of the camera" << std::endl;
    std::cout << "  --mode MODE         Set the mode of the camera" << std::endl;
    std::cout << "      0               320 * 240 with single frequency" << std::endl;
    std::cout << "      1               320 * 240 with double frequency" << std::endl;
    std::cout << "      2               640 * 480 with single frequency" << std::endl;
    std::cout << "      3(*)            640 * 480 with double frequency (default)" << std::endl;
    std::cout << "  --cfg PATH          The usb camera config file path" << std::endl;
}

enum class ArgEnum {
    none,
    help,
    version,
    device,
    raw,
    depth,
    fps,
    mode,
    cfg,
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
    case ArgEnum::fps:
        return "fps";
    case ArgEnum::mode:
        return "mode";
    case ArgEnum::cfg:
        return "cfg";
    default:
        return "unknown";
    }
}

struct opt_data {
    bool raw = false;
    int device = 0;
    int mode = 3;
    const char* cfg = nullptr;
    int fps = -1;
};

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
                SHORT_END('d', ArgEnum::device)
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

int main(int argc, char* argv[])
{
    opt_data opt;
    int& option = opt.mode;
    for (int i = 1; i < argc;) {
        int tmp = __parse_opt(argc, argv, i, opt);
        if (tmp == 0) {
            std::cerr << "Invalid option: " << argv[i] << std::endl;
            return -1;
        }
        i += tmp;
    }

    Arducam::ArducamTOFCamera tof;
    // if (tof.openWithFile(Arducam::DEPTH_FRAME, "imx316_mcu.cfg", 375e5))
    if (opt.cfg) {
        check(tof.openWithFile(opt.cfg, opt.device), "open camera failed");
    } else {
        check(tof.open(Connection::CSI, opt.device), "open camera failed");
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
            set_ctl(FRAME_MODE, to_int(TofFrameWorkMode::DOUBLE_FREQ_4PHASE_GRAY_4PHASE_BG));
        } else {
            set_ctl(MODE, to_int(TofWorkMode::SINGLE_FREQ));
            set_ctl(FRAME_MODE, to_int(TofFrameWorkMode::SINGLE_FREQ_4PHASE_GRAY));
        }
    }

    if (opt.fps != -1) {
        int max_fps = 0;
        tof.getControl(CameraCtrl::FRAME_RATE, &max_fps);
        if (opt.fps > max_fps) {
            std::cerr << "Invalid fps: " << opt.fps << ", max fps: " << max_fps << std::endl;
            return -1;
        }
        tof.setControl(CameraCtrl::FRAME_RATE, opt.fps);
    }

    if (opt.raw) {
        check(tof.start(FrameType::RAW_FRAME), "start camera failed");
    } else {
        check(tof.start(FrameType::DEPTH_FRAME), "start camera failed");
    }

    // int rang = 0;
    // tof.setControl(CameraCtrl::RANGE, MAX_DISTANCE);
    // tof.getControl(CameraCtrl::RANGE, &rang);
    // std::cout << rang << std::endl;

    auto info = tof.getCameraInfo();
    tof.getControl(CameraCtrl::RANGE, &max_range);
    std::cout << "open camera with (" << info.width << "x" << info.height << ") with range " << max_range << std::endl;

    uint8_t* preview_ptr = new uint8_t[info.width * info.height * 2];

    cv::namedWindow("preview", cv::WINDOW_NORMAL);

    cv::createTrackbar("confidence", "preview", &confidence_value, 255, on_confidence_changed);
    cv::setMouseCallback("preview", onMouse);

    if (opt.raw) {
        for (; raw_loop(tof, preview_ptr);) {
        }
    } else {
        cv::namedWindow("amplitude", cv::WINDOW_NORMAL);
        for (; depth_loop(tof, preview_ptr);) {
        }
    }

    delete[] preview_ptr;

    if (tof.stop()) {
        fprintf(stderr, "stop camera failed\n");
    }
    if (tof.close()) {
        fprintf(stderr, "close camera failed\n");
    }
    return 0;
}
