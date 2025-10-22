#include "test.hpp"

#include <fstream>
#include <iostream>

#define LOCAL static inline

/* #region parse_opt */

LOCAL void help(const char* exec)
{
    std::cout << "Usage: " << exec << " [OPTION]" << std::endl;
    std::cout << "  --help                Display this information" << std::endl;
    std::cout << "  -V,--version          Display the version of the program" << std::endl;
    std::cout << "  -d,--device NUM       Set the device number" << std::endl;
    std::cout << "  --raw/--depth         Display the raw or depth frame" << std::endl;
    std::cout << "  -P,--no-preview       Do not display the preview" << std::endl;
    std::cout << "  -C,--no-confidence    Do not display the confidence" << std::endl;
    std::cout << "  -A,--no-amplitude     Do not display the amplitude" << std::endl;
    std::cout << "  --fps FPS             Set the fps of the camera" << std::endl;
    std::cout << "  --mode MODE           Set the mode of the camera" << std::endl;
    std::cout << "      0                 Near mode" << std::endl;
    std::cout << "      1                 Far mode (default if supported)" << std::endl;
    std::cout << "      -1                Do not set mode" << std::endl;
    std::cout << "  --confidence NUM      Set the confidence value" << std::endl;
    std::cout << "  --exposure NUM        Set the exposure time" << std::endl;
    std::cout << "  -h,--hflip,--h-flip   Enable the horizontal flip" << std::endl;
    std::cout << "  -v,--vflip,--v-flip   Enable the vertical flip" << std::endl;
    std::cout << "  --cfg PATH            The usb camera config file path" << std::endl;
    std::cout << "  -m,--min-range NUM    Set the min range of the camera (mm)" << std::endl;
    std::cout << "  -M,--max-range NUM    Set the max range of the camera (mm)" << std::endl;
    std::cout << "  -E,--no-load-eeprom   Disable loading the eeprom" << std::endl;
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
    confidence_val,
    exposure_val,
    h_flip,
    v_flip,
    cfg,
    min_range,
    max_range,
    no_load_eeprom,
};

template <ArgEnum arg_enum> LOCAL const char* to_str()
{
    switch (arg_enum) {
    case ArgEnum::help:
        return "help";
    case ArgEnum::version:
        return "version";
    case ArgEnum::device:
        return "device number";
    case ArgEnum::raw:
        return "raw";
    case ArgEnum::depth:
        return "depth";
    case ArgEnum::no_preview:
        return "no preview";
    case ArgEnum::no_confidence:
        return "no confidence";
    case ArgEnum::no_amplitude:
        return "no amplitude";
    case ArgEnum::fps:
        return "fps";
    case ArgEnum::mode:
        return "mode";
    case ArgEnum::confidence_val:
        return "confidence";
    case ArgEnum::exposure_val:
        return "exposure";
    case ArgEnum::h_flip:
        return "horizontal flip";
    case ArgEnum::v_flip:
        return "vertical flip";
    case ArgEnum::cfg:
        return "config file path";
    case ArgEnum::min_range:
        return "min range";
    case ArgEnum::max_range:
        return "max range";
    case ArgEnum::no_load_eeprom:
        return "no load eeprom";
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

template <ArgEnum arg_enum> LOCAL int __parse_opt(const char* exec, const char* curr, const char* opt, opt_data& data)
{
    (void)curr;
#define ERR_OPT()                                                                                                      \
    do {                                                                                                               \
        std::cerr << "Invalid " << to_str<arg_enum>() << std::endl;                                                    \
        return 0;                                                                                                      \
    } while (false)
#define CHECK_OPT()                                                                                                    \
    do {                                                                                                               \
        if (opt == nullptr) {                                                                                          \
            ERR_OPT();                                                                                                 \
            return 0;                                                                                                  \
        }                                                                                                              \
    } while (false)
#define ASSERT_OPT(cond)                                                                                               \
    do {                                                                                                               \
        if (!(cond)) {                                                                                                 \
            ERR_OPT();                                                                                                 \
            return 0;                                                                                                  \
        }                                                                                                              \
    } while (false)

    std::cout << "[info] process " << to_str<arg_enum>() << std::endl;
    switch (arg_enum) {
    case ArgEnum::device: {
        CHECK_OPT();
        data.device = atoi(opt);
        return 2;
    } break;
    case ArgEnum::cfg: {
        CHECK_OPT();
        ASSERT_OPT(__parse_cfg(data, opt));
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
    case ArgEnum::no_load_eeprom: {
        data.no_load_cali = true;
    } break;
    case ArgEnum::h_flip: {
        data.h_flip = true;
    } break;
    case ArgEnum::v_flip: {
        data.v_flip = true;
    } break;
    case ArgEnum::confidence_val: {
        CHECK_OPT();
        data.confidence_value = atoi(opt);
        ASSERT_OPT(data.confidence_value >= 0);
        return 2;
    } break;
    case ArgEnum::exposure_val: {
        CHECK_OPT();
        data.exp_time = atoi(opt);
        ASSERT_OPT(data.exp_time >= 0);
        return 2;
    } break;
    case ArgEnum::min_range: {
        CHECK_OPT();
        data.min_range = atoi(opt);
        ASSERT_OPT(data.min_range >= 0);
        return 2;
    } break;
    case ArgEnum::max_range: {
        CHECK_OPT();
        data.max_range = atoi(opt);
        ASSERT_OPT(data.max_range >= 0);
        return 2;
    } break;
    case ArgEnum::mode: {
        CHECK_OPT();
        data.mode = atoi(opt);
        ASSERT_OPT(-1 <= data.mode && data.mode <= 2);
        return 2;
    } break;
    case ArgEnum::fps: {
        CHECK_OPT();
        data.fps = atoi(opt);
        ASSERT_OPT(data.fps >= 0);
        return 2;
    } break;
    case ArgEnum::help: {
        help(exec);
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
    bool is_opt, has_arg = false;
    std::string tmp;
    const char* arg = argv[curr];
    const char* next = curr + 1 < argc ? argv[curr + 1] : nullptr;
    // if start with "--"
    if (arg[0] != '-')
        return 0;
    if (arg[1] == '-') {
        is_opt = true;
        arg += 2;
        auto eq_op = strchr(arg, '='); // find the equal sign
        if (eq_op != nullptr) {
            has_arg = true;
            tmp = std::string(arg, eq_op);
            arg = tmp.c_str();
            next = eq_op + 1;
        }
    } else {
        is_opt = false;
        arg += 1;
    }

// parse a short option
#define SHORT(chr, type)                                                                                               \
    case chr:                                                                                                          \
        if (1 != __parse_opt<type>(argv[0], arg, next, data)) {                                                        \
            return 0;                                                                                                  \
        }                                                                                                              \
        break;
// parse a short option with argument
#define SHORT_END(chr, type)                                                                                           \
    case chr:                                                                                                          \
        if (arg[index + 1] != '\0') {                                                                                  \
            return __parse_opt<type>(argv[0], arg, arg + index + 1, data) == 0 ? 0 : 1;                                \
        }                                                                                                              \
        return __parse_opt<type>(argv[0], arg, next, data);                                                            \
        break;
// parse a long option
#define LONG(chr, type)                                                                                                \
    else if (!strcmp(arg, chr))                                                                                        \
    {                                                                                                                  \
        auto ret = __parse_opt<type>(argv[0], arg, next, data);                                                        \
        return (has_arg && ret > 1) ? 1 : ret;                                                                         \
    }

    if (!is_opt) {
        for (int index = 0; arg[index] != '\0'; index++) {
            switch (arg[index]) {
                // SHORT('r', ArgEnum::raw)
                SHORT('h', ArgEnum::h_flip)
                SHORT('v', ArgEnum::v_flip)
                SHORT('V', ArgEnum::version)
                SHORT('P', ArgEnum::no_preview)
                SHORT('C', ArgEnum::no_confidence)
                SHORT('A', ArgEnum::no_amplitude)
                SHORT('E', ArgEnum::no_load_eeprom)
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
        LONG("confidence", ArgEnum::confidence_val)
        LONG("exposure", ArgEnum::exposure_val)
        LONG("hflip", ArgEnum::h_flip)
        LONG("h-flip", ArgEnum::h_flip)
        LONG("vflip", ArgEnum::v_flip)
        LONG("v-flip", ArgEnum::v_flip)
        LONG("no-preview", ArgEnum::no_preview)
        LONG("no-confidence", ArgEnum::no_confidence)
        LONG("no-amplitude", ArgEnum::no_amplitude)
        LONG("no-load-eeprom", ArgEnum::no_load_eeprom)
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
#undef SHORT_END
#undef LONG

    return 1;
}

bool parse_opt(int argc, char* argv[], opt_data& opt)
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
